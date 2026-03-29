# DOPE Engine — Mathematical Reference

> **How to use this document**
>
> Every formula here is linked bidirectionally to source code via `[MATH §N.M]` tags.
> When you modify a tagged line, open this document and update the corresponding section.
> When you modify a section here, grep for the tag in source and update the code to match.
>
> Quick-find: `grep -rn "\[MATH §" lib/`

---

## Table of Contents

1. [Overview & Module Map](#1-overview--module-map)
2. [Coordinate System & Sign Conventions](#2-coordinate-system--sign-conventions)
3. [Atmospheric Model — Density & Speed of Sound](#3-atmospheric-model--density--speed-of-sound)
4. [Ballistic Coefficient Correction (4-Factor)](#4-ballistic-coefficient-correction-4-factor)
5. [Drag Deceleration](#5-drag-deceleration)
6. [AHRS — Attitude & Heading Reference](#6-ahrs--attitude--heading-reference)
7. [Euler Angles from Quaternion](#7-euler-angles-from-quaternion)
8. [Zero-Angle Binary Search](#8-zero-angle-binary-search)
9. [Barrel-Length MV Adjustment](#9-barrel-length-mv-adjustment)
10. [Wind Decomposition](#10-wind-decomposition)
11. [RK4 Trajectory Integrator](#11-rk4-trajectory-integrator)
12. [Spin Drift](#12-spin-drift)
13. [Coriolis & Eötvös Effects](#13-coriolis--eötvös-effects)
14. [MOA Hold Computation](#14-moa-hold-computation)
15. [Cant Correction](#15-cant-correction)
16. [Uncertainty Propagation](#16-uncertainty-propagation)
- [16.4 Barrel Attachment & Thermal Dispersion](#164-barrel-attachment--thermal-dispersion)
- [Appendix A — Constants & Unit Reference](#appendix-a--constants--unit-reference)

---

## 1. Overview & Module Map

The DOPE (Digital Optical Performance Engine) is a single-instance, zero-heap-allocation C++ engine. It is called once per sensor frame via `DOPE_Update()` and outputs a `FiringSolution` containing MOA hold values, ballistic data, and uncertainty estimates.

### Data In, Data Out, and What Happens Inside

The following diagram illustrates the high-level data flow across the DOPE engine boundary:

```text
┌─────────────────────────┐         ┌─────────────────────────┐         ┌─────────────────────────┐
│    Host (DOPE-ASS)      │         │   DOPE Core Engine      │         │     Application UI      │
│                         │         │                         │         │                         │
│  [Raw Sensors]          │         │  [Atmospheric Trans.]   │    /--->│  [FiringSolution]       │
│        │                │    /--->│          │              │    |    │   - Hold (MOA)          │
│        ▼                |    |    │          ▼              │    |    │   - Uncertainty         │
│  [Sensor Fusion]        │    |    │  [Mathematical Zero]    │    |    │   - Velocity/Energy     │
│  [& Calibration]        │    |    │          │              │    |    │                         │
│        │                │    |    │          ▼              │    |    │                         │
│        ▼                │    |    │  [Ballistic Solver]     │    |    │                         │
│  [SensorFrame payload]  │    |    │   (Table-first/RK4)     │----/    │                         │
│   (Quat, Range, etc.)   │----/    │                         │         │                         │
└─────────────────────────┘         └─────────────────────────┘         └─────────────────────────┘
```

### Internal Call Graph (V2 Boundary)

```
DOPE_Update(SensorFrame*)
    │
    ├─ §3,4  Atmosphere ──── pressure, temperature, humidity
    │         ├─ air density ρ
    │         ├─ speed of sound c
    │         └─ corrected BC
    │
    ├─ §8    Zero solver ─── binary search for bore elevation at zero range
    │
    ├─ §9    buildSolverParams ─ barrel MV adj, §4 BC, §10 wind decompose
    │
    └─ §11   RK4 Integrator ─ trajectory to target range
              ├─ §5   drag deceleration
              ├─ §12  spin drift
              ├─ §13  Coriolis / Eötvös
              ├─ §14  drop & windage → MOA
              ├─ §15  cant correction
              └─ §16  uncertainty propagation
```

> **Note on V2 Architecture:** In DOPE v2.0, sensor processing (AHRS fusion, Magnetometer Calibration, LRF filtering) has been moved to the DOPE-ASS application boundary. DOPE now receives pre-calculated quaternions and ranges.

---

## 2. Coordinate System & Sign Conventions

The trajectory integrator ([lib/dope/src/solver/solver.cpp](lib/dope/src/solver/solver.cpp)) uses a right-handed, bore-centred frame:

| Axis | Direction | Sign convention |
|------|-----------|----------------|
| **X** | Downrange (horizontal) | Positive toward target |
| **Y** | Vertical | Positive up |
| **Z** | Lateral | Positive to the right (from shooter's perspective) |

**Angular conventions:**

| Angle | Definition | Positive direction |
|-------|-----------|-------------------|
| Pitch | Rotation about lateral axis (nose up/down) | Nose up |
| Roll | Rotation about bore axis | Right wing down |
| Yaw | Rotation about vertical axis | Clockwise from north |
| Wind heading | Direction wind comes *from* | True north = 0°, clockwise |

**Output sign convention:** Positive elevation MOA means the point of impact is above point of aim; a positive hold means dialling *up*. Positive windage means the impact is to the right; a positive hold means dialling *right*. The solver outputs negative Y for a bullet that has fallen below bore axis.

---

## 3. Atmospheric Model — Density & Speed of Sound

**Source:** [lib/dope/src/atmo/atmosphere.cpp](lib/dope/src/atmo/atmosphere.cpp), `Atmosphere::recompute()`.

### 3.1 Saturation Vapor Pressure — Buck Equation

`[MATH §3.1]`

The saturation vapor pressure of water at temperature $T_C$ (°C) is computed via the Buck equation:

$$e_{sat} = 611.21 \cdot \exp\!\left[\left(18.678 - \frac{T_C}{234.5}\right)\frac{T_C}{257.14 + T_C}\right] \quad \text{(Pa)}$$

The partial vapor pressure actually present in the air is then:

$$e_{vapor} = h \cdot e_{sat}$$

where $h$ is relative humidity as a fraction $[0, 1]$.

> **Sync:** If the Buck equation coefficients (18.678, 234.5, 257.14) or the leading constant (611.21) change, update this section.
> **Implementation:** `atmosphere.cpp` lines tagged `[MATH §3.1]`

### 3.2 Virtual Temperature

`[MATH §3.2]`

The virtual temperature accounts for the reduced density of moist air relative to dry air at the same pressure and temperature:

$$T_v = T_K \cdot \left(1 + \frac{0.378 \cdot e_{vapor}}{P}\right) \quad \text{(K)}$$

where $T_K = T_C + 273.15$ is the temperature in Kelvin and $P$ is absolute pressure in Pa.

The factor 0.378 is the ratio of the gas constant of dry air to that of water vapor ($R_{dry}/R_{water} \approx 0.622$), expressed as $(1 - 0.622)/1 = 0.378$.

> **Sync:** If the constant 0.378 changes, update this section.
> **Implementation:** `atmosphere.cpp` lines tagged `[MATH §3.2]`

### 3.3 Air Density

`[MATH §3.3]`

Using the ideal gas law for moist air with virtual temperature:

$$\rho = \frac{P}{R_{dry} \cdot T_v} = \frac{P}{287.05 \cdot T_v} \quad \text{(kg/m³)}$$

> **Sync:** If $R_{dry}$ (287.05 J/kg·K) or the formula form changes, update §3.3.
> **Implementation:** `atmosphere.cpp` lines tagged `[MATH §3.3]`

### 3.4 Speed of Sound

`[MATH §3.4]`

The speed of sound in moist air is approximated using the virtual temperature:

$$c = 20.05 \cdot \sqrt{T_v} \quad \text{(m/s)}$$

The constant 20.05 is derived from $\sqrt{\gamma \cdot R_{dry}} = \sqrt{1.4 \times 287.05} \approx 20.05$.

> **Sync:** If the speed-of-sound formula or constant changes, update §3.4.
> **Implementation:** `atmosphere.cpp` lines tagged `[MATH §3.4]`

---

## 4. Ballistic Coefficient Correction (4-Factor)

**Source:** [lib/dope/src/atmo/atmosphere.cpp](lib/dope/src/atmo/atmosphere.cpp), `Atmosphere::correctBC()`.

Reference: Litz, *Applied Ballistics for Long Range Shooting*; US Army Metro standard.

The standard BC is derived at Army Metro conditions (sea level, 59 °F, 29.53 inHg). All four factors use **imperial units** internally to match the reference formulas.

Let:
- $h_{ft}$ = altitude in feet
- $T_F$ = temperature in °F
- $P_{inHg}$ = pressure in inches of mercury
- $RH\%$ = relative humidity in percent ($= h \times 100$)
- Reference values: $T_{ref} = 59$ °F, $P_{ref} = 29.53$ inHg

### 4.1 Altitude Factor $F_A$

`[MATH §4.1]`

$$F_A = 1 - 3.158 \times 10^{-5} \cdot h_{ft}$$

Clamped to a minimum of 0.5 at extreme altitudes. This is a linear approximation to the ISA density–altitude relationship.

### 4.2 Temperature Factor $F_T$

`[MATH §4.2]`

$$F_T = \frac{T_F - T_{ref}}{T_{ref} + 460} = \frac{T_F - 59}{519}$$

$F_T > 0$ when warmer than standard (hot air is less dense → bullet retards less → effective BC increases).

### 4.3 Pressure Factor $F_P$

`[MATH §4.3]`

$$F_P = \frac{P_{ref} - P_{inHg}}{P_{ref}} = \frac{29.53 - P_{inHg}}{29.53}$$

$F_P > 0$ when pressure is below standard; lower pressure means lower density, again increasing effective BC.

### 4.4 Humidity Factor $F_R$

`[MATH §4.4]`

$$F_R = 1 + 0.00002 \cdot (RH\% - 50)$$

A small linear correction: moist air is slightly less dense than dry air at the same pressure.

### 4.5 Combined Correction

`[MATH §4.5]`

$$BC_{corrected} = BC_{std} \cdot F_A \cdot (1 + F_T - F_P) \cdot F_R$$

Minimum clamped to 0.01. The $F_T$ and $F_P$ factors appear together as $(1 + F_T - F_P)$ because temperature and pressure have opposite effects on density.

> **Sync:** Any change to the factor formulas or their combination must be reflected in §4.1–§4.5.
> **Implementation:** `atmosphere.cpp` lines tagged `[MATH §4.1]`–`[MATH §4.5]`

---

## 5. Drag Deceleration

**Source:** [lib/dope/src/drag/drag_model.cpp](lib/dope/src/drag/drag_model.cpp), [lib/dope/src/drag/drag_tables.h](lib/dope/src/drag/drag_tables.h).

### 5.1 Drag Table Lookup

`[MATH §5.1]`

For each standard drag model (G1–G8), a `constexpr` table of $(M_i, C_{d,i})$ pairs is stored in flash/ROM (no RAM cost). Given a Mach number $M$:

1. Binary search for the index $j$ such that $M_j \le M < M_{j+1}$.
2. Linear interpolation:

$$C_d(M) = C_{d,j} + \frac{M - M_j}{M_{j+1} - M_j} \cdot (C_{d,j+1} - C_{d,j})$$

Values outside the table range are clamped to the nearest endpoint.

> **Sync:** If a drag table is updated or re-tabulated, ensure the section of `drag_tables.h` containing that model's data is updated, and this section notes the model summary has changed.
> **Implementation:** `drag_model.cpp` lines tagged `[MATH §5.1]`

### 5.2 Retardation Formula

`[MATH §5.2]`

The drag deceleration (retardation) of the bullet is:

$$a_{drag} = \frac{C_d(M) \cdot (\rho / \rho_{std}) \cdot v^2}{BC_{corrected} \cdot K_{bal}}$$

where:
- $C_d(M)$ = drag coefficient at current Mach number (from §5.1)
- $\rho$ = current air density (kg/m³, from §3.3)
- $\rho_{std} = 1.225$ kg/m³ (ISA sea-level standard, `DOPE_STD_AIR_DENSITY`)
- $v$ = bullet speed relative to surrounding air (m/s)
- $BC_{corrected}$ = atmospherically-corrected ballistic coefficient (from §4.5)
- $K_{bal} = 900$ (`dope::math::BALLISTIC_DRAG_CONSTANT`) — the legacy ballistic drag constant that maps BC (lbf/in²) to SI retardation units

The result $a_{drag}$ has units of m/s².

**Physical interpretation:** $C_d/BC_{corrected}$ is proportional to the drag-to-inertia ratio of the projectile relative to the reference G-model projectile. Multiplying by $\rho/\rho_{std}$ scales for actual air density; multiplying by $v^2$ gives the characteristic drag force per unit mass (scaled by $K_{bal}$).

> **Sync:** If $K_{bal}$, $\rho_{std}$, or the formula structure changes, update §5.2 and `dope::math::BALLISTIC_DRAG_CONSTANT` in `dope_math_utils.h`.
> **Implementation:** `drag_model.cpp` lines tagged `[MATH §5.2]`

---

## 6. AHRS — Attitude & Heading Reference

> **V2 Boundary Note:** As part of the DOPE v2.0 migration, all raw sensor fusion (Mahony/Madgwick) and magnetometer calibration algorithms are being relocated to the DOPE-ASS application layer. This section describes the mathematical theory for these filters, which the host application uses to produce the orientation quaternion fed into the `SensorFrame`.

**Source (pending migration check):** [lib/dope/src/ahrs/madgwick.cpp](lib/dope/src/ahrs/madgwick.cpp), [lib/dope/src/ahrs/mahony.cpp](lib/dope/src/ahrs/mahony.cpp).

Both filters maintain an orientation **quaternion** $\mathbf{q} = [q_0, q_1, q_2, q_3]$ (scalar-first, i.e., $[w, x, y, z]$). The quaternion is always kept unit-normalized: $|\mathbf{q}| = 1$.

---

### 6.1 Quaternion Kinematics — Gyroscope Integration

`[MATH §6.1]`

Given angular velocity $\boldsymbol{\omega} = [g_x, g_y, g_z]$ (rad/s, body frame), the rate of change of the orientation quaternion is:

$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix}0 \\ g_x \\ g_y \\ g_z\end{bmatrix}$$

Expanding the quaternion product:

$$\dot{q}_0 = \frac{1}{2}(-q_1 g_x - q_2 g_y - q_3 g_z)$$

$$\dot{q}_1 = \frac{1}{2}(+q_0 g_x + q_2 g_z - q_3 g_y)$$

$$\dot{q}_2 = \frac{1}{2}(+q_0 g_y - q_1 g_z + q_3 g_x)$$

$$\dot{q}_3 = \frac{1}{2}(+q_0 g_z + q_1 g_y - q_2 g_x)$$

These four equations are how sensor gyro measurements are integrated into the orientation estimate. Both Madgwick and Mahony start from this same base.

> **Sync:** If sensor axis mapping or sign conventions change, update §6.1.
> **Implementation:** `madgwick.cpp` and `mahony.cpp` lines tagged `[MATH §6.1]`/`[MATH §6.5]`

---

### 6.2 Madgwick — Earth Magnetic Field Reference

`[MATH §6.2]`

Reference: S. Madgwick, "An efficient orientation filter for inertial and inertial/magnetic sensor arrays," 2010.

When the magnetometer is active, the filter first rotates the calibrated magnetometer reading $\hat{\mathbf{m}} = [m_x, m_y, m_z]$ into the Earth frame using the current quaternion estimate:

$$h_x = m_x(q_0^2 + q_1^2 - q_2^2 - q_3^2) + 2m_y(q_1 q_2 - q_0 q_3) + 2m_z(q_1 q_3 + q_0 q_2)$$

$$h_y = 2m_x(q_1 q_2 + q_0 q_3) + m_y(q_0^2 - q_1^2 + q_2^2 - q_3^2) + 2m_z(q_2 q_3 - q_0 q_1)$$

The horizontal component of Earth's field is:

$$b_x = \sqrt{h_x^2 + h_y^2}, \qquad b_z = 2m_x(q_1 q_3 - q_0 q_2) + 2m_y(q_2 q_3 + q_0 q_1) + m_z(q_0^2 - q_1^2 - q_2^2 + q_3^2)$$

This forces the horizontal field component to point only along the X-axis of the Earth frame, which is how the filter locks yaw to magnetic north.

> **Sync:** If the magnetometer rotation convention or Earth field decomposition changes, update §6.2.
> **Implementation:** `madgwick.cpp` lines tagged `[MATH §6.2]`

---

### 6.3 Madgwick — Gradient Descent Corrective Step

`[MATH §6.3]`

The filter minimizes the objective function:

$$f(\mathbf{q}) = \mathbf{J}^T \mathbf{f}_{obj}$$

where $\mathbf{f}_{obj}$ is the vector of residuals between estimated and measured sensor directions. The normalized gradient descent step $\hat{\mathbf{s}} = \nabla f / |\nabla f|$ is computed analytically.

**IMU-only (accel vs. gravity, 4 equations):**

$$s_0 = 4q_0 q_2^2 + 2q_2 a_x + 4q_0 q_1^2 - 2q_1 a_y$$

$$s_1 = 4q_1 q_3^2 - 2q_3 a_x + 4q_0^2 q_1 - 2q_0 a_y - 4q_1 + 8q_1^3 + 8q_1 q_2^2 + 4q_1 a_z$$

$$s_2 = 4q_0^2 q_2 + 2q_0 a_x + 4q_2 q_3^2 - 2q_3 a_y - 4q_2 + 8q_2 q_1^2 + 8q_2^3 + 4q_2 a_z$$

$$s_3 = 4q_1^2 q_3 - 2q_1 a_x + 4q_2^2 q_3 - 2q_2 a_y$$

where $[a_x, a_y, a_z]$ is the unit-normalized accelerometer reading (the measured gravity direction in body frame).

**9-DOF (accel + mag):** An additional 12 terms per component encode the magnetometer residual using the $b_x, b_z$ reference computed in §6.2. The full form is in `madgwick.cpp`.

In both cases, $\hat{s}_i = s_i / |\mathbf{s}|$.

> **Sync:** The gradient expressions are derived analytically from the objective function. Any change to the sensor model (e.g., different Earth field vector structure) requires re-deriving these expressions.
> **Implementation:** `madgwick.cpp` lines tagged `[MATH §6.3]`

---

### 6.4 Madgwick — Feedback Application

`[MATH §6.4]`

The gradient step corrects the gyro-propagated $\dot{\mathbf{q}}$ before integration:

$$\dot{\mathbf{q}}_{corrected} = \dot{\mathbf{q}}_{gyro} - \beta \cdot \hat{\mathbf{s}}$$

where $\beta = 0.1$ (engine built-in default; runtime value supplied via `DOPE_AHRSConfig::madgwick_beta` in `DOPE_SetAHRSConfig()`) controls the aggressiveness of the correction (larger $\beta$ → trusts accelerometer/magnetometer more, less gyro drift, more noise).

> **Sync:** If the default $\beta$ changes, update the built-in default in `dope_engine.cpp` `init()` and this section.
> **Implementation:** `madgwick.cpp` lines tagged `[MATH §6.4]`

---

### 6.5 Quaternion Integration (both filters)

`[MATH §6.5]`

After computing the corrected $\dot{\mathbf{q}}$, the quaternion is updated by first-order Euler integration:

$$\mathbf{q}_{new} = \mathbf{q} + \dot{\mathbf{q}} \cdot dt$$

then immediately re-normalized:

$$\mathbf{q} \leftarrow \mathbf{q}_{new} / |\mathbf{q}_{new}|$$

> **Sync:** Both `madgwick.cpp` and `mahony.cpp` use this identical integration step.
> **Implementation:** `madgwick.cpp` and `mahony.cpp` lines tagged `[MATH §6.5]`

---

### 6.6–6.9 Mahony Filter

Reference: R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear Complementary Filters on the Special Orthogonal Group," IEEE TAC, 2008.

The Mahony filter replaces gradient descent with a PI (proportional-integral) feedback controller on the attitude error.

**6.6 Error Initialization** `[MATH §6.6]`

$$\mathbf{e} = \mathbf{0}$$

Error will accumulate from the cross-product comparisons below.

---

**6.7 Estimated Gravity Direction** `[MATH §6.7]`

The current quaternion predicts what direction gravity should appear in the body frame. From $\mathbf{q} = [q_0, q_1, q_2, q_3]$:

$$v_x = 2(q_1 q_3 - q_0 q_2)$$

$$v_y = 2(q_0 q_1 + q_2 q_3)$$

$$v_z = q_0^2 - q_1^2 - q_2^2 + q_3^2$$

This is the third column of the rotation matrix $\mathbf{R}(\mathbf{q})$, which maps Earth's downward gravity $[0, 0, 1]$ into the body frame.

---

**6.8 Accel Cross-Product Error** `[MATH §6.8]`

The error vector is the cross product between the measured (normalized) acceleration $\hat{\mathbf{a}}$ and the predicted gravity direction $\mathbf{v}$:

$$\mathbf{e}_{accel} = \hat{\mathbf{a}} \times \mathbf{v}$$

$$e_x \mathrel{+}= a_y v_z - a_z v_y, \quad e_y \mathrel{+}= a_z v_x - a_x v_z, \quad e_z \mathrel{+}= a_x v_y - a_y v_x$$

When the magnetometer is active, an analogous cross product $\hat{\mathbf{m}} \times \hat{\mathbf{w}}$ is added to $\mathbf{e}$, where $\hat{\mathbf{w}}$ is the predicted magnetic field direction.

---

**6.9 PI Feedback** `[MATH §6.9]`

The error $\mathbf{e}$ feeds a PI controller that corrects the angular rate before integration:

$$\boldsymbol{i} \mathrel{+}= k_i \cdot \mathbf{e} \cdot dt \quad \text{(integral accumulator)}$$

$$\boldsymbol{\omega}_{corrected} = \boldsymbol{\omega}_{gyro} + k_p \cdot \mathbf{e} + \boldsymbol{i}$$

where:
- $k_p = 2.0$ (engine built-in default; runtime value via `DOPE_AHRSConfig::mahony_kp` in `DOPE_SetAHRSConfig()`) — proportional gain
- $k_i = 0.005$ (engine built-in default; runtime value via `DOPE_AHRSConfig::mahony_ki` in `DOPE_SetAHRSConfig()`) — integral gain (removes gyro bias drift)

The corrected $\boldsymbol{\omega}_{corrected}$ is then used in the quaternion kinematics of §6.1 → §6.5.

> **Sync:** If $k_p$ or $k_i$ defaults change, update the built-in defaults in `dope_engine.cpp` `init()` and §6.9.
> **Implementation:** `mahony.cpp` lines tagged `[MATH §6.6]`–`[MATH §6.9]`

---

## 7. Euler Angles from Quaternion

**Source:** [lib/dope/src/ahrs/ahrs_interface.h](lib/dope/src/ahrs/ahrs_interface.h).

The quaternion $\mathbf{q} = [q_0, q_1, q_2, q_3]$ (scalar $q_0$, vector $q_1, q_2, q_3$) encodes orientation using the ZYX (yaw-pitch-roll) Euler convention.

### 7.1 Pitch

`[MATH §7.1]`

$$\theta_{pitch} = \arcsin\!\bigl(2(q_0 q_2 - q_3 q_1)\bigr) \quad \text{(rad, nose up positive)}$$

The argument is clamped to $[-1, 1]$ before `asin` to guard against numerical noise exceeding unity.

### 7.2 Roll

`[MATH §7.2]`

$$\phi_{roll} = \text{atan2}\!\bigl(2(q_0 q_1 + q_2 q_3),\; 1 - 2(q_1^2 + q_2^2)\bigr) \quad \text{(rad, right wing down positive)}$$

### 7.3 Yaw

`[MATH §7.3]`

$$\psi_{yaw} = \text{atan2}\!\bigl(2(q_0 q_3 + q_1 q_2),\; 1 - 2(q_2^2 + q_3^2)\bigr) \quad \text{(rad, clockwise from north positive)}$$

Yaw is passed through the magnetic heading computation to become the true heading used everywhere else in the engine:

$$\text{heading}_{true} = \psi_{yaw} \cdot \frac{180}{\pi} + \delta_{declination}, \quad \text{normalized to } [0°, 360°)$$

> **Sync:** If the quaternion-to-Euler convention changes (e.g., body-frame axis ordering), all three formulas change and must be re-derived.
> **Implementation:** `ahrs_interface.h` lines tagged `[MATH §7.1]`–`[MATH §7.3]`

---

## 8. Zero-Angle Binary Search

**Source:** [lib/dope/src/solver/solver.cpp](lib/dope/src/solver/solver.cpp), `BallisticSolver::solveZeroAngle()`.

The "zero angle" $\theta_0$ is the bore elevation that makes the bullet intersect the line of sight at the zero range $R_0$.

**Geometry:** The optic is mounted $h_s$ above the bore. The line of sight (LOS) is a straight line from the optic to the target. At range $x$, the LOS height above bore axis is:

$$y_{LOS}(x) = h_s \cdot \left(1 - \frac{x}{R_0}\right)$$

So at $x = R_0$: $y_{LOS}(R_0) = 0$. With a flat target plane we target $y_{bullet}(R_0) = 0$ relative to bore (LOS crosses bore at the zero range).

**Search:** Binary search over $\theta \in [-5°, +5°]$, tolerance $= 1$ mm, max 50 iterations:

1. Set $\theta_{mid} = (\theta_{lo} + \theta_{hi}) / 2$
2. Integrate trajectory with launch angle $\theta_{mid}$
3. If $y(\text{at } R_0) > 0$: bullet is too high → $\theta_{hi} = \theta_{mid}$
4. If $y(\text{at } R_0) < 0$: bullet is too low → $\theta_{lo} = \theta_{mid}$
5. Converge when $|y| < 0.001$ m

If the search fails to converge, `ZERO_UNSOLVABLE` fault is raised.

The firing angle at any target range is:

$$\theta_{fire} = \theta_0 + \theta_{pitch}$$

where $\theta_{pitch}$ is the current measured bore pitch from AHRS (§7.1).

> **Sync:** If the zero geometry (e.g., angular line of sight rather than straight-line) changes, re-derive the target drop formula and update §8.
> **Implementation:** `solver.cpp`, `solveZeroAngle()`

---

## 9. Barrel-Length MV Adjustment

**Source:** [lib/dope/src/engine/dope_engine.cpp](lib/dope/src/engine/dope_engine.cpp), `DOPE_Engine::buildSolverParams()`.

`[MATH §9]`

Muzzle velocity data is typically measured at a reference barrel length $L_{ref}$ (default 24 inches if unspecified). When the actual barrel $L_{actual}$ differs from $L_{ref}$, the MV is linearly adjusted:

$$MV_{adj,fps} = MV_{base,fps} + (L_{actual,in} - L_{ref,in}) \cdot \Delta_{fps/in}$$

$$MV_{adj,m/s} = MV_{adj,fps} \times 0.3048$$

where:
- $MV_{base,fps} = MV_{base,m/s} \times 3.28084$ is the nominal MV converted to fps
- $\Delta_{fps/in} = |\texttt{mv\_adjustment\_factor}|$ — the user-supplied fps-per-inch sensitivity (always treated as positive magnitude; the direction is determined by sign of $L_{actual} - L_{ref}$)
- Positive delta = longer barrel → higher MV; negative = shorter barrel → lower MV

> **Sync:** If the barrel adjustment formula or unit convention changes, update §9.
> **Implementation:** `dope_engine.cpp` lines tagged `[MATH §9]`

---

## 10. Wind Decomposition

**Source:** [lib/dope/src/corrections/wind.cpp](lib/dope/src/corrections/wind.cpp), `WindCorrection::decompose()`.

`[MATH §10]`

Wind is specified as a speed $v_w$ (m/s) and a heading $\alpha_w$ (degrees true north), defined as the direction the wind blows **from**. The firing azimuth is $\alpha_{fire}$ (degrees true north).

The relative angle is:

$$\phi = (\alpha_w - \alpha_{fire}) \cdot \frac{\pi}{180}$$

The wind components in the bore-aligned frame:

$$v_{head} = v_w \cdot \cos\phi \quad \text{(headwind, positive = into face = retards bullet)}$$

$$v_{cross} = v_w \cdot \sin\phi \quad \text{(crosswind, positive = right-to-left = deflects bullet left)}$$

These feed directly into the integrator's relative velocity computation (§11.2).

> **Sync:** If the sign convention for headwind or crosswind changes, update §10 and §11.2 together.
> **Implementation:** `wind.cpp` lines tagged `[MATH §10]`

---

## 11. RK4 Trajectory Integrator

**Source:** [lib/dope/src/solver/solver.cpp](lib/dope/src/solver/solver.cpp), `BallisticSolver::integrateToRange()`.

The trajectory is a 3-DOF point-mass simulation integrating the bullet's position $[x, y, z]$ and velocity $[v_x, v_y, v_z]$ over time.

### 11.1 Initial Conditions

The bullet starts at the muzzle (bore origin):

$$x_0 = y_0 = z_0 = 0, \quad t_0 = 0$$

$$v_{x,0} = MV_{adj} \cdot \cos(\theta_{fire}), \quad v_{y,0} = MV_{adj} \cdot \sin(\theta_{fire}), \quad v_{z,0} = 0$$

### 11.2 Relative Velocity

`[MATH §11.2]`

Drag acts on the bullet's velocity **relative to the air mass**. Headwind adds directly to the bullet's apparent forward speed; crosswind adds to the lateral component:

$$v_{x,rel} = v_x + v_{head}$$

$$v_{z,rel} = v_z + v_{cross}$$

$$v_{rel} = \sqrt{v_{x,rel}^2 + v_y^2 + v_{z,rel}^2}$$

Note: $v_y$ (vertical) has no wind correction; vertical wind is not modelled.

> **Sync:** If vertical wind is added, update this formula and the accelerations in §11.3.
> **Implementation:** `solver.cpp` lines tagged `[MATH §11.2]`

### 11.3 Acceleration Kernel

`[MATH §11.3]`

At each sub-step, the three acceleration components are:

$$a_x = -a_{drag} \cdot \frac{v_{x,rel}}{v_{rel}} \cdot \kappa$$

$$a_y = -a_{drag} \cdot \frac{v_y}{v_{rel}} \cdot \kappa - g$$

$$a_z = -a_{drag} \cdot \frac{v_{z,rel}}{v_{rel}} \cdot \kappa$$

where:
- $a_{drag}$ = retardation magnitude from §5.2 (m/s²)
- $\kappa$ = `drag_reference_scale` × `stability_drag_scale`
- $g = 9.80665$ m/s² (`DOPE_GRAVITY`)

`stability_drag_scale` is a bounded SG-coupled correction derived from the dynamic stability estimate in §12. Lower $S_G$ slightly increases effective drag; higher $S_G$ slightly decreases it. The implementation keeps this as a small correction so BC remains the dominant drag driver.

Current default coupling uses:
$$\text{stability\_drag\_scale} = \text{clamp}\bigl(1 + 0.01\,(1.5 - S_G),\;0.985,\;1.04\bigr)$$

Gravity acts only in the $-Y$ direction. Drag acts opposite to the relative velocity direction.

> **Sync:** If gravity constant changes, update §11.3 and `DOPE_GRAVITY` in `dope_config.h`.
> **Implementation:** `solver.cpp` lines tagged `[MATH §11.3]`

### 11.4 Adaptive Timestep

`[MATH §11.4]`

The timestep $dt$ is chosen to balance accuracy and performance:

$$dt = \begin{cases} dt_{min} & \text{if } 0.9 < M < 1.2 \quad \text{(transonic — use finest step)}\\ \min\!\left(\dfrac{0.5}{v},\; \dfrac{\Delta x_{max}}{v}\right) & \text{otherwise} \end{cases}$$

Final clamp: $dt_{min} \le dt \le dt_{max}$, where $dt_{min} = 10\;\mu\text{s}$, $dt_{max} = 1\;\text{ms}$, $\Delta x_{max} = 0.25\;\text{m}$.

The 0.5/v formula ensures each step covers at most 0.5 m at the current speed. The transonic override (M ∈ [0.9, 1.2]) uses the minimum step because $C_d$ changes most rapidly near Mach 1.

> **Sync:** If any of the adaptive timestep constants change, update §11.4 and the corresponding `DOPE_DT_*` / `DOPE_MAX_STEP_DISTANCE_M` constants in `dope_config.h`.
> **Implementation:** `solver.cpp` lines tagged `[MATH §11.4]`

### 11.5 RK4 Integration Step

`[MATH §11.5]`

For each axis, the classic 4th-order Runge-Kutta update:

Let $f(\mathbf{v})$ be the acceleration kernel (§11.3). Define intermediate slopes:

$$k_1 = f(\mathbf{v}_n), \qquad k_2 = f\!\left(\mathbf{v}_n + \tfrac{dt}{2} k_1\right)$$

$$k_3 = f\!\left(\mathbf{v}_n + \tfrac{dt}{2} k_2\right), \qquad k_4 = f\!\left(\mathbf{v}_n + dt\, k_3\right)$$

Update:

$$\mathbf{v}_{n+1} = \mathbf{v}_n + \frac{dt}{6}(k_1 + 2k_2 + 2k_3 + k_4)$$

$$\mathbf{x}_{n+1} = \mathbf{x}_n + \frac{dt}{6}(u_1 + 2u_2 + 2u_3 + u_4)$$

where $u_i = \mathbf{v}$ at each sub-step position (since $\dot{\mathbf{x}} = \mathbf{v}$).

**Termination conditions:**
- $x \ge R_{target}$ (target range reached)
- $|\mathbf{v}| < 30$ m/s (`DOPE_MIN_VELOCITY`) — bullet subsided
- Iterations $\ge 500{,}000$ (`DOPE_MAX_SOLVER_ITERATIONS`) — runaway guard

At each integer metre of downrange distance, a `TrajectoryPoint` record is written to the static table with: drop, windage, speed, time of flight, and kinetic energy. Values are linearly interpolated between the previous and current RK4 states so the table reflects the state at the exact metre mark (not just the end of the integration step). The final returned drop at the requested target range is likewise interpolated between the bracketing step endpoints.

> **Sync:** Any change to the RK4 order or termination conditions must update §11.5.
> **Implementation:** `solver.cpp` lines tagged `[MATH §11.5]`

---

## 12. Spin Drift

**Source:** [lib/dope/src/solver/solver.cpp](lib/dope/src/solver/solver.cpp), `BallisticSolver::integrate()`.

`[MATH §12]`

Spin drift is a gyroscopic effect: a spinning projectile precesses in the direction of rifling twist due to the overturning moment from aerodynamic drag. This causes a persistent lateral drift growing with time of flight.

The Litz approximation is used with a dynamic stability factor estimate:

$$\delta_{drift,m} = 0.0254 \times 1.25 \times (S_G + 1.2) \times TOF^{1.83}$$

$$S_G \approx \frac{30 m}{t^2 d^3 l (1+l^2)} \cdot \left(\frac{v}{2800}\right)^{1/3} \cdot \frac{\rho_{std}}{\rho}$$

where $m$ is bullet mass (grains), $d$ caliber (inches), $l$ length in calibers, $t$ twist in calibers/turn, and $v$ is velocity (fps). The implementation clamps $S_G$ to a bounded range for numerical stability and falls back to $S_G=1.5$ when geometry inputs are invalid.

where $TOF$ is the time of flight in seconds.

**Sign:** Positive (rightward) for right-hand (positive) twist; negative (leftward) for left-hand (negative) twist.

**MOA:**

$$\text{spin\_drift\_moa} = \frac{\delta_{drift,m}}{R} \times K_{RAD \to MOA}$$

where $K_{RAD \to MOA} \approx 3437.75$ and $R$ is the target range in metres.

> **Sync:** If the $S_G$ model changes (formula inputs, clamps, or fallback behavior), update this section.
> **Implementation:** `solver.cpp` lines tagged `[MATH §12]`

---

## 13. Coriolis & Eötvös Effects

**Source:** [lib/dope/src/solver/solver.cpp](lib/dope/src/solver/solver.cpp), `BallisticSolver::integrate()`.

`[MATH §13]`

The Coriolis effect deflects a moving bullet due to Earth's rotation. The Eötvös effect is the vertical component of the same phenomenon (affecting elevation via apparent gravitational change when firing east/west).

**Assumption:** This is a simplified constant-velocity model valid for short to medium engagement distances. It assumes the bullet travels at a constant effective speed equal to $v_{avg} \approx R / TOF$.

Let:
- $\omega_E = 7.2921 \times 10^{-5}$ rad/s — Earth's rotation rate
- $\lambda$ = shooter's geodetic latitude (rad)
- $\alpha$ = firing azimuth (rad, from north, clockwise)
- $R$ = target range (m)
- $TOF$ = time of flight (s)

**Horizontal (windage) deflection:**

$$\Delta z = \omega_E \cdot R \cdot TOF \cdot \sin\lambda \cdot \sin\alpha$$

Maximum deflection occurs for east/west shots ($\alpha = \pm 90°$); north/south shots ($\alpha = 0, 180°$) have zero horizontal Coriolis.

**Vertical (Eötvös) deflection:**

$$\Delta y = \omega_E \cdot R \cdot TOF \cdot \cos\lambda \cdot \sin\alpha$$

Firing east ($\alpha = 90°$) increases apparent gravity (bullet hits lower); firing west decreases it (bullet hits higher).

**MOA conversions:**

$$\text{coriolis\_wind\_moa} = \frac{\Delta z}{R} \times K_{RAD \to MOA}$$

$$\text{coriolis\_elev\_moa} = \frac{\Delta y}{R} \times K_{RAD \to MOA}$$

Coriolis is disabled (zeroed) when no latitude is provided (diagnostic flag `CORIOLIS_DISABLED`).

> **Sync:** If a more rigorous Coriolis model is implemented (e.g., full vector integration), remove the $R \cdot TOF$ approximation and update §13 with the new derivation.
> **Implementation:** `solver.cpp` lines tagged `[MATH §13]`

---

## 14. MOA Hold Computation

**Source:** [lib/dope/src/engine/dope_engine.cpp](lib/dope/src/engine/dope_engine.cpp), `DOPE_Engine::computeSolution()`.

### 14.1 Sight-Line Geometry

`[MATH §14.1]`

The solver returns the bullet's vertical position $y_{bullet}(R)$ measured from the **bore axis**. To convert to a hold, we need the drop relative to the **sight line**.

The sight line at range $x$ is:

$$y_{LOS}(x) = h_s \cdot \left(1 - \frac{x}{R_0}\right)$$

where $h_s$ is sight height above bore (m) and $R_0$ is the zero range. This is a straight line from the optic ($h_s$ at $x = 0$) to the target ($0$ at $x = R_0$).

The drop relative to the sight line at the target range $R$:

$$\Delta y_{rel} = y_{bullet}(R) - y_{LOS}(R) = y_{bullet}(R) - h_s\left(1 - \frac{R}{R_0}\right)$$

### 14.2 Drop and Windage → MOA

`[MATH §14.2]`

The angular correction required to bring point of aim onto point of impact:

$$\text{drop\_moa} = -\frac{\Delta y_{rel}}{R} \times K_{RAD \to MOA}$$

$$\text{wind\_moa} = -\frac{z_{bullet}(R)}{R} \times K_{RAD \to MOA}$$

The minus sign: a positive $\Delta y_{rel}$ (bullet above sight line) means the point of impact is high, so the hold is negative (aim lower). A positive $z_{bullet}$ (bullet drifted right under wind) means hold left, so negative.

These raw values are then summed with:
- `+` Coriolis elevation (§13)
- `+` Boresight vertical offset (mechanical optic misalignment, MOA)
- `+` Reticle vertical offset (user dial-in offset, MOA)

And for windage:
- `+` Coriolis windage (§13)
- `+` Spin drift (§12)
- `+` Boresight horizontal offset
- `+` Reticle horizontal offset

### 14.3 LRF IIR Filter

`[MATH §14.3]`

Incoming laser rangefinder (LRF) measurements are smoothed with a first-order infinite impulse response (IIR) filter (exponential moving average):

$$R_{filtered,n} = \alpha \cdot R_{new} + (1 - \alpha) \cdot R_{filtered,n-1}$$

where $\alpha = 0.2$ (`DOPE_LRF_FILTER_ALPHA`). This weights the new reading at 20% and retains 80% of the history, providing modest noise rejection while remaining responsive.

> **Sync:** If $\alpha$ changes, update §14.3 and `DOPE_LRF_FILTER_ALPHA` in `dope_engine.cpp`.
> **Implementation:** `dope_engine.cpp` lines tagged `[MATH §14.3]`

### 14.4 FOV from Focal Length

`[MATH §14.4]`

The camera field of view is computed from the encoder-reported focal length $f$ (mm) and the IMX477 sensor half-dimensions:

$$HFOV = 2 \cdot \arctan\!\left(\frac{3.1435}{f}\right) \cdot \frac{180°}{\pi}$$

$$VFOV = 2 \cdot \arctan\!\left(\frac{2.3560}{f}\right) \cdot \frac{180°}{\pi}$$

Constants 3.1435 mm and 2.3560 mm are the half-width and half-height of the IMX477 sensor (reference values; camera pipeline not yet active in engine — encoder → FOV computation reserved for future implementation).

> **Sync:** When the camera pipeline is integrated, update the sensor constants in `dope_engine.cpp` and this section.
> **Implementation:** `dope_engine.cpp` lines tagged `[MATH §14.4]` (currently stubbed — encoder frame is not consumed)

> **Overall sync for §14:** Any change to the sight-line model, MOA conversion constant, or filter $\alpha$ requires updates to the corresponding subsection above.
> **Implementation:** `dope_engine.cpp` lines tagged `[MATH §14.1]`–`[MATH §14.4]`

---

## 15. Cant Correction

**Source:** [lib/dope/src/corrections/cant.cpp](lib/dope/src/corrections/cant.cpp), `CantCorrection::apply()`.

`[MATH §15]`

When the rifle is canted (rolled) by angle $\theta_{cant}$ (= AHRS roll, positive = right wing down), the reticle's vertical axis is no longer aligned with vertical. The elevation hold vector rotates in the sight plane:

$$\text{elev}_{corrected} = \text{elev} \cdot \cos\theta_{cant}$$

$$\text{wind}_{cant} = \text{elev} \cdot \sin\theta_{cant}$$

The cant converts some vertical hold into spurious horizontal hold. At 90° cant, all elevation becomes windage and vice versa.

**Important:** The cant correction is applied to the *total accumulated elevation hold* (after all other corrections), not to the raw drop. The cant-introduced windage is **added** to the existing windage hold, not the other way around.

> **Sync:** If the cant model changes (e.g., cant is applied before the other offsets), update both §15 and the call site order in `dope_engine.cpp` `computeSolution()`.
> **Implementation:** `cant.cpp` lines tagged `[MATH §15]`; call site in `dope_engine.cpp` tagged `[MATH §15]`

---

## 16. Uncertainty Propagation

**Source:** [lib/dope/src/engine/dope_engine.cpp](lib/dope/src/engine/dope_engine.cpp), `DOPE_Engine::computeUncertainty()`.

`[MATH §16]`

### 16.1 Theory — Central Finite Differences

`[MATH §16.1]`

For a scalar output $y$ (e.g., elevation hold in MOA) that depends nonlinearly on an uncertain input $x$ with 1-sigma uncertainty $\sigma_x$, the first-order Gaussian error propagation gives:

$$\sigma_y^2 \approx \left(\frac{\partial y}{\partial x}\right)^2 \sigma_x^2$$

The partial derivative is approximated by the symmetric (central) finite difference:

$$\frac{\partial y}{\partial x} \approx \frac{y(x + \sigma_x) - y(x - \sigma_x)}{2\sigma_x}$$

Substituting:

$$\sigma_y^2 \approx \left(\frac{y(x+\sigma_x) - y(x-\sigma_x)}{2\sigma_x}\right)^2 \sigma_x^2 = \left(\frac{\Delta y}{2}\right)^2$$

where $\Delta y = y(x+\sigma_x) - y(x-\sigma_x)$. The $\sigma_x$ terms cancel — the name of the perturbation does not matter.

For **multiple independent inputs** $x_1, \ldots, x_n$, variances are summed (independence assumption):

$$\text{var}_{e,total} = \sum_{i} \left(\frac{\Delta e_i}{2}\right)^2, \qquad \sigma_{elev} = \sqrt{\text{var}_{e,total}}$$

$$\text{var}_{w,total} = \sum_{i} \left(\frac{\Delta w_i}{2}\right)^2, \qquad \sigma_{wind} = \sqrt{\text{var}_{w,total}}$$

Elevation–windage covariance:

$$\text{cov}_{ew} = \sum_{i} \frac{\Delta e_i}{2} \cdot \frac{\Delta w_i}{2}$$

Each perturbation pair requires **two full solver integrations** (plus cant correction). With 17 inputs, this is 34 solver runs per frame.

> **Sync:** If the number of uncertain inputs changes, update the `kNumUncertaintyInputs` constant in `dope_types.h` and the index table below.
> **Implementation:** `dope_engine.cpp` lines tagged `[MATH §16.1]`

### 16.2 Output

`[MATH §16.2]`

$$\sigma_{elev} = \sqrt{\text{var}_{e,total}}, \qquad \sigma_{wind} = \sqrt{\text{var}_{w,total}}, \qquad \text{cov}_{ew} = \text{cov}_{ew,total}$$

All three are stored in the `FiringSolution`. The per-input variance arrays `uc_var_elev[i]` and `uc_var_wind[i]` allow the UI to show the dominant source of uncertainty.

The engine intentionally outputs uncertainty primitives (sigmas/covariance/per-input variance) and does not output sampled-shot counts, hit-probability percentages, or an engine-level confidence score.

> **Implementation:** `dope_engine.cpp` lines tagged `[MATH §16.2]`

### 16.3 Barrel Stiffness, Accuracy Hierarchy, and Finish MV Sigma

`[MATH §16.3]`

Baseline dispersion is injected before user-provided sigmas:

- **Barrel stiffness floor (radial):** wall thickness at muzzle \(w = (D_{od} - D_{bore})/2\). Map to \(M_{stiff}\): Bull/Heavy if \(w > 0.35\) → 0.20 MOA; Medium if \(0.20 \le w \le 0.35\) → 0.75 MOA; Pencil if \(w < 0.20\) → 1.25 MOA. Add \(M_{stiff}\) via RSS to both elevation and windage variance.
- **Accuracy hierarchy:** pick highest-fidelity source: measured CEP50 (manufacturer test) → manufacturer spec MOA → category defaults (radial/vertical). If CEP50 exists, split 80% radial, 20% vertical. Combine radial with \(M_{stiff}\) via RSS into both axes; combine the vertical slice via RSS into elevation only. Category fallbacks are already axis-specific.
- **Finish MV sigma floor:** muzzle-velocity sigma is floored to the barrel-finish value (Stainless 5 fps, Nitride 8.5 fps default, Chrome 20 fps) before other MV uncertainty; engine converts to m/s internally.

These baselines precede any CEP-table scaling so axis ratios are preserved.

### 16.3 Input Table

| Index | Input $x_i$ | Perturbation $\sigma_{x_i}$ (default) |
|-------|------------|---------------------------------------|
| 0 | Muzzle velocity | 1.5 m/s (~5 fps) |
| 1 | Ballistic coefficient | 2% of BC |
| 2 | Range | 1 m |
| 3 | Wind speed | 1.0 m/s (~2.24 mph) |
| 4 | Wind heading | 2° |
| 5 | Temperature | 1.111 °C (~2 °F) |
| 6 | Pressure | 5 Pa |
| 7 | Humidity | 5% (0.05) |
| 8 | Sight height | 0.075 mm |
| 9 | Cant angle | 1.5° |
| 10 | Latitude | 0° (disabled by default) |
| 11 | Mass | GUI-derived: $\sigma_m = m_{gr} \cdot K_m$ (default $K_m = 0.0012$, Match-Grade) |
| 12 | Length | GUI-derived: $\sigma_L = L_{mm} \cdot K_L$ (default $K_L = 0.005$, Standard), propagated through dynamic $S_G$ into spin drift and drag coupling |
| 13 | Caliber | GUI-derived: $\sigma_c = c_{in} \cdot K_c$ (default $K_c = 0.00045$, Lead Core Match-Grade) |
| 14 | Twist | GUI-derived: $\sigma_t = 0.01 \cdot t_{in/turn}$ (1% of twist rate; e.g. 8 → 0.08, 10 → 0.10) |
| 15 | Zero range | 0.5 m |
| 16 | MV adjustment | 1.0 fps/in |

**Atmospheric inputs (5–7):** Each pair uses a temporary `Atmosphere` copy so the engine state is not mutated during uncertainty evaluation.

**Range input (2):** Both perturbed runs use a different `target_range_m` and a correspondingly different sight-line geometry, so the evaluation is self-consistent.

**Cant (9):** The trajectory integration is *not* re-run; only the cant rotation (§15) is re-applied with a different roll angle.

**GUI material/type presets (for inputs 11–13):**
- Mass ($K_m$): Monolithic Copper `0.0005`, Match-Grade `[DEFAULT]` `0.0012`, Soft Point `0.0025`, Cheap `0.005`
- Length ($K_L$): Polymer Tips `0.002`, Soft Point `0.008`, Standard `[DEFAULT]` `0.005`, Match Sorted `0.001`
- Caliber ($K_c$): Monolithic Copper `0.00015`, Lead Core Match-Grade `[DEFAULT]` `0.00045`, Soft Point `0.00075`, Cheap `0.0015`

> **Sync:** If default sigma values change, update the table above and `getDefaultUncertaintyConfig()` in `dope_engine.cpp`.

### 16.4 Barrel Attachment & Thermal Dispersion

`[MATH §16.4]`

Additional barrel-side dispersion adjustments applied after the §16.3 hierarchy and before CEP scaling. Implementation: [lib/dope/src/engine/dope_engine.cpp](lib/dope/src/engine/dope_engine.cpp).

- **Free-float penalty:** If the barrel is not free-floated, radial dispersion becomes `rss(radial_moa, 0.75)`.
- **Suppressor scaling:** With a suppressor attached, radial dispersion is scaled linearly by muzzle OD: 1.25× at ≤0.55", 1.05× at ≥1.00", interpolated between.
- **Barrel tuner:** If present, radial dispersion is multiplied by 0.85.
- **Material scaling:** Stiffness and thermal growth are scaled by barrel material: CMV (~0.95× stiffness factor, lower CTE), 416 stainless (baseline), carbon-wrapped (~1.30× stiffness factor, low CTE, lower density/higher heat capacity).
- **Thermal growth:** For ΔT = T_barrel − T_ambient (kelvin), apply $$m_{heat} = \min\bigl(1.6,\; \max(1,\; \sqrt{1 + 0.01\,\Delta T})\bigr)$$ to the barrel-side radial term only.
- **Cooling:** Barrel temperature decays toward ambient via $$T_{b,new} = T_a + (T_b - T_a) e^{-\Delta t/\tau}$$ with τ interpolated from 70 s at 0.55" muzzle OD to 120 s at 1.0". Ambient tracks the latest baro temperature when available.
- **Heating per shot:** Each shot deposits $$E_b = \text{clamp}\bigl(0.2 \cdot 0.5 m_b v^2,\; 400,\; 2000\bigr)$$ J into the barrel (m_b bullet mass, v muzzle velocity). The resulting rise $$\Delta T_{shot} = E_b / C_b$$ uses barrel heat capacity $C_b = m_{barrel} \cdot 500$ J/kgK; barrel mass is a clamped cylindrical estimate (defaults: 24" length, 0.7" muzzle OD, limits 0.5–4.0 kg).

---

## Appendix A — Constants & Unit Reference

**Source:** [lib/dope/include/dope/dope_config.h](lib/dope/include/dope/dope_config.h), [lib/dope/include/dope/dope_math_utils.h](lib/dope/include/dope/dope_math_utils.h).

### A.1 Physical Constants

| Constant | Value | Unit | Used in |
|----------|-------|------|---------|
| `DOPE_GRAVITY` | 9.80665 | m/s² | §11.3 |
| `DOPE_OMEGA_EARTH` | 7.2921 × 10⁻⁵ | rad/s | §13 |
| `DOPE_R_DRY_AIR` | 287.05 | J/(kg·K) | §3.3 |
| `DOPE_STD_AIR_DENSITY` | 1.2250 | kg/m³ | §5.2 |
| `DOPE_KELVIN_OFFSET` | 273.15 | K | §3 |
| `dope::math::BALLISTIC_DRAG_CONSTANT` | 900.0 | — | §5.2 |
| `DOPE_EXTERNAL_REFERENCE_DRAG_SCALE` | 0.84 | — | §11.3 |

### A.2 Integration Limits

| Constant | Value | Used in |
|----------|-------|---------|
| `DOPE_MAX_RANGE_M` | 2500 | m | §11 |
| `DOPE_MIN_VELOCITY` | 30.0 | m/s | §11.5 |
| `DOPE_DT_MIN` | 10 µs | §11.4 |
| `DOPE_DT_MAX` | 1 ms | §11.4 |
| `DOPE_MAX_STEP_DISTANCE_M` | 0.25 | m | §11.4 |
| `DOPE_ZERO_TOLERANCE_M` | 0.001 | m | §8 |
| `DOPE_ZERO_MAX_ITERATIONS` | 50 | — | §8 |
| `DOPE_MAX_SOLVER_ITERATIONS` | 500,000 | — | §11.5 |

### A.3 Unit Conversions

All conversions are in `dope_math_utils.h` namespace `dope::math`:

| Conversion | Factor |
|-----------|--------|
| m/s → fps | × 3.280839895 |
| fps → m/s | × 0.3048 |
| grains → kg | × 6.479891 × 10⁻⁵ |
| inches → m | × 0.0254 |
| mm → m | × 0.001 |
| MOA → rad | × 2.90888 × 10⁻⁴ |
| rad → MOA | × 3437.747 |
| deg → rad | × π/180 |
| Pa → inHg | × 2.9530 × 10⁻⁴ |
| J → ft-lbf | × 0.737562 |

### A.4 Internal Units Summary

| Quantity | Internal unit | Notes |
|---------|---------------|-------|
| Angles (all) | radians | Converted to/from degrees at API boundary |
| Output corrections | MOA | Positive = up/right |
| Velocity | m/s | |
| Range, distance | m | |
| Mass | kg | Grains converted on ingest |
| Pressure | Pa | |
| Temperature | °C (stored), K (computed) | |
| Air density | kg/m³ | |
| Magnetometer | µT | |
| Accelerometer | m/s² | |
| Gyroscope | rad/s | |
| Timestamps | µs | |
| Integration time | s | |
| Energy | J | |
| Barrel length | inches | |
| Sight height | mm (API), m (internal) | |
| Focal length | mm | |
| FOV | degrees (output) | |
