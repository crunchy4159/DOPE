# DOPE Data In / Data Out (ASCII)

Horizontal pipeline: inputs on the left feed numbered stages inside the engine; outputs exit the right wall of the engine from the stage that produces them.

## Data Flow

```text
INPUTS                              +=========================================================+          OUTPUTS
------                              |               DOPE ENGINE  |  BCE_Update()              |          -------
                                    |                                                          |
[Weapon + Zero]  ------------->     |  [1] VALIDATE + NORMALIZE                               |
  zero_range                        |           |                                              |
  sight_height                      |           v                                              |
  muzzle_velocity                   |  [2] ATMO CORRECTION  (SS3, SS4)                        |
                                    |           rho, speed of sound (c), BC_corrected         |
[Atmosphere]  ----------------->    |           |                                              |  --->  [Trajectory Table]
  temp, pressure, humidity, alt     |           v                                              |          per-metre: drop,
                                    |  [3] ZERO-ANGLE SOLVE  (SS8)                            |          windage, speed,
[Projectile + Drag]  ---------->    |           binary search -> bore elevation (theta_0)     |          TOF, energy
  BC, mass, caliber, G1-G8 model    |           |                                              |
                                    |           v                                              |
[Wind]  ------------------------>   |  [4] BUILD SOLVER PARAMS  (SS9, SS10)                   |  --->  [Final Point @ R]
  speed, heading                    |           MV_adj, headwind, crosswind                   |          y (drop)
                                    |           |                                              |          z (windage)
[AHRS / Orientation]  ---------->   |           v                                              |          velocity, TOF, KE
  pitch, roll (cant), yaw           |  [5] RK4 TRAJECTORY INTEGRATE  (SS5, SS11-SS13)        |
                                    |           drag + spin drift + Coriolis                  |  --->  [Firing Solution]
                                    |           adaptive dt until x = R_target                |          elevation_moa
                                    |           |                                              |          windage_moa
                                    |                                                          |  --->  [Realtime Solution]
                                    |                                                          |          elevation_moa
                                    |                                                          |          windage_moa
                                    |                                                          |          uncertainty_radius_moa
                                    |                                                          |          mode/fault/defaults
                                    |           v                                              |
                                    |  [6] HOLD COMPOSITION  (SS12-SS15)                      |  --->  [Correction Terms]
                                    |           drop + windage -> MOA                         |          spin_drift_moa
                                    |           + spin drift + Coriolis + cant                |          coriolis_elev_moa
                                    |           |                                              |          coriolis_wind_moa
[Uncertainty Config]  ---------->   |           v                                              |          cant_moa
  sigma per input (17 params)       |  [7] UNCERTAINTY PROPAGATION  (SS16)                   |
                                    |           central finite-diff x 34 solver runs          |  --->  [Uncertainty]
                                    |           per-input variance breakdown                  |          sigma_elev_moa
                                    +=========================================================+          sigma_wind_moa
                                                                                                         cov_elev_wind
                                                                                                         uc_var[i] x 17
```

## Notes

- All inputs feed into the engine via `BCE_Update()` at the top of each solver frame.
- `BCE_GetRealtimeSolution()` returns a compact hot-path payload for embedded loops; `BCE_GetSolution()` remains the full diagnostic payload.
- The trajectory table records a `TrajectoryPoint` at every integer metre; the final point at `x = R_target` drives the firing solution.
- Uncertainty (stage 7) re-runs stages 4–6 twice per input (34 total runs) and does not mutate engine state.
