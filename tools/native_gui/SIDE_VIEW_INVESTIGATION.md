**Summary**
- **Symptom:** Side View Arc shows the visual `Target` marker elevated when `Elevation Delta` is 0. When `Elevation Delta` is ±3 (ft), the aim point sometimes crosses the trajectory arc (not tangent) and the grey bore→target connector no longer lines up.
- **Impact:** Visualization no longer reflects solver geometry reliably; user confusion and potential misinterpretation of aim/impact.

**Reproduction Steps**
- Open the native GUI (tools/native_gui/gui_main.cpp harness).
- Set `Elevation Delta` to 0 → observe red `Target` marker elevated above expected bore-line.
- Set `Elevation Delta` to -3 or 3 → observe aim marker tangent behavior is broken and grey connector misaligned.

**Key Code Locations**
- Side-view rendering and math: [tools/native_gui/gui_main.cpp](tools/native_gui/gui_main.cpp#L4238-L4585)
  - reads `g_state.target_elevation_m` and computes `target_dist_horiz_m`, calls `DOPE_GetTrajectoryPoint()` and computes `drop_at_target_interp` (solver drop used in final placement).
  - arc sampling and aim-point root-finding live here (lines show arc construction, sightline intersection, and target drawing).
- Trajectory point data (engine contract): [lib/dope/include/dope/dope_types.h](lib/dope/include/dope/dope_types.h#L388-L396)
  - `TrajectoryPoint.drop_m` is defined as vertical displacement from the bore line (m, negative = below bore).
- Where GUI frames are built and `SensorFrame.target_elevation_m` is sent to engine: [tools/native_gui/gui_main.cpp](tools/native_gui/gui_main.cpp#L1818-L1826)

**What I tried (change log)**
- Tried making visual `Target` marker reflect the user-entered muzzle→target delta directly by solving for a local y such that the rotated world Y == `g_state.target_elevation_m`. This made the red marker exactly match the UI input but broke the visual relationship with the trajectory arc and aim tangent for nonzero deltas (overshoot/steep arcs and connector mismatch).
- Reverted that change and restored the original logic using the solver-interpolated `drop_at_target_interp` for target placement. Current repo state uses the solver's trajectory to place the red target marker (restored behavior).

**Diagnosis / Root Causes**
- Two coordinate frames are involved:
  - Solver `TrajectoryPoint.drop_m` is expressed relative to the **bore-axis** frame (drop_m ≤ 0 below bore line).
  - The UI `target_elevation_m` is a **world-space muzzle→target** vertical delta (user input).
- The side-view combines solver data, look-angle (incline), and sight-height offsets; mixing user-entered world-space Y with solver local drop without consistent conversion leads to visual inconsistencies, especially when the solver arc and the user delta disagree.
- The aim-point root-finding expects the trajectory arc (solver) geometry; forcing the target marker to an arbitrary world Y can make the sightline intersection fallback code produce non-tangent results or fallbacks that place aim incorrectly.

**Design Decision Options**
1. Keep the visual `Target` marker as the solver-predicted impact (current):
   - Pros: Always aligns with the arc and aim intersection math; shows what the engine predicts.
   - Cons: May not visually reflect the user-entered muzzle→target delta (source of confusion).
2. Make the visual `Target` marker exactly match the user-entered muzzle→target delta:
   - Pros: Visual matches user mental model (muzzle→target delta). 
   - Cons: Requires drawing a second solver-impact marker to avoid losing solver context; otherwise aim/arc tangent relationships will appear broken.
3. Show both: add a separate `User Target` marker and keep the `Solver Impact` marker (different color/labels). Also add a short UI blurb explaining the difference.

**Recommended Next Steps (short-term)**
- Implement option (3): draw both markers.
  - Keep the solver-impact marker (current red or change to different color) placed using `drop_at_target_interp` (unchanged).
  - Add a second marker labeled `User Target` placed using the user delta (convert carefully between world/bore coordinates when necessary; simpler: rotate user world Y into the same render space using the same transform currently used for solver points).
  - Add a small legend text in the `Side View Arc` explaining which marker is which.
- Add automated visual tests / manual checklist (see below) to verify tangent/connector alignment across ±5 ft deltas.

**Recommended Next Steps (agents & automation)**
- Create an `Explore` agent task that:
  - Greps for all uses of `target_elevation_m`, `drop_at_target_interp`, `DOPE_GetTrajectoryPoint` across `tools/native_gui`.
  - Prepares a paired diff that adds the second `User Target` marker with a distinct color and label.
- Create a `RenderTest` agent to run the GUI harness headlessly (if possible) or snapshot renders for a small set of delta values: {-3, 0, 3} ft, and compare pixel positions of muzzle, solver-impact, and user-target.

**Quick Manual Verification Checklist**
- [ ] Start GUI harness.
- [ ] Set `Elevation Delta` = 0 → ensure solver-impact marker lies on arc and `Aim pt` is tangent. User-target (if added) should coincide if zero.
- [ ] Set `Elevation Delta` = -3 / +3 → ensure `Aim pt` remains tangent to the trajectory arc; solver-impact and user-target markers should be separately visible and connector should run horizontally at bore Y to solver-impact X.

**Implementation notes & snippets**
- To draw an extra `User Target` marker, compute the same rotated world coords used when rendering the arc but use `user_target_elev_m` instead of `drop_at_target_interp` when composing the rotated coordinates so the marker visually represents the entered delta.
- Keep the aim-point root-finding unchanged (it must operate on the solver arc). If needed, draw the `User Target` marker purely as an annotation, not as input to the aim calculation.

**Files to review / edit**
- [tools/native_gui/gui_main.cpp](tools/native_gui/gui_main.cpp#L4238-L4585) — side view rendering.
- [lib/dope/include/dope/dope_types.h](lib/dope/include/dope/dope_types.h#L388-L396) — TrajectoryPoint contract.

**Suggested agent tasks (for your new agents)**
- Agent `side-view-explore` (quick): gather all side-view rendering code paths and produce a patch that adds a second marker and legend.
- Agent `side-view-test` (medium): run the GUI with scripted inputs and capture screenshots; compare marker alignment and produce a small report.

**Author / History**
- Investigation started by pair session with assistant. Attempted a direct visual-target substitution and reverted due to breakage. Current repo state restored to solver-based visual placement.

---
If you want, I can now:
- create the `side-view-explore` agent task and produce the patch adding a `User Target` marker + legend, or
- implement the patch directly and run the quick verification steps for you to test locally.

Tell me which of the two you'd like me to do next.

---

Development log — changes applied
- **Fixed X-axis mapping**: arc sampling now maps X using horizontal distance (grid units) so arc points align with X-axis ticks and the solver table. See [tools/native_gui/gui_main.cpp](tools/native_gui/gui_main.cpp#L4238-L4680).
- **Removed unused variables / cleaned up root-find**: eliminated spurious locals and tightened the root-finding logic to avoid compiler warnings.
- **Added debug overlay**: `Show Side View Debug` checkbox prints `target_dist_horiz_m`, `traj_drop_at_target_m`, `drop_at_target_m`, `sight_height_m`, `target_y_world_rot`, `aim_dist_horiz`, and `aim_y_world` to help verify mapping at runtime. See the Side View window in the GUI.
- **Aim-point fallback logic**: initially tried residual-minimizing search, then several heuristics, and finally implemented a secant-based solver starting near the target; if secant fails the aim now falls back to the target horizontal distance to ensure the aim marker reaches the target X (see [tools/native_gui/gui_main.cpp](tools/native_gui/gui_main.cpp#L4350-L4680)).

How to verify locally
- Build & run the native GUI (Windows):

```powershell
.
# from workspace root
.
```

- In the GUI open **Side View Arc** and enable **Show Side View Debug** (checkbox lower-left).
- Note the printed DBG values (three lines). Copy/paste them here or attach a screenshot. Key values:
  - `target_dist_horiz_m` — horizontal distance to the target (m)
  - `traj_drop_at_target_m` / `drop_at_target_m` — solver drop used for placement (m)
  - `aim_dist_horiz` / `aim_y_world` — where the aim marker is plotted

Next recommended steps
- If the visuals now match expectations, I'll remove the debug overlay and add a small legend explaining `Solver Impact` vs `User Target` (option 3 from the doc).
- If the aim still looks wrong, paste the DBG output and a screenshot and I'll iterate (I can add a distinct `User Target` marker as an annotation to clarify the difference).

Documentation updated in: [tools/native_gui/SIDE_VIEW_INVESTIGATION.md](tools/native_gui/SIDE_VIEW_INVESTIGATION.md)

**Handoff (for next developer)**

- **Status:** Investigation complete; code adjusted to use horizontal X mapping and several aim-finding fallbacks were implemented. Visual mismatch remains for the user's default presets: the `Aim pt` plotted substantially closer than the `Target` (example DBG: `target_dist_horiz_m` ≈ 457.2 m, `aim_dist_horiz` ≈ 146.25 m).
- **What I could not do:** I could not run a build-and-verify cycle from my environment due to a missing local Python executable used by the project's PlatformIO task. The user ran the native GUI locally and supplied the debug screenshot and values.
- **Repro steps (developer):**
  - Build & run the native GUI on Windows using the project's PlatformIO/native build or your preferred toolchain.
  - Open the Side View Arc window and enable `Show Side View Debug`.
  - Test `Elevation Delta` values: -3, 0, +3 ft and capture the printed DBG values and screenshots.
- **Immediate suggested work items (highest priority):**
  1. Add a distinct `User Target` marker + legend (annotation only) so solver-impact vs user-entered target are visible simultaneously. Keep aim computation driven by the solver arc.
  2. Iterate on the aim-finding heuristic: prefer intersections nearer the target X (already attempted) or otherwise ensure the fallback behavior is explicitly labeled in the UI (e.g., "Aim placed on sightline at target X").
  3. Add a small visual test harness or scripted screenshots for delta values {-3, 0, 3} to validate connector/tangent alignment across presets.
- **Files of interest:**
  - Side view rendering & aim logic: [tools/native_gui/gui_main.cpp](tools/native_gui/gui_main.cpp#L4238-L4680)
  - Investigation doc: [tools/native_gui/SIDE_VIEW_INVESTIGATION.md](tools/native_gui/SIDE_VIEW_INVESTIGATION.md)
  - Trajectory types/contract: [lib/dope/include/dope/dope_types.h](lib/dope/include/dope/dope_types.h#L388-L396)
- **Notes for incoming dev:**
  - Keep the solver `drop_at_target_interp` as the source of truth for impact placement if the goal is to show engine predictions.
  - If you show both markers, clearly label which marker is "Solver Impact" and which is "User Target"; annotate whether the aim calculation uses solver data or user-entered delta.
  - The debug overlay prints the key diagnostics used during investigation: `target_dist_horiz_m`, `traj_drop_at_target_m`, `drop_at_target_m`, `sight_height_m`, `target_y_world_rot`, `aim_dist_horiz`, `aim_y_world`.

If you'd like, I can also open a PR with the `User Target` marker added and a small legend, or prepare a short script to capture side-view screenshots for the three delta values.


--------


## Plan: Side View Aim/Target Diagnosis

TL;DR - What, why, and how
- What: Investigate why the GUI `Aim pt` is plotted much closer to the shooter than the `Target` X, and why the visual `Target` is slightly too high.
- Why: The GUI mixes differently-sampled solver values and has a multi-stage root-finding/fallback that can select a near-muzzle intersection; also small interpolation/discretization and sight-height/unit inconsistencies produce a persistent Y bias.
- How: Verify with the debug overlay, unify interpolation for target placement and tangent geometry, tighten root-finding heuristics (or make fallback explicit), and add an annotation for the user-entered target.

**Steps**
1. Discovery & verification (manual):
   - Reproduce the symptom with the GUI harness and enable `Show Side View Debug`.
   - Collect DBG values shown: `target_dist_horiz_m`, `traj_drop_at_target_m`, `drop_at_target_m`, `sight_height_m`, `target_y_world_rot`, `aim_dist_horiz`, `aim_y_world` for deltas {-3, 0, +3} ft and at least one preset.
   - Save screenshots and DBG lines.
2. Code inspection & small patch (non-invasive):
   - Replace any integer-sampled `traj_drop_at_target_m` computation with the same interpolated `drop_at_target_interp` used for final target placement so both the geometry/tangent math and drawn target use identical data. (*depends on step 1*)
3. Confirm conventions & units:
   - Assert `TrajectoryPoint.drop_m` sign and units match the GUI assumptions (meters, negative = below bore).
   - Confirm `zero.sight_height_mm` -> `sight_height_m` conversion is done consistently everywhere.
4. Root-finding robustness improvements:
   - Prefer bracketing/intervals whose midpoints are nearest the target X.
   - When no bracket exists, prefer the sampled-best-x only if it is >= 0.9 * target_dist_horiz_m (tune threshold), otherwise place aim on sightline at target_X and mark that as an explicit fallback.
   - Add logging to record which branch (bisection/bracket/sample/fallback) was used for each frame.
5. UX clarity: draw both `Solver Impact` and `User Target` markers with distinct colors and a short legend explaining which marker is used by aim calculations.
6. Verification & tests:
   - Manual visual test for deltas {-3, 0, +3} ft across a few cartridges/presets.
   - Snapshot tests: capture side-view screenshots programmatically for those deltas and assert pixel X positions of muzzle, solver-impact, and aim match within tolerance.

**Relevant files**
- tools/native_gui/gui_main.cpp — side-view rendering, arc sampling, root-finding, aim draw. Key areas: around L4205-L4760 (arc sampling and aim math) and the target/state definitions at ~L360-L380.
- tools/native_gui/SIDE_VIEW_INVESTIGATION.md — investigation notes and recommended short-term changes.
- lib/dope/include/dope/dope_types.h — `TrajectoryPoint.drop_m` contract and target fields.
- lib/dope/src/engine/dope_engine.cpp — how engine computes `drop_at_target_m` and uses sight-height when computing holds.
- lib/dope/src/dope_api.cpp — `DOPE_GetTrajectoryPoint(...)` wrapper usage.

**Verification**
1. Run native GUI and enable `Show Side View Debug`.
2. For delta in {-3, 0, +3} ft and one or two presets, capture DBG and screenshot. Expected:
   - `aim_dist_horiz` ≈ `target_dist_horiz_m` (within small tolerance, e.g., 1-2% or 5-10 m for long ranges).
   - `target_y_world_rot` matches arc Y at that X (visual target sits on the sampled/interpolated arc).
   - No aim fallback branch selected except when unavoidable; when fallback occurs it is logged and UI indicates fallback.
3. Add automated snapshot comparisons for regression prevention.

**Decisions / assumptions**
- `TrajectoryPoint.drop_m` is the authoritative solver output and must remain the source-of-truth for impact placement in visualizations (engine predictions > user guess). The `User Target` annotation is allowed but must not change aim math.
- Fix prioritization: first make data-paths consistent (interpolation), then improve root-finding heuristics, then add UX clarity.

**Further Considerations**
1. If interpolation fix alone does not fully resolve near-aim selection, the root cause may be a sign convention mismatch in `drop_m` or a rotation formula bug — add unit/sign assertions and small unit tests for arc->world transform.
2. Consider exposing which aim-branch was used in the UI debug overlay to speed future triage.
3. Option: run a short script to automatically open the GUI, set deltas, and capture screenshots (platform-dependent); if CI-friendly headless runs are possible, add snapshot tests.

Saved for handoff: this plan is persisted to /memories/session/plan.md for follow-up edits and execution.

---

## Follow-up Update: Flat Arc Regression (March 2026)

**Symptom observed**
- Side View showed a visibly flat arc while `hold_elevation_moa` was non-zero.
- Debug example from user run:
  - `target_x_m=457.200`
  - `hold_moa=11.924`
  - `launch_angle_rad=0.000000`
  - `impact_y_at_target_m=-1.738`

**Diagnosis**
- Rendering path used `solution.launch_angle_rad` directly for arc/bore slope.
- In this runtime path, `launch_angle_rad` can be effectively zero while the trajectory table
  still indicates non-flat drop behavior, causing the arc to render too flat.

**Fix applied**
- In Side View rendering, use:
  - `launch_tan = tan(solution.launch_angle_rad)`
  - if `abs(launch_tan) < 1e-5`, fallback to target-anchored `geom_tan_theta` for render slope
  - otherwise use `launch_tan`
- Use this `render_bore_tan` for:
  - arc base equation `y_raw = x * render_bore_tan + drop_interp(x)`
  - bore-direction reference line
- Keep endpoint anchor correction and target/POA semantics unchanged.

**Debug additions**
- `DBG: render_bore_tan=<value> (fallback|launch)` added to side-view debug overlay.

**Expected behavior after fix**
- Arc no longer appears artificially flat when hold is significant.
- Bore line and arc slope remain visually consistent with the selected target geometry.

## Follow-up Update: Over-steep Arc vs Aim (March 2026, second pass)

**Symptom observed**
- After the flat-arc fallback patch, arc could appear steeper than the aim line near muzzle.
- This was due to mixed rendering models (direct `launch_angle_rad` fallback plus endpoint anchoring).

**Reference-based correction**
- Re-applied the older proven side-view shaping approach (used in historical GUI implementation):
  - `hold_m = hold_moa * range`
  - `bore_tan = (target_y + hold_m) / range`
  - `q_a = bore_tan - table_slope_at_0`
  - `q_b = (target_y - (traj_y_target + q_a*range)) / range^2`
  - arc render: `y(x) = x*q_a + x^2*q_b + drop_interp(x)`
- Bore direction line is rendered from `bore_tan`.
- Single `Target`, `Sight`, `Muzzle`, and `Aim pt` semantics remain unchanged.

**Debug overlay changes**
- Removed temporary `render_bore_tan` fallback debug.
- Added: `bore_tan`, `q_a`, `q_b` to inspect arc-shaping behavior.

## Follow-up Update: Top Metric Alignment (March 2026)

**Symptom observed**
- Side View top text "Estimated drop at target" did not match engine output panel
  (`Offset: Elev ...`), causing confusion.

**Fix applied**
- Top metric now uses the same hold-distance basis as engine output:
  - `abs(hold_elevation_moa) * (range * tan(1 MOA))`
- Label updated to `Elevation hold at target` to reflect what is shown.

## Follow-up Update: Bore vs Aim Separation (March 2026)

**Symptom observed**
- Bore line and aim line could appear nearly identical at long range, making sight-height effects
  visually unclear.

**Fix applied**
- Bore reference line now explicitly accounts for sight-height + zero-range convergence:
  - `sightline_tan = (poa_y - sight_height) / range`
  - `bore_visual_tan = sightline_tan + sight_height / zero_range`
- This keeps:
  - Aim line anchored from `Sight` to `Aim pt`
  - Bore line anchored from `Muzzle` with zero-aware separation.

**Debug additions**
- Added: `sightline_tan` and `bore_visual_tan` to side-view debug output.
