PR: Enforce DOPE pure‑math boundary; move sensor processing to DOPE‑ASS

Summary

- Clarified and enforced the DOPE / DOPE-ASS boundary in `DOPE SRS.md` to ensure DOPE remains a pure numeric engine.
- Removed AHRS and raw driver/protocol references from the DOPE engine block in the system architecture diagram.
- Added a `SensorFrame` sanitized input schema to Section 7 of `DOPE SRS.md` describing the exact fields DOPE expects from DOPE-ASS.
- Created `DOPE_ASS_MIGRATION.md` listing items that must be implemented in DOPE-ASS (fusion, driver parsing, calibration flows, LRF protocol handling, GNSS parsing, etc.).
- Created/updated GUI guidance: `tools/native_gui/GUI_BOUNDARY_TODO.md` now references the migration checklist and provides actionable GUI update steps.

Files changed/added

- Updated: `DOPE SRS.md` — clarified segmentation, removed AHRS from DOPE block, added `SensorFrame` schema.
- Added: `DOPE_ASS_MIGRATION.md` — migration checklist for DOPE-ASS.
- Added/Updated: `tools/native_gui/GUI_BOUNDARY_TODO.md` — GUI action items and link to migration checklist.

Rationale

- The project goal is that `lib/dope/` remains optimized and lightweight (pure math). Moving sensor handling and fusion into DOPE-ASS avoids cross-cutting platform/driver code in the engine and preserves determinism and portability.

Recommended next steps

1. Review `DOPE_ASS_MIGRATION.md` and assign owners for each migration task.
2. Update `tools/native_gui` to reflect DOPE-ASS responsibilities (remove driver/protocol controls or mark them DOPE-ASS-only). See `tools/native_gui/GUI_BOUNDARY_TODO.md`.
3. Add integration tests ensuring DOPE receives valid `SensorFrame` shapes via `DOPE_Update`.
4. Optionally open a PR with these edits; suggested commit title below.

Suggested commit message

"docs(srs): enforce DOPE pure-math boundary; add SensorFrame schema and DOPE-ASS migration checklist"

If you want, I can open a PR branch and create the PR text and diff next.
