# Snapshot v1 (2026-11-23)

State before trying a riskier distance route.

- work/: measured arena, plan (distance and safety both use tracking_margin_m = 0.10),
  runs (check_01, trial_safety_01/02, safety_01, distance_01) and analysis.
- code/: hw4.py, tests, report template, runbook, example arena, ekf_navigator.py, ekf.py, test_ekf.py.

Results: distance route box clearance 11.2 cm, 15.2 s; safety 30.7 cm, 22.2 s; both completed.

Restore: copy code/ files back to docs/hw4/ and robot_control/, copy work/ back to
docs/hw4/work/, then rsync work/plan to rover ~/hw4/plan (or use ~/hw4/plan_v1 there).
