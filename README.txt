ASR700 PROJECT - REPRODUCIBLE RETEST PACKAGE

Evidence status
- Actual Webots measurements are in results/webots and *_webots*.csv.
- Offline synthetic experiments are in results/tables/metrics.json and results/figures.
- Offline accuracy, coverage proxies and PID curves are NOT live Webots outcomes.
- The PDF discloses remaining B2/B3 evaluation gaps. Do not claim every rubric item is fully demonstrated.

Tested computer
Windows 11 25H2, Webots R2025a; Intel i5-11320H, RTX 3050 Laptop GPU.
Python 3.14.0; numpy 2.5.4, scipy 1.18.1, matplotlib 3.11.2,
OpenCV 5.0.0, sklearn 1.9.1, joblib 1.6.0.
The report was built with the separate bundled Python/reportlab/pypdf runtime.

Run from this project folder
1. Install Python dependencies: python -m pip install -r requirements.txt
2. python verify_setup.py
3. python scripts/run_all.py
   Runs all offline reference experiments and rewrites metrics, plots and navigation_mlp.joblib.
   Expected duration on the tested machine: approximately 10 seconds, load dependent.
4. The live controller currently loads data/navigation_mlp_local.joblib, rebuilt with sklearn 1.9.1.
   If using another sklearn version, run the offline script and copy its navigation_mlp.joblib
   to navigation_mlp_local.joblib before Webots. Do not load an incompatible pickle silently.
5. In each controllers/*/runtime.ini set COMMAND to your Python executable.
   The tested configuration is C:/Python314/python.exe. Controller files run inside Webots,
   not directly as ordinary Python scripts, because they require Webots' controller module.
6. Open worlds/component_A_fixed.wbt in Webots. Run Fast (Ctrl+3).
   Allow 180.064 simulation seconds; observer pauses automatically.
7. Open worlds/component_B_fixed.wbt. Run to 120.064 simulation seconds.
   This is the original 1 m radio experiment. Archive its B logs/images before the next test.
8. python analyze_webots.py regenerates summaries and the measured trajectory figure.
9. Open worlds/component_B_connected_test.wbt for the separate 4 m radio diagnostic.
   It overwrites the current B CSVs/images. Keep results/webots/original_range and connected
   separate. Do not combine one run's screenshot with another run's measurements.
10. python build_report.py rebuilds the 9-page report from saved summaries.
    Windows Times New Roman font files are used. Requires the archived connected summary.

Restore worlds
python create_worlds.py regenerates repaired worlds at original starting positions with parquet.
It rewrites world files, not controllers or result logs. Do this before opening a fresh test.
Do not save a simulated final robot pose over the initial world. If you do, regenerate it.
The visual parquet planks do not have collision geometry; the single flat physical floor remains.

Evidence layout
results/webots/A_start.png, A_end.png: actual Webots viewport exports.
A_camera_*.png, A_lidar_*.csv, A_plans.json, A_map_logodds.csv: real controller exports.
A_physics.csv, B_physics.csv: independent supervisor, every 0.032 s physics step.
Contact diagnostic counts points above z=0.012 m; it excludes floor contact.
A_webots_log.csv: pose, active target, commands, detections and replans.
B_webots_epuck*.csv: pose, estimate, packets, action, model status and station distances.
measured_results.json: live summary, distinct from offline metrics.json.
original_range/: preserved 1 m test; connected/: preserved 4 m diagnostic.

Methods and limitations
A live PID=(5,.05,.22); offline illustrative step PID=(10,.6,2).
A uses ground-truth GPS/IMU pose and observed LiDAR occupancy, not SLAM.
B plans on known geometry. Offline PSO stations are projected to safe free grid cells.
B uses a neural avoidance action, hard proximity override and route-based motion.
Radio consensus can fail in disconnected 1 m topology; 4 m is a control experiment.
No live distributed PSO, matched centralised/distributed Webots trials, failure-injection suite,
or model training on real recorded Webots data has been completed. One scene is not robustness proof.
Earlier failed trials are retained in the workspace evidence folder, outside the final clean package.

Submission
Read SUBMISSION_CHECKLIST.md. Complete and personally sign the supplied cover/declaration.
Review all work, explain it yourself and acknowledge assistance per institutional rules.
The brief's cover requests PDF while detailed deliverables request a ZIP: confirm the upload field.
The archive contains the report and runnable source; no Moodle submission has been made.
