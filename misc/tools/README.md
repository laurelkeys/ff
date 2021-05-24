- `scripts/simulation/`
  - `cv_plot_point_cloud.py` uses [io_ply.py](io_ply.py)

- `scripts/reconstruction/`
  - `evaluate_reconstruction.py` uses [tanksandtemples_evaluator.py](tanksandtemples_evaluator.py) (and so do all `eval_*.py`)
  - `uavmvs_generate_trajectory.py` uses [uavmvs_make_traj.py](uavmvs_make_traj.py)
  - `uavmvs_evaluate_trajectory.py`, `uavmvs_visualize_trajectory.py`, and `uavmvs_trace_trajectory.py` use [uavmvs_parse_traj.py](uavmvs_parse_traj.py)
