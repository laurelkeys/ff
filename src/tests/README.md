- generate `record_log.txt` with `record_viewpoints.py`, directly from AirSim;
- export `outputViewAndPoses.txt` from Meshroom after reconstructing the captured images;
- parse `outputViewAndPoses.txt` into `reconstructed_log.txt` with `evaluator.py`;
- remove the first rows and rename `record_log.txt -> pose_gt.txt` and `reconstructed_log.txt -> pose_est.txt`;
- run `tartanair_evaluator.py`:
```bash
  Scale, 10.235081105553421
  ATE scale: 10.216079572182947
{'ate_score': 0.0021499151379920883, 'rpe_score': (0.15008655267786258, 11.953153520069066), 'kitti_score': (0.684527754775734, 0.7642737812736924)}
```