# ff

![](docs/workflow.png)

```bash
.
├── docs/                   # Project proposal and reports
│
├── misc/                   # General scripts for setup and workflow automation
│
├── scripts/
│   ├── data/               # Generated data logs (.txt, .ply, .json, ...)
│   ├── reconstruction/     # Meshroom and Open3D related code
│   └── simulation/         # AirSim related code for drone control
│
├── src/
│   ├── ds/                 # Collection of classes common to different scripts
│   ├── ff/
│   └── ie/                 # API wrappers ("id est", in other words)
│
└── vendor/
    ├── TanksAndTemples/    # 3D reconstruction evaluation
    └── tartanair_tools/    # SLAM evaluation (ATE + RPE)
```
