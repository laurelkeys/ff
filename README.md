# ff

![](docs/workflow.png)

```bash
.
├── data/                   # Generated data logs (.txt, .ply, .json, ...)
│
├── docs/                   # Project proposal and reports
│
├── misc/                   # General scripts for setup and workflow automation
│
├── src/
│   ├── ds/                 # Collection of classes common to different scripts
│   ├── ff/
│   ├── ie/                 # API wrappers ("id est", in other words)
│   └── scripts/
│       ├── reconstruction/ # Meshroom and Open3D related code
│       └── simulation/     # AirSim related code for drone control
│
└── vendor/
    ├── TanksAndTemples/    # 3D reconstruction evaluation
    └── tartanair_tools/    # SLAM evaluation (ATE + RPE)
```
