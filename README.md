# ff

![](docs/workflow.png)

```bash
.
├── docs                    # Project proposal and reports
│
├── misc                    # General scripts for setup and workflow automation
│
├── reconstruction
│   ├── models              # Reconstructed models/scenes
│   │   ├── <model name>
│   │   │   ├── data        # Reconstruction files (.ply, .sfm, .log, etc.)
│   │   │   ├── images      # Used images files (.jpg)
│   │   │   └── Meshroom    # Meshroom intermediate files
│   │   └── ...
│   └── tanksandtemples     # Tanks and Temples benchmark evaluation scripts
│
├── simulation
│   ├── multirotor          # AirSim related code for drone control
│   │   └── ff              # Drone scripts common code wrapper
│   └── viewpoints          # Images captured in the simulator
│
├── src                     # 🚧 WIP revamping of the whole code
│
└── visualization           # Open3D related code
```

## 🚧 WIP
Currently, everything is being rewritten in `src/`, so there's no work being done on other folders as they will either be copied and updated inside of `src/`, or deleted once it's done and the project is reestructured.
