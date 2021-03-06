<!--  -->
# Useful direct links
## AirSim
- [`faq.md`](https://github.com/microsoft/AirSim/blob/master/docs/faq.md)
- [`settings.md`](https://github.com/microsoft/AirSim/blob/master/docs/settings.md)
- [`hard_drive.md`](https://github.com/microsoft/AirSim/blob/master/docs/hard_drive.md)
- [`dev_workflow.md`](https://github.com/microsoft/AirSim/blob/master/docs/dev_workflow.md)
- [`build_windows.md`](https://github.com/microsoft/AirSim/blob/master/docs/build_windows.md)
- [`unreal_blocks.md`](https://github.com/microsoft/AirSim/blob/master/docs/unreal_blocks.md)
- [`unreal_custenv.md`](https://github.com/microsoft/AirSim/blob/master/docs/unreal_custenv.md)
- [`unreal_upgrade.md`](https://github.com/microsoft/AirSim/blob/master/docs/unreal_upgrade.md)
## Unreal Engine
- Unreal Engine [command-line arguments](https://docs.unrealengine.com/en-US/Programming/Basics/CommandLineArguments/index.html)

<!--  -->
# To-do list
- [ ] Create a `scripts/common/` folder and (at least to begin with) symlink it to `reconstruction/` and `simulation`
- [ ] Use longer (more descriptive) names for the files under `scripts/`
- [ ] Use `settings.py` as an alternative for command line arguments
- [ ] Automatically change `SimMode` for `cv_*.py` files (and also restore it)
- [ ] Only choose the first environment (when there's more than one possibility) if a flag `--choose` is passed, it might be better for this to be the default behavior instead
  - [ ] Add an argument to it, so that calling `$ ... --launch --choose 2` starts the second environment from the list of possibilities
- [ ] Pass an option to add `-log` in `_build_run_cmds` for `.uproject` files
  - [ ] See [UE4's Stat Commands](https://docs.unrealengine.com/en-US/Engine/Performance/StatCommands/index.html) for monitoring performance, e.g. `Stat FPS`, `Stat UnitGraph`, `r.VSync`, `t.maxFPS`
- [ ] See https://github.com/harvard-edge/airlearning-rl/blob/master/game_handling/game_handler_class.py#L30
- [ ] See https://github.com/harvard-edge/airlearning-rl/blob/master/misc/move_to_airsim/move.py
- [ ] See https://github.com/microsoft/AirSim/issues/1677#issuecomment-469999696
- [ ] See https://github.com/microsoft/AirSim/issues/1677#issuecomment-605440212
- [ ] See https://github.com/microsoft/AirSim/pull/2324
- [ ] See https://github.com/microsoft/AirSim/issues/543
- [ ] See https://github.com/microsoft/AirSim/blob/master/PythonClient/computer_vision/capture_ir_segmentation.py#L10
- [ ] See https://github.com/microsoft/AirSim/blob/master/ros/src/airsim_tutorial_pkgs/scripts/multi_drone_json_creator.py
- [ ] See https://meshroom-manual.readthedocs.io/en/latest/node-reference/nodes/SfMTransform.html
- [ ] See https://github.com/microsoft/AirSim-Drone-Racing-Lab/blob/master/baselines/baseline_racer.py#L36
- [ ] See https://github.com/microsoft/AirSim/issues/3251
- [ ] See https://github.com/microsoft/AirSim/pull/3239/files

<!--  -->
# Basic `simulation` script template
```python
import argparse

import ff
import airsim

from ie.airsimy import connect

## preflight (called before connecting) #######################################
def preflight(args: argparse.Namespace) -> None:
    # ...
    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(*ff.LaunchEnvArgs(args))
        ff.input_or_exit("\nPress [enter] to connect to AirSim ")

## fly (called after connecting) ##############################################
def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    initial_pose = client.simGetVehiclePose()
    if args.verbose:
        ff.print_pose(initial_pose, airsim.to_eularian_angles)
    # ...

## main #######################################################################
def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)
    preflight(args)  # setup
    client = connect(ff.SimMode.ComputerVision)
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C
    finally:
        ff.log("Done")

## argument parsing ###########################################################
def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")
    # ...
    ff.add_arguments_to(parser)
    return parser

###############################################################################
if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)
```
