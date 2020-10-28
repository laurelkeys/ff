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
## Misc
- Unreal Engine [command-line arguments](https://docs.unrealengine.com/en-US/Programming/Basics/CommandLineArguments/index.html)

# To-do list
- [ ] Use `settings.py` as an alternative for command line arguments
- [ ] Automatically change `SimMode` for `cv_*.py` files (and also restore it)
- [ ] Only choose the first environment (when there's more than one possibility) if a flag `--choose` is passed, it might be better for this to be the default behavior instead
  - [ ] Add an argument to it, so that calling `$ ... --launch --choose 2` starts the second environment from the list of possibilities
- [ ] Make sure every script uses `ff.add_airsim_to_path` when importing AirSim, and that `airsim`-specific imports are done in a `finally` block
- [ ] Pass an option to add `-log` in `_build_run_cmds` for `.uproject` files
  - [ ] See [UE4's Stat Commands](https://docs.unrealengine.com/en-US/Engine/Performance/StatCommands/index.html) for monitoring performance, e.g. `Stat FPS`, `Stat UnitGraph`, `r.VSync`, `t.maxFPS`
- [ ] Move `src/*.py` files to `src/scripts/`
- [ ] See https://github.com/harvard-edge/airlearning-rl/blob/master/game_handling/game_handler_class.py#L30
- [ ] See https://github.com/harvard-edge/airlearning-rl/blob/master/misc/move_to_airsim/move.py
- [ ] See https://github.com/microsoft/AirSim/issues/1677#issuecomment-469999696
- [ ] See https://github.com/microsoft/AirSim/issues/1677#issuecomment-605440212
- [ ] See https://github.com/microsoft/AirSim/pull/2324
- [ ] See https://github.com/microsoft/AirSim/issues/543
- [ ] See https://github.com/microsoft/AirSim/blob/master/PythonClient/computer_vision/capture_ir_segmentation.py#L10
- [ ] See https://github.com/microsoft/AirSim/blob/master/ros/src/airsim_tutorial_pkgs/scripts/multi_drone_json_creator.py
- [ ] Prefix all classes from `airsimy` with `AirSim`, like in `meshroomy`?