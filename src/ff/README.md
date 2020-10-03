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
- [ ] See https://github.com/harvard-edge/airlearning-rl/blob/master/game_handling/game_handler_class.py#L30
- [ ] See https://github.com/harvard-edge/airlearning-rl/blob/master/misc/move_to_airsim/move.py
- [ ] See https://github.com/microsoft/AirSim/issues/1677#issuecomment-469999696
- [ ] See https://github.com/microsoft/AirSim/issues/1677#issuecomment-605440212
- [ ] See https://github.com/microsoft/AirSim/pull/2324
- [ ] Automatically change `SimMode` for `cv_*.py` files (and also restore it)
- [ ] Only choose the first environment (when there's more than one possibility) if a flag `--choose` is passed, it might be better for this to be the default behavior instead
  - [ ] Add an argument to it, so that calling `$ ... --launch --choose 2` starts the second environment from the list of possibilities
- [ ] See https://blog.miguelgrinberg.com/post/sync-vs-async-python-what-is-the-difference
- [ ] Make sure every script uses `ff.add_airsim_to_path` when importing AirSim, and that `airsim`-specific imports are done in a `finally` block (e.g. `from airsim.types import Vector3r`)
- [ ] Check `args.verbose` and use something like `ff.log_debug` where appropriate
- [ ] Run `black` on all scripts, remove unused imports with `Pylance`, and add missing docstrings
- [ ] Check out [Unreal Engine](https://docs.unrealengine.com/en-US/Programming/Basics/CommandLineArguments/index.html)'s commands: `LOG`, `VADEBUG`, `VERBOSE`
- [ ] Add a method to `airsimy.AirSimRotation` to convert from/to `airsim.Quaternionr`
- [ ] Pass an option to add `-log` in `_build_run_cmds` for `.uproject` files
- [ ] See https://github.com/microsoft/AirSim/issues/543