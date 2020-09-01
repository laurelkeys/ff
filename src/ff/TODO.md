# To-Dos
- [ ] Use `settings.py` as an alternative for command line arguments
- [ ] See https://github.com/harvard-edge/airlearning-rl/blob/master/game_handling/game_handler_class.py#L30
- [ ] See https://github.com/harvard-edge/airlearning-rl/blob/master/misc/move_to_airsim/move.py
- [ ] See https://github.com/microsoft/AirSim/issues/1677#issuecomment-469999696
- [ ] See https://github.com/microsoft/AirSim/issues/1677#issuecomment-605440212
- [ ] Automatically change `SimMode` for `cv_*.py` files (and also restore it)
- [ ] Only choose the first environment (when there's more than one possibility) if a flag `--choose` is passed, it's better for this to be the default behavior instead
  - [ ] Add an argument to it, so that calling `$ ... --launch --choose 2` starts the second environment from the list of possibilities
