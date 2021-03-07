from __future__ import annotations

from enum import Enum


class EditMode(Enum):
    SCALING = 0
    ROTATING = 1
    TRANSLATING = 2

    @staticmethod
    def next(
        edit_mode: EditMode,
        skip_scaling: bool = False,
        skip_rotating: bool = False,
        skip_translating: bool = False,
    ) -> EditMode:
        """ Returns the next `EditMode` value, in circular fashion. """
        edit_modes = list(EditMode)

        if skip_scaling:
            edit_modes.remove(EditMode.SCALING)
        if skip_rotating:
            edit_modes.remove(EditMode.ROTATING)
        if skip_translating:
            edit_modes.remove(EditMode.TRANSLATING)

        next_mode = (edit_modes.index(edit_mode) + 1) % len(edit_modes)
        return edit_modes[next_mode]
