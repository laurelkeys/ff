from __future__ import annotations

from enum import Enum


class EditMode(Enum):
    SCALING = 0
    TRANSLATING = 1

    @staticmethod
    def next(edit_mode: EditMode) -> EditMode:
        """ Returns the next `EditMode` value, in circular fashion. """
        edit_modes = list(EditMode)
        return edit_modes[(edit_mode.value + 1) % len(edit_modes)]
