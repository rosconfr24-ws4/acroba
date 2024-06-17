from enum import Enum, auto


class PieceType(Enum):
    BASE = auto()
    SCREEN = auto()
    MECHANISM = auto()

class PieceColor(Enum):
    RED = auto()
    ORANGE = auto()
    BLUE = auto()