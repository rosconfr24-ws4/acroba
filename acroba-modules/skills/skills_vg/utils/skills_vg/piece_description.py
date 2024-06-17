from enum import Enum, auto


class PieceType(Enum):
    BASE = auto()
    SCREEN = auto()
    MECHANISM = auto()
    
    @classmethod
    def get(cls, piece_type): 
        piece_type = piece_type.lower()
        if piece_type == "base": 
            return cls.BASE
        if "mech" in piece_type: 
            return cls.MECHANISM
        if piece_type == "screen": 
            return cls.SCREEN
        return None
    
    def to_code(self):
        if self == PieceType.BASE:
            return "Uhr_Gehause Variant"
        if self == PieceType.MECHANISM:
            return "Ziffernblatt Variant"
        if self == PieceType.SCREEN:
            return "Glas Variant"
        return None
        

class PieceColor(Enum):
    RED = auto()
    ORANGE = auto()
    BLUE = auto()
    
    @classmethod
    def get(cls, color): 
        color = color.lower()
        if color == "red": 
            return cls.RED
        if color == "blue": 
            return cls.BLUE
        if color == "orange": 
            return cls.ORANGE
        return None
    
    def to_code(self):
        if self == PieceColor.RED:
            return "(2)"
        if self == PieceColor.ORANGE:
            return "(1)"
        if self == PieceColor.BLUE:
            return ""
        return None

    @classmethod
    def from_name(cls, name): 
        if "(2)" in name: 
            return cls.RED
        if "(1)" in name: 
            return cls.ORANGE
        return cls.BLUE
    
