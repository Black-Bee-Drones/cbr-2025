class CenterWithArena:
    def __init__(self):
        pass

    @staticmethod
    def calc_arena_position(positions):
        arena_positions = []
        for position in positions:
            x_y = {}
            x_y["x"] = -position["x"] - 2.2
            x_y["y"] = position["y"] + 3.25
            arena_positions.append(x_y)

        return arena_positions
         




