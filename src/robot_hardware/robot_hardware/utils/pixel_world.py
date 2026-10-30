# Pixel to world
def pixel_to_world(px: float, py: float, a: float, b: float, c: float, d: float, e: float, f: float):
    wx = a * px + b * py + c
    wy = d * px + e * py + f
    return wx, wy