def map_axis(raw_value):
    abs_val = abs(raw_value)
    if abs_val <= 0.3:
        return 0.0
    mapped = (abs_val - 0.3) / (1.0 - 0.3)
    sign = 1 if raw_value > 0 else -1
    return round(mapped * sign, 2)

def map_range(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
