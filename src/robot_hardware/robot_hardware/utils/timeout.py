# Freshness check
def is_data_fresh(now_time, last_time, timeout_sec):
    if last_time is None:
        return False
    if isinstance(now_time, (int, float)) and isinstance(last_time, (int, float)):
        return abs(now_time - last_time) < timeout_sec
    return (now_time - last_time).nanoseconds / 1e9 < timeout_sec