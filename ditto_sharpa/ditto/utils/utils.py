import time
import numpy as np

def precise_sleep(duration: float, precise_duration: float = 0.001) -> float:
    """
    Precise sleep using hybrid approach: time.sleep() for bulk, busy-wait for precision.
    
    Uses time.sleep() for most of the duration, then busy-waits with time.perf_counter()
    for the final portion to achieve microsecond precision.
    
    Args:
        duration: Sleep duration in seconds
        precise_duration: Amount of time to leave for busy-wait (default 1ms)
    """
    assert duration >= 0, "Duration must be non-negative"
    assert precise_duration >= 0, "Precise duration must be non-negative"
    
    start_time = time.perf_counter()
    end_time = start_time + duration
    
    # time.sleep() for all of "duration" except for "precise_duration"
    if duration > precise_duration:
        time.sleep(duration - precise_duration) 
    
    # busy-wait for the remaining time with microsecond precision
    while time.perf_counter() < end_time:
        pass
    

def interpolate_array(current: np.ndarray | list, 
                      target: np.ndarray | list, 
                      DT: float, 
                      dt: float) -> np.ndarray:
    """
    Linearly interpolates from `current` to `target` over time `DT`,
    with steps of `dt`. Excludes 'current' from interpolated trajectory.

    Args:
        current: numpy array of current values
        target: numpy array of target values
        DT: total movement time
        dt: time step size

    Returns:
        traj: numpy array of shape (N_steps, len(current)),
              where traj[-1] == target
    """
    if not isinstance(current, np.ndarray):
        current = np.array(current, dtype=float)
    if not isinstance(target, np.ndarray):
        target = np.array(target, dtype=float)

    assert current.shape == target.shape, "Shapes must match"
    assert DT >= 0, "DT must be non-negative"
    
    if DT <= 0 or np.allclose(current, target):
        return np.array([target])
    
    # Number of steps
    dt = max(dt, 1e-9)  # prevent division by zero
    steps = max(1, int(np.floor(DT / dt)))
    
    # Normalized interpolation alpha(t) - exclude current (start from first step)
    # Use steps points, starting from alpha > 0 to exclude current position
    alphas = np.linspace(1.0 / steps, 1.0, steps).reshape(-1, 1)
    
    # Linear interpolation for all dimensions at once
    traj = current + alphas * (target - current)
    
    return traj