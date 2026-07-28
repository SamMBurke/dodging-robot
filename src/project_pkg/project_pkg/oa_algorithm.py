import math
from typing import Optional, Tuple, List, Dict, Any
import numpy as np

# Utility Functions

def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def solve_quadratic_min_positive(a: float, b: float, c: float) -> Optional[float]:
    discriminant = b * b - 4.0 * a * c
    
    if discriminant < 0:
        return None
    
    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)
    
    positive_roots = []
    if t1 > 0:
        positive_roots.append(t1)
    if t2 > 0:
        positive_roots.append(t2)
    
    if not positive_roots:
        return None
    
    return min(positive_roots)


def local_to_global(px: float, py: float, robot_pose: Tuple[float, float, float]) -> Tuple[float, float]:
    x_r, y_r, theta_r = robot_pose
    cos_t = math.cos(theta_r)
    sin_t = math.sin(theta_r)
    
    X = x_r + px * cos_t - py * sin_t
    Y = y_r + px * sin_t + py * cos_t
    return (X, Y)


def global_to_local_vector(vgx: float, vgy: float, robot_pose: Tuple[float, float, float]) -> Tuple[float, float]:
    '''
    Rotates a free vector (e.g. velocity) from the global frame into the robot's local frame.
    Unlike global_to_local, this applies rotation only -- no translation -- since a velocity
    vector has no fixed position to translate. Use this for velocities; use global_to_local
    for positions/points.
    '''
    _, _, theta_r = robot_pose
    cos_t = math.cos(theta_r)
    sin_t = math.sin(theta_r)

    vpx = vgx * cos_t + vgy * sin_t
    vpy = -vgx * sin_t + vgy * cos_t
    return (vpx, vpy)


def global_to_local(gx: float, gy: float, robot_pose: Tuple[float, float, float]) -> Tuple[float, float]:
    x_r, y_r, theta_r = robot_pose
    cos_t = math.cos(theta_r)
    sin_t = math.sin(theta_r)
    
    dx = gx - x_r
    dy = gy - y_r
    
    px = dx * cos_t + dy * sin_t
    py = -dx * sin_t + dy * cos_t
    return (px, py)


# Path Planning Functions

def step1_gating(x_o: float, y_o: float, vx_o: float, vy_o: float, R: float) -> Tuple[bool, float]:
    
    v_sq = vx_o * vx_o + vy_o * vy_o
    
    current_dist = math.hypot(x_o, y_o)

    # Obstacle is stationary 
    if v_sq < 1e-8:
        is_threat = current_dist < R
        return is_threat, current_dist
    
    dot = x_o * vx_o + y_o * vy_o

    t_cpa = -dot / v_sq
    
    if t_cpa < 0:  # Obstacle is moving away
        d_cpa = current_dist
    else:  # Obstacle is approaching
        future_x = x_o + vx_o * t_cpa
        future_y = y_o + vy_o * t_cpa
        d_cpa = math.hypot(future_x, future_y)
    
    is_threat = d_cpa < R
    
    return is_threat, d_cpa


def step2_compute_ttc(x_o: float, y_o: float, vx_o: float, vy_o: float, R: float) -> Optional[float]:

    v_sq = vx_o * vx_o + vy_o * vy_o
    
    # Obstacle is stationary
    if v_sq < 1e-8:
        return None
    
    # Set up quadratic eqn to solve for TTC
    a = v_sq
    b = 2.0 * (x_o * vx_o + y_o * vy_o)
    c = (x_o * x_o + y_o * y_o) - R * R
 
    return solve_quadratic_min_positive(a, b, c)


def step2_generate_candidates(
    x_o: float,
    y_o: float,
    vx_o: float,
    vy_o: float,
    t_entry: Optional[float],
    R: float,
    robot_radius: float = 0.18,
    v_max: float = 0.22,
    omega_max: float = 2.84,
    num_angles: int = 72,
    num_radii: int = 10
) -> List[Dict[str, Any]]:

    # Real collision position
    contact_x = None
    contact_y = None
    if t_entry is not None and t_entry > 0:
        contact_x = x_o + vx_o * t_entry
        contact_y = y_o + vy_o * t_entry

    # Generate polar grid
    angles_deg = np.linspace(0, 360, num_angles, endpoint=False)
    angles_rad = [math.radians(deg) for deg in angles_deg]
    
    delta_r = R / num_radii
    radii = [delta_r * (i + 1) for i in range(num_radii)]

    candidates = []

    # Convert to Cartesian
    for theta in angles_rad:
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        
        for r in radii:
            px = r * cos_t
            py = r * sin_t
            dist = math.hypot(px, py)

            # Filter 1: Obstacle occupancy check (margin includes the robot's own footprint)
            dist_to_obs = math.hypot(px - x_o, py - y_o)
            if dist_to_obs < R + robot_radius:
                continue

            # Filter 2: Collision check (same footprint margin)
            if t_entry is not None and contact_x is not None:
                dist_to_contact = math.hypot(px - contact_x, py - contact_y)
                if dist_to_contact < R + robot_radius:
                    continue

            # Filter 3: Time check
            t_reach = float('inf')
            delta_theta_eff = 0.0
            use_reverse = False

            if t_entry is not None:
                target_angle = math.atan2(py, px)

                # If robot moves forward
                delta_fwd = math.atan2(math.sin(target_angle), math.cos(target_angle))
                t_fwd = abs(delta_fwd) / omega_max + dist / v_max

                # If robot reverses
                rear_angle = target_angle - math.pi
                delta_rev = math.atan2(math.sin(rear_angle), math.cos(rear_angle))
                t_rev = abs(delta_rev) / omega_max + dist / v_max

                if t_fwd <= t_rev:
                    t_reach = t_fwd
                    delta_theta_eff = delta_fwd
                    use_reverse = False
                else:
                    t_reach = t_rev
                    delta_theta_eff = delta_rev
                    use_reverse = True

                if t_reach >= t_entry:
                    continue
            else:
                t_reach = 0.0
                delta_theta_eff = math.atan2(math.sin(math.atan2(py, px)), math.cos(math.atan2(py, px)))
                use_reverse = False

            candidates.append({
                'px': px,
                'py': py,
                'dist': dist,
                'angle': theta,
                'delta_theta_eff': delta_theta_eff,
                'use_reverse': use_reverse,
                't_reach': t_reach,
                'is_backward': px < 0
            })

    return candidates


def step3_cost_function(
    candidates: List[Dict[str, Any]],
    t_entry: float,
    R: float,
    w_dist: float = 0.25,
    w_angle: float = 0.45,
    w_time: float = 0.30
) -> Optional[Dict[str, Any]]:

    if not candidates:
        return None

    # Check for safety
    if t_entry is None or t_entry <= 0:
        compute_time_cost = False
    else:
        compute_time_cost = True

    best_candidate = None
    best_cost = float('inf')

    for cand in candidates:
        # Distance Cost
        J_dist = cand['dist'] / R

        # Turn Cost
        J_angle = (2.0 * abs(cand['delta_theta_eff'])) / math.pi

        if J_angle > 1.0:
            J_angle = 1.0

        # Time Cost
        if compute_time_cost:
            J_time = cand['t_reach'] / t_entry
        else:
            J_time = 0.0

        # Compute cost function
        cost = w_dist * J_dist + w_angle * J_angle + w_time * J_time

        # Penalize reversing
        if cand['is_backward']:
            cost *= 1.5

        if cost < best_cost:
            best_cost = cost
            best_candidate = cand

    # 将计算出的最优代价存入字典，方便调试
    if best_candidate is not None:
        best_candidate['cost'] = best_cost

    return best_candidate


def step4_lock_global(
    best_local: Dict[str, Any],
    robot_pose: Tuple[float, float, float],
    locked_global: Optional[Tuple[float, float]],
    locked_cost: float,
    locked_reverse: bool,
    locked_delta_theta: float,
    t_entry: Optional[float],
    x_o: float,
    y_o: float,
    vx_o: float,
    vy_o: float,
    R: float,
    v_max: float,
    omega_max: float,
    rho: float = 0.85
) -> Tuple[Tuple[float, float], float, bool, float]:
    
    # Convert from local frame to global frame
    X_new, Y_new = local_to_global(
        best_local['px'], 
        best_local['py'], 
        robot_pose
    )
    cost_new = best_local['cost']
    use_reverse_new = best_local['use_reverse']
    delta_theta_new = best_local['delta_theta_eff']

    # If no old evasion point
    if locked_global is None:
        return (X_new, Y_new), cost_new, use_reverse_new, delta_theta_new

    # If old evasion point exists
    X_old, Y_old = locked_global

    px_old, py_old = global_to_local(X_old, Y_old, robot_pose)
    dist_old = math.hypot(px_old, py_old)

    # Check if old point is still safe
    is_old_safe = True

    # Apply same 3 filters 
    if t_entry is not None and t_entry > 0:
        # Filter 1
        dist_to_obs = math.hypot(px_old - x_o, py_old - y_o)
        if dist_to_obs < R:
            is_old_safe = False

        # Filter 2
        if is_old_safe:
            contact_x = x_o + vx_o * t_entry
            contact_y = y_o + vy_o * t_entry
            dist_to_contact = math.hypot(px_old - contact_x, py_old - contact_y)
            if dist_to_contact < R:
                is_old_safe = False

        # Filter 3
        if is_old_safe:
            target_angle_old = math.atan2(py_old, px_old)
            delta_fwd_old = wrap_angle(target_angle_old)
            delta_rev_old = wrap_angle(target_angle_old - math.pi)
            
            t_fwd_old = abs(delta_fwd_old) / omega_max + dist_old / v_max
            t_rev_old = abs(delta_rev_old) / omega_max + dist_old / v_max
            t_reach_old = min(t_fwd_old, t_rev_old)
            
            if t_reach_old >= t_entry:
                is_old_safe = False
    else:
        # i.e. Obstacle stops (Not in scope, for safety)
        dist_to_obs = math.hypot(px_old - x_o, py_old - y_o)
        if dist_to_obs < R:
            is_old_safe = False

    # Update evasion point if necessary
    if not is_old_safe:
        return (X_new, Y_new), cost_new, use_reverse_new, delta_theta_new
    else: # Update point only when new point is significantly better
        if cost_new < locked_cost * rho:
            return (X_new, Y_new), cost_new, use_reverse_new, delta_theta_new
        else:
            return locked_global, locked_cost, locked_reverse, locked_delta_theta


def step5_compute_velocity(
    locked_global: Tuple[float, float],
    robot_pose: Tuple[float, float, float],
    use_reverse: bool,
    delta_theta_eff: float,
    v_max: float,
    omega_max: float,
    wheel_base: float,
    Kv: float = 0.6,
    Kw: float = 0.8,
    alpha: float = 0.3,
    v_prev: float = 0.0,
    omega_prev: float = 0.0,
    v_dead: float = 0.005,
    omega_dead: float = 0.005,
    stop_dist: float = 0.05,
) -> Tuple[float, float, float, float, float, float]:

    X_locked, Y_locked = locked_global
    x_r, y_r, theta_r = robot_pose

    dx = X_locked - x_r
    dy = Y_locked - y_r
    dist = math.hypot(dx, dy)

    # If robot is close enough, do not move (prevent jitter)
    if dist < stop_dist:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    # Angular
    omega_raw = Kw * delta_theta_eff   
    omega_cmd = max(-omega_max, min(omega_max, omega_raw))

    # Linear
    V_base = max(0.0, min(v_max, Kv * dist))

    # When orientation is far off, robot focuses on turning rather than driving
    V_scaled = V_base * math.cos(delta_theta_eff)

    if use_reverse:
        V_cmd = -V_scaled
    else:
        V_cmd = V_scaled

    # Smooth velocity command using EMA
    V_smooth = alpha * V_cmd + (1.0 - alpha) * v_prev
    omega_smooth = alpha * omega_cmd + (1.0 - alpha) * omega_prev

    # Mitigate command jitter
    V_out = V_smooth if abs(V_smooth) > v_dead else 0.0
    omega_out = omega_smooth if abs(omega_smooth) > omega_dead else 0.0

   
    V_R = V_out + (wheel_base / 2.0) * omega_out
    V_L = V_out - (wheel_base / 2.0) * omega_out

    # Not sure if we need V or omega
    # omega_R = (V_out + (wheel_base / 2.0) * omega_out) / wheel radius
    # omega_L = (V_out - (wheel_base / 2.0) * omega_out) / wheel radius

    return V_R, V_L, V_out, omega_out, V_smooth, omega_smooth