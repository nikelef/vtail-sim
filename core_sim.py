from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Dict, Any, Tuple
import math
import random

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def vlen(x: float, y: float) -> float:
    return math.hypot(x, y)

# -----------------------------
# V-tail ruddervator mixer
# -----------------------------
# Convention:
#   pitch_cmd  in [-1..+1]  ( + = pull/pitch-up )
#   yaw_cmd    in [-1..+1]  ( + = yaw left )
#
# We generate left/right surface deflections in [-1..+1]
# Typical mixer:
#   left  = pitch + yaw
#   right = pitch - yaw
#
# Then re-derive effective pitch/yaw after saturation
# to reflect real-world loss of authority when one surface saturates.
def ruddervator_mix(pitch_cmd: float, yaw_cmd: float) -> Tuple[float, float, float, float]:
    pitch_cmd = clamp(pitch_cmd, -1.0, 1.0)
    yaw_cmd = clamp(yaw_cmd, -1.0, 1.0)

    left = clamp(pitch_cmd + yaw_cmd, -1.0, 1.0)
    right = clamp(pitch_cmd - yaw_cmd, -1.0, 1.0)

    # Inverse mix (approx) from saturated surfaces
    pitch_eff = clamp(0.5 * (left + right), -1.0, 1.0)
    yaw_eff = clamp(0.5 * (left - right), -1.0, 1.0)
    return left, right, pitch_eff, yaw_eff

@dataclass
class SimParams:
    # Gravity
    g: float = 9.81

    # Start
    start_alt_m: float = 900.0
    start_x_m: float = 0.0
    start_y_m: float = 0.0
    start_heading_deg: float = 0.0

    # Target
    target_x_m: float = 2400.0
    target_y_m: float = 700.0
    target_radius_m: float = 60.0

    # Aerodynamic-ish parameters (playable, not certified)
    mass_kg: float = 1100.0
    v_min: float = 16.0
    v_max: float = 60.0

    # “Trim” reference values
    v_trim: float = 26.0

    # Pitch -> glide ratio mapping envelope
    gr_min: float = 4.0
    gr_max: float = 18.0

    # Stall region (excessive pitch-up or too slow)
    stall_pitch_eff: float = 0.75
    stall_v: float = 18.0
    stall_gr_factor: float = 0.55
    stall_sink_boost: float = 1.35

    # Control responsiveness
    pitch_response_per_s: float = 1.6
    yaw_rate_deg_s: float = 55.0

    # Bank model from yaw command (turning requires bank → extra sink)
    bank_max_deg: float = 40.0
    bank_response_per_s: float = 2.0
    bank_sink_factor: float = 0.30  # additional sink multiplier at max bank

    # Drag / energy
    # We treat drag power ~ k * v^3. This is a simple way to prevent “free speed”.
    drag_k: float = 0.012

    # Wind
    wind_x_mps: float = 2.2
    wind_y_mps: float = -1.0
    gust_mps: float = 0.35

    # Integration
    ground_alt_m: float = 0.0

@dataclass
class SimState:
    x_m: float
    y_m: float
    alt_m: float
    heading_rad: float

    # Controls (pilot commands)
    pitch_cmd: float = 0.15
    yaw_cmd: float = 0.0

    # Surface deflections
    left_rv: float = 0.0
    right_rv: float = 0.0

    # Effective control after saturation
    pitch_eff: float = 0.15
    yaw_eff: float = 0.0

    # Aircraft attitude proxies
    bank_deg: float = 0.0

    # Energy/speed
    v_air_mps: float = 26.0

    # Time
    t_s: float = 0.0

    alive: bool = True
    impact: Dict[str, Any] | None = None

    # History for dashboard
    hist: List[Dict[str, float]] = field(default_factory=list)

def reset_state(p: SimParams) -> SimState:
    return SimState(
        x_m=p.start_x_m,
        y_m=p.start_y_m,
        alt_m=p.start_alt_m,
        heading_rad=math.radians(p.start_heading_deg),
        pitch_cmd=0.15,
        yaw_cmd=0.0,
        v_air_mps=p.v_trim,
        bank_deg=0.0,
    )

def compute_glide_ratio(p: SimParams, pitch_eff: float) -> float:
    # Map pitch_eff [-1..+1] -> [gr_min..gr_max]
    t = (pitch_eff + 1.0) * 0.5
    return p.gr_min + (p.gr_max - p.gr_min) * t

def step_sim(p: SimParams, s: SimState, dt: float, pitch_in: float, yaw_in: float) -> None:
    if not s.alive:
        return

    # Update pilot commands
    s.pitch_cmd = clamp(s.pitch_cmd + pitch_in * 0.9 * dt, -1.0, 1.0)
    s.yaw_cmd = clamp(yaw_in, -1.0, 1.0)

    # Ruddervator mixing (explicit left/right)
    s.left_rv, s.right_rv, pitch_eff_cmd, yaw_eff_cmd = ruddervator_mix(s.pitch_cmd, s.yaw_cmd)

    # Smooth effective pitch (airframe response)
    a = clamp(p.pitch_response_per_s * dt, 0.0, 1.0)
    s.pitch_eff += (pitch_eff_cmd - s.pitch_eff) * a
    s.yaw_eff = yaw_eff_cmd

    # Bank proxy follows yaw magnitude (turn coordination)
    bank_target = clamp(abs(s.yaw_eff) * p.bank_max_deg, 0.0, p.bank_max_deg)
    b = clamp(p.bank_response_per_s * dt, 0.0, 1.0)
    s.bank_deg += (bank_target - s.bank_deg) * b

    # Compute glide ratio from pitch
    gr = compute_glide_ratio(p, s.pitch_eff)

    # Stall logic
    stall = (s.pitch_eff > p.stall_pitch_eff) or (s.v_air_mps < p.stall_v)
    if stall:
        gr *= p.stall_gr_factor

    # Convert glide ratio to flight path angle gamma:
    # tan(gamma)=1/GR => sink = V * sin(gamma)
    gamma = math.atan2(1.0, max(gr, 0.1))

    # Base sink and forward component (air-relative)
    sink = s.v_air_mps * math.sin(gamma)
    fwd = s.v_air_mps * math.cos(gamma)

    # Turn-induced sink (bank penalty)
    bank_frac = clamp(s.bank_deg / p.bank_max_deg, 0.0, 1.0) if p.bank_max_deg > 0 else 0.0
    sink *= (1.0 + p.bank_sink_factor * (bank_frac ** 2))

    if stall:
        sink *= p.stall_sink_boost

    # Heading update from yaw effective
    yaw_rate = math.radians(p.yaw_rate_deg_s) * s.yaw_eff
    s.heading_rad += yaw_rate * dt

    # Wind with gust
    gust_x = (random.random() * 2 - 1) * p.gust_mps
    gust_y = (random.random() * 2 - 1) * p.gust_mps
    wind_x = p.wind_x_mps + gust_x
    wind_y = p.wind_y_mps + gust_y

    # Air-relative velocity
    vx_air = fwd * math.cos(s.heading_rad)
    vy_air = fwd * math.sin(s.heading_rad)

    # Ground-relative velocity
    vx = vx_air + wind_x
    vy = vy_air + wind_y

    # Update position and altitude
    s.x_m += vx * dt
    s.y_m += vy * dt
    s.alt_m -= sink * dt

    # Energy update (potential ↔ kinetic + drag)
    # Specific mechanical energy e = g*h + 0.5*v^2.
    # As h decreases, v tends to increase, but drag reduces it.
    e = p.g * s.alt_m + 0.5 * (s.v_air_mps ** 2)

    # Drag power ~ k * v^3 => energy per mass per second ~ k * v^3
    # Apply as e -= drag * dt
    e -= p.drag_k * (s.v_air_mps ** 3) * dt

    # Prevent negative
    e = max(0.0, e)

    # Convert back to v from e and h:
    # 0.5*v^2 = e - g*h
    kin = max(0.0, e - p.g * s.alt_m)
    s.v_air_mps = math.sqrt(2.0 * kin)
    s.v_air_mps = clamp(s.v_air_mps, p.v_min, p.v_max)

    # Time
    s.t_s += dt

    # History sample (sparse but adequate)
    if (len(s.hist) == 0) or (s.t_s - s.hist[-1]["t_s"] > 0.10):
        dx = s.x_m - p.target_x_m
        dy = s.y_m - p.target_y_m
        miss = math.hypot(dx, dy)
        s.hist.append({
            "t_s": s.t_s,
            "x_m": s.x_m,
            "y_m": s.y_m,
            "alt_m": s.alt_m,
            "v_air_mps": s.v_air_mps,
            "heading_deg": (math.degrees(s.heading_rad) % 360.0),
            "pitch_cmd": s.pitch_cmd,
            "yaw_cmd": s.yaw_cmd,
            "left_rv": s.left_rv,
            "right_rv": s.right_rv,
            "pitch_eff": s.pitch_eff,
            "yaw_eff": s.yaw_eff,
            "bank_deg": s.bank_deg,
            "miss_m": miss,
            "stall": 1.0 if stall else 0.0,
            "gr": gr,
        })

    # Impact check
    if s.alt_m <= p.ground_alt_m:
        s.alt_m = p.ground_alt_m
        s.alive = False
        miss = math.hypot(s.x_m - p.target_x_m, s.y_m - p.target_y_m)
        s.impact = {
            "t_s": s.t_s,
            "x_m": s.x_m,
            "y_m": s.y_m,
            "miss_m": miss,
            "on_target": miss <= p.target_radius_m,
        }
