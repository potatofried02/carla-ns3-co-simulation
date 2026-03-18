# Safety evaluation for two-vehicle EB scenario (Leader id=0, Follower id=1).
# Metrics: collision, TTC violation ratio, min inter-vehicle distance.
# See Project_Outline/Step3_Benchmark_Implementation.md §2.1

import json
import math
import os
from typing import Dict, List, Any, Optional

# Thresholds consistent with the Step 3 document
D_CRIT_M = 1.5   # Longitudinal distance below this value is considered a "collision/contact"
TAU_TTC_SEC = 1.5  # TTC below this value is considered a violation (expected time to collision is too short)


def _rad(deg: float) -> float:
    return math.radians(deg)


def _longitudinal_distance_and_relative_speed(
    leader: Dict, follower: Dict
) -> tuple:
    """
    Take the longitudinal direction based on the Leader's heading, calculate the longitudinal distance D and relative speed Δv.
    D = (pos_L - pos_F)·forward_L; Δv = v_F_long - v_L_long (Δv > 0 when the follower is catching up to the leader).
    Returns (D_m, delta_v_m_s). If vehicle fields are missing, returns (float('inf'), 0.0) to avoid false collision detection.
    """
    try:
        pL = leader["position"]
        pF = follower["position"]
        vL = leader["velocity"]
        vF = follower["velocity"]
        yaw_deg = leader.get("heading", 0.0)
    except (KeyError, TypeError):
        return (float("inf"), 0.0)

    yaw = _rad(yaw_deg)
    fx = math.cos(yaw)
    fy = math.sin(yaw)
    # Longitudinal distance (center distance): Projection of the vector from Follower to Leader on the Leader's heading (D_center > 0 if F is behind L)
    dx = pL["x"] - pF["x"]
    dy = pL["y"] - pF["y"]
    D_center = dx * fx + dy * fy

    # If vehicle length information is available, convert center distance to "gap from leader's rear to follower's front"
    # gap = D_center - (L_length/2 + F_length/2), gap <= 0 is considered contact/collision
    L_len = leader.get("length_m")
    F_len = follower.get("length_m")
    if isinstance(L_len, (int, float)) and isinstance(F_len, (int, float)):
        half_sum = 0.5 * (L_len + F_len)
        D = D_center - half_sum
    else:
        D = D_center

    vL_long = vL["x"] * fx + vL["y"] * fy
    vF_long = vF["x"] * fx + vF["y"] * fy
    delta_v = vF_long - vL_long
    return (D, delta_v)


def compute_ttc(D_m: float, delta_v_m_s: float) -> float:
    """TTC = D / Δv when Δv > 0 (closing), else +inf."""
    if delta_v_m_s <= 0:
        return float("inf")
    return D_m / delta_v_m_s


def compute_gap_and_ttc(vehicle_data: List[Dict]) -> tuple:
    """
    Calculate the gap from leader's rear to follower's front (meters) and TTC (seconds) using the current frame's vehicle_data.
    Used for method=2 (world state approximation) and others. Convention: vehicle_data[0]=Leader, vehicle_data[1]=Follower.
    Returns (gap_m, ttc_sec); if data is invalid, returns (float('inf'), float('inf')).
    """
    if not vehicle_data or len(vehicle_data) < 2:
        return (float("inf"), float("inf"))
    leader, follower = vehicle_data[0], vehicle_data[1]
    D, delta_v = _longitudinal_distance_and_relative_speed(leader, follower)
    ttc = compute_ttc(D, delta_v)
    return (D, ttc)


class SafetyEvalAccumulator:
    """
    Updates frame by frame, calculates collision, TTC violation ratio, minimum inter-vehicle distance, etc., 
    after simulation ends, and can output safety_eval_summary.json.
    Convention: in vehicle_data, id=0 is Leader, id=1 is Follower.
    """

    def __init__(
        self,
        d_crit_m: float = D_CRIT_M,
        tau_ttc_sec: float = TAU_TTC_SEC,
    ):
        self.d_crit_m = d_crit_m
        self.tau_ttc_sec = tau_ttc_sec
        self.collision_occurred = False
        self.collision_relative_speed_ms = 0.0
        self.min_inter_vehicle_distance_m = float("inf")
        self.ttc_violation_time_sec = 0.0
        self.total_time_sec = 0.0
        self._dt_sec = 0.1  # Inferred from the time step t in update
        # Correction for collision relative speed "zeroing" caused by sampling: 
        # record the previous frame state (used to get the previous frame's closing speed at the time of first contact)
        self._prev_D = None
        self._prev_delta_v = None

    def update(self, sim_time_sec: float, vehicle_data: List[Dict]) -> None:
        """
        Called once per frame. vehicle_data is the list returned by collect_vehicle_data, in the order [Leader, Follower].
        """
        if not vehicle_data or len(vehicle_data) < 2:
            return
        leader = vehicle_data[0]
        follower = vehicle_data[1]
        D, delta_v = _longitudinal_distance_and_relative_speed(leader, follower)

        # Total duration: approximated using the current sim_time (the last frame will add dt once more, inferred from the passed step)
        self.total_time_sec = sim_time_sec

        # Minimum inter-vehicle distance
        if D < self.min_inter_vehicle_distance_m:
            self.min_inter_vehicle_distance_m = D

        # Collision: recorded when D <= d_crit for the first time
        if D <= self.d_crit_m and not self.collision_occurred:
            self.collision_occurred = True
            rel = abs(delta_v)
            # If already "crashed and stopped/overlapped" in this frame, rel may be ≈ 0; 
            # use previous frame's closing speed to approximate the first contact speed
            if (rel < 1e-6) and (self._prev_delta_v is not None):
                rel = abs(self._prev_delta_v)
            self.collision_relative_speed_ms = rel

        # TTC violation: if TTC < tau and TTC is finite, this frame is counted as a violation
        ttc = compute_ttc(D, delta_v)
        if ttc < self.tau_ttc_sec and math.isfinite(ttc):
            self.ttc_violation_time_sec += self._dt_sec

        # update prev frame state
        self._prev_D = D
        self._prev_delta_v = delta_v

    def set_dt(self, dt_sec: float) -> None:
        """Main loop step size, used for TTC violation time accumulation. Default is 0.1."""
        self._dt_sec = dt_sec

    def get_summary(
        self,
        run_timestamp: str = "",
        run_label: str = "",
        method: int = 1,
        world_duration_sec: Optional[float] = None,
        extra_params: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Returns a dictionary serializable to safety_eval_summary.json. If world_duration_sec is provided, it's used for the TTC ratio denominator, otherwise total_time_sec + dt is used."""
        duration = world_duration_sec
        if duration is None or duration <= 0:
            duration = self.total_time_sec + self._dt_sec
        duration = max(duration, 1e-6)
        ratio = self.ttc_violation_time_sec / duration
        min_dist = self.min_inter_vehicle_distance_m if math.isfinite(self.min_inter_vehicle_distance_m) else None

        params_dict = {
            "d_crit_m": self.d_crit_m,
            "tau_ttc_sec": self.tau_ttc_sec,
        }
        if extra_params:
            params_dict.update(extra_params)

        summary = {
            "run_timestamp": run_timestamp,
            "run_label": run_label,
            "method": method,
            "collision_occurred": self.collision_occurred,
            "collision_relative_speed_m_s": round(self.collision_relative_speed_ms, 4),
            "min_inter_vehicle_distance_m": round(min_dist, 4) if min_dist is not None else None,
            "ttc_violation_ratio": round(ratio, 6),
            "ttc_violation_seconds": round(self.ttc_violation_time_sec, 4),
            "simulation_duration_sec": round(duration, 4),
            "params": params_dict,
            "_metrics_description": {
                "collision_occurred": "Whether a collision occurred (longitudinal distance between two vehicles was ever <= critical distance d_crit)",
                "collision_relative_speed_m_s": "If a collision occurred, the relative speed at the time of first contact (m/s), otherwise 0",
                "min_inter_vehicle_distance_m": "The minimum longitudinal distance (m) between the two vehicles during the simulation, smaller is more dangerous",
                "ttc_violation_ratio": "TTC violation time ratio: the proportion of time when 'expected time to collision < tau_ttc seconds' during the simulation, 0~1, higher is more dangerous",
                "ttc_violation_seconds": "Cumulative TTC violation seconds (for intuitive understanding)",
                "simulation_duration_sec": "Total simulation duration (s)",
            },
        }
        return summary

    def write_summary(
        self,
        output_dir: str,
        run_timestamp: str = "",
        run_label: str = "",
        method: int = 1,
        world_duration_sec: Optional[float] = None,
        extra_params: Optional[Dict[str, Any]] = None,
    ) -> Optional[str]:
        """Writes safety_eval_summary to output_dir/safety_eval_summary.json. Returns the written path, or None on failure."""
        try:
            os.makedirs(output_dir, exist_ok=True)
            path = os.path.join(output_dir, "safety_eval_summary.json")
            summary = self.get_summary(
                run_timestamp=run_timestamp,
                run_label=run_label,
                method=method,
                world_duration_sec=world_duration_sec,
                extra_params=extra_params,
            )
            with open(path, "w", encoding="utf-8") as f:
                json.dump(summary, f, ensure_ascii=False, indent=2)
            return path
        except Exception:
            return None