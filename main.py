import argparse
import time
import subprocess
from datetime import datetime
from pathlib import Path
from src.carla.carla_connector import (
    connect_to_carla,
    spawn_leader_follower_pair,
    spawn_leader_follower_pair_fixed,
    apply_control,
    destroy_actors,
    add_camera_to_vehicle,
    destroy_sensors,
    add_overhead_camera_to_vehicle,
)
from src.bridge.carla_ns3_bridge import CarlaNs3Bridge
from src.common.logger import logger
from src.carla.vehicle_data import collect_vehicle_data
from src.common.vehicle_data_logger import vehicle_data_logger
from src.common.visualization import VehicleDataVisualizer
from src.common.safety_eval import SafetyEvalAccumulator, compute_gap_and_ttc
from config.settings import CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT, MAP_NAME

# Default parameters for two-vehicle scenario (consistent with step 3 document)
DEFAULT_T0 = 2.5        # Leader hard brakes at T0 seconds (to ensure rear-end collision, leader brakes early)
DEFAULT_GAP_M = 20.0    # Initial gap between the two vehicles (meters)
DEFAULT_DT = 0.1        # Control step (seconds), one step per tick in synchronous mode
DEFAULT_MAX_TIME = 30.0 # Maximum simulation time (seconds)
DEFAULT_THROTTLE = 0.8  # Throttle for both vehicles before T0, large throttle to accelerate close to target speed ASAP
LEADER_BRAKE_SCALE = 1.5  # Leader braking is weak in simulation, multiply by this factor and cap at 1.0 when applying


def main():
    parser = argparse.ArgumentParser(description="Two-vehicle Emergency Braking scenario (Leader/Follower).")
    parser.add_argument("--method", type=int, default=1, choices=[1, 2, 3, 4],
                        help="Follower reaction strategy: 1=No brake(baseline), 2=Sensor only, 3=V2X 802.11p, 4=V2X 5G")
    parser.add_argument("--t0", type=float, default=1.5, help=f"Leader hard brake time(s), default 1.5")
    parser.add_argument("--v0-leader", type=float, default=20.0, help=f"Leader initial speed(m/s), default 20.0")
    parser.add_argument("--v0-follower", type=float, default=20.0, help=f"Follower initial speed(m/s), default 20.0")
    parser.add_argument("--brake-leader", type=float, default=1.0, help=f"Leader brake intensity(0.0-1.0), default 1.0")
    parser.add_argument("--brake-follower", type=float, default=1.0, help=f"Follower brake intensity(0.0-1.0), default 1.0")
    parser.add_argument("--gap", type=float, default=DEFAULT_GAP_M, help=f"Initial gap between vehicles(m), default {DEFAULT_GAP_M}")
    parser.add_argument("--max-time", type=float, default=8.0, help=f"Max simulation time(s), default 8.0")
    parser.add_argument("--scene", type=str, default="default",
                        choices=["default", "california_urban", "california_highway"],
                        help="Scene layout: default=By spawn point+gap; california_urban/highway=Fixed coordinates facing -Y")
    parser.add_argument("--overhead-height", type=float, default=120.0,
                        help="Overhead camera height(m), default 120 to cover large gap car-following (approx 4x)")
    parser.add_argument("--spawn-index", type=int, default=0, help="Map spawn point index(Leader pos), valid only when scene=default")
    # method=2 world state approximation: trigger conditions and reaction delay
    parser.add_argument("--sensor-reaction-delay", type=float, default=0.5,
                        help="Reaction delay from 'perceiving danger' to braking for method=2 (s), default 0.5")
    parser.add_argument("--sensor-gap-threshold", type=float, default=10.0,
                        help="Gap trigger threshold for method=2 (m), danger if gap < this value; <=0 means not used")
    parser.add_argument("--sensor-ttc-threshold", type=float, default=3.0,
                        help="TTC trigger threshold for method=2 (s), danger if TTC < this value; <=0 means not used")
    parser.add_argument("--sensor-debug", action="store_true",
                        help="Print gap/TTC and trigger/brake logs per frame for method=2 debugging")
    # Step 4.5: M3/M4 Channel Regime (must match ns-3 --regime parameter)
    parser.add_argument("--regime", type=int, default=None, choices=[1, 2, 3],
                        help="Channel regime for M3/M4: 1=Good(R1), 2=Medium(R2), 3=Bad(R3). Default 1 when method is 3 or 4.")
    # Sync/Pacing: To prevent CARLA simulation time from running much faster than ns-3 realtime 
    # (causing brake_warning to always be "too late"), method=3/4 defaults to real-time pacing 
    # using DEFAULT_DT; this parameter can override it.
    parser.add_argument("--sleep-per-tick-sec", type=float, default=None,
                        help="Wall-clock sleep time after each tick(s). Default: DEFAULT_DT for method=3/4; 0.02 for others.")
    # lock-step: Strict synchronization (Python sends tick(seq,t,vehicles) -> waits for ns-3 tick_ack(seq) -> then world.tick())
    parser.add_argument("--lockstep", type=int, default=None, choices=[0, 1],
                        help="Whether to enable strict lock-step synchronization. Default: enabled(1) for method=3/4, disabled(0) for others.")
    args = parser.parse_args()

    # If regime is not specified for M3/M4, default to 1 (R1 Good Channel)
    if args.regime is None and args.method in (3, 4):
        args.regime = 1
        logger.info("method=%d: --regime not set, using 1 (R1 Good). Start ns-3 with same: --regime=1", args.method)

    logger.info(f"Run config: method={args.method} t0={args.t0}s v0_L={args.v0_leader}m/s v0_F={args.v0_follower}m/s gap={args.gap}m max_time={args.max_time}s")
    if args.method == 2:
        logger.info(f"  method=2 sensor: reaction_delay={args.sensor_reaction_delay}s gap_thr={args.sensor_gap_threshold}m ttc_thr={args.sensor_ttc_threshold}s debug={args.sensor_debug}")
    logger.info("Connecting to Carla simulator (synchronous mode for deterministic control)")
    client, world = connect_to_carla(
        CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT,
        synchronous=True,
        fixed_delta_seconds=DEFAULT_DT,
    )
    if not client or not world:
        logger.error("Failed to connect to Carla. Make sure the simulator is running.")
        return 1

    logger.info("Successfully connected to Carla simulator!")

    # Default pacing choice
    if args.sleep_per_tick_sec is None:
        args.sleep_per_tick_sec = DEFAULT_DT if args.method in (3, 4) else 0.02
    if args.lockstep is None:
        args.lockstep = 1 if args.method in (3, 4) else 0
    if args.lockstep:
        # lockstep implies we should not rely on wall-clock sleeps for correctness
        logger.info("Lock-step enabled: Python will wait for ns-3 tick_ack each tick.")
    logger.info(f"Tick pacing: sleep_per_tick_sec={args.sleep_per_tick_sec} (DEFAULT_DT={DEFAULT_DT})")

    current_map_name = world.get_map().name
    if MAP_NAME not in current_map_name:
        logger.info(f"Loading map {MAP_NAME} (current: {current_map_name})...")
        try:
            world = client.load_world(MAP_NAME)
        except RuntimeError as e:
            logger.error(f"Failed to load map {MAP_NAME}: {e}")
            return 1
        # Need to re-enable synchronous mode after map change
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = DEFAULT_DT
        world.apply_settings(settings)
    else:
        logger.info(f"Map is already {MAP_NAME}, skipping load.")

    if not world:
        logger.error(f"Failed to load map {MAP_NAME}.")
        return 1
    logger.info(f"Map {MAP_NAME} loaded successfully!")

    # Create an independent output directory for this simulation, e.g., temp/Simu022518_1755
    project_root = Path(__file__).resolve().parent
    run_tag = datetime.now().strftime("Simu%m%d_%H%M%S")
    run_dir = project_root / "temp" / run_tag
    run_dir.mkdir(parents=True, exist_ok=True)
    logger.info(f"Current simulation output directory: {run_dir}")

    # Spawn the two vehicles: default uses spawn point + gap; California scenes use fixed coordinates and orientation
    if args.scene == "default":
        leader, follower = spawn_leader_follower_pair(
            world, gap_m=args.gap, vehicle_type="coupe_2020", spawn_point_index=args.spawn_index
        )
    else:
        leader, follower = spawn_leader_follower_pair_fixed(
            world, scene=args.scene, vehicle_type="coupe_2020", gap_m=args.gap
        )
    if not leader or not follower:
        logger.error("Failed to spawn leader/follower pair. Try another --spawn-index or --scene.")
        return 1

    all_vehicles = [leader, follower]  # 0=Leader, 1=Follower, consistent with NS-3 ID

    # Set initial velocity (Force -Y direction for California scenes, consistent with spawn orientation)
    import math
    import carla

    if args.scene in ("california_urban", "california_highway"):
        # Driving direction is fixed to world coordinate -Y
        leader.set_target_velocity(carla.Vector3D(0.0, -args.v0_leader, 0.0))
        follower.set_target_velocity(carla.Vector3D(0.0, -args.v0_follower, 0.0))
    else:
        yaw_L = math.radians(leader.get_transform().rotation.yaw)
        leader.set_target_velocity(carla.Vector3D(args.v0_leader * math.cos(yaw_L), args.v0_leader * math.sin(yaw_L), 0.0))
        yaw_F = math.radians(follower.get_transform().rotation.yaw)
        follower.set_target_velocity(carla.Vector3D(args.v0_follower * math.cos(yaw_F), args.v0_follower * math.sin(yaw_F), 0.0))
    leader.apply_control(carla.VehicleControl(throttle=DEFAULT_THROTTLE, brake=0.0, steer=0.0))
    follower.apply_control(carla.VehicleControl(throttle=DEFAULT_THROTTLE, brake=0.0, steer=0.0))

    all_sensors = []
    # Overhead camera frames directory: Simu.../Video/frames
    video_frames_dir = run_dir / "Video" / "frames"
    overhead_camera = add_overhead_camera_to_vehicle(
        world,
        leader,
        output_dir=video_frames_dir,
        height=args.overhead_height,
        sensor_tick=DEFAULT_DT,  # Consistent with simulation step size, capture every frame (approx. 10 FPS)
    )
    if overhead_camera:
        all_sensors.append(overhead_camera)

    # Write vehicle_data.json to the current simulation directory and clear old frames
    vehicle_data_logger.set_run_dir(str(run_dir))
    vehicle_data_logger.reset()
    # Write bridge's v2x_eval_summary to the current simulation directory too
    bridge = CarlaNs3Bridge(run_dir=str(run_dir))
    bridge.start()

    # Safety metrics: update every frame, write safety_eval_summary.json at the end
    safety_eval = SafetyEvalAccumulator()
    safety_eval.set_dt(DEFAULT_DT)

    t = 0.0
    tick_seq = 0
    leader_braked = False
    follower_braked = False
    follower_brake_applied_t = None
    brake_warning_first_seen_t = None
    # method=2: World state approximation, count reaction delay from "first satisfying gap/TTC trigger"; 
    # judged using vehicle_data from previous frame
    sensor_danger_since = None
    vehicle_data_for_control = None

    # Main loop end condition: controlled only by t >= max_time; NS-3 simulation_end is only used to stop Bridge and write evaluation
    try:
        while t < args.max_time:
            # Control logic: Both vehicles use throttle before T0, Leader hard brakes from T0; Follower branches by method
            if t < args.t0:
                apply_control(leader, throttle=DEFAULT_THROTTLE)
                apply_control(follower, throttle=DEFAULT_THROTTLE)
            else:
                if not leader_braked:
                    eff_brake_l = min(1.0, args.brake_leader * LEADER_BRAKE_SCALE)
                    apply_control(leader, brake=eff_brake_l)
                    leader_braked = True
                    logger.info(f"t={t:.1f}s: Leader brake applied (effective={eff_brake_l}).")
                else:
                    eff_brake_l = min(1.0, args.brake_leader * LEADER_BRAKE_SCALE)
                    apply_control(leader, brake=eff_brake_l)  # Maintain braking

                if args.method == 1:
                    # baseline: Follower does not brake, will crash
                    apply_control(follower, throttle=DEFAULT_THROTTLE)
                elif args.method == 2:
                    # World state approximation: calculate gap/TTC using previous vehicle_data, trigger timing, brake after delay
                    if follower_braked:
                        apply_control(follower, brake=args.brake_follower)
                    else:
                        gap_m, ttc_sec = (float("inf"), float("inf"))
                        if vehicle_data_for_control is not None and len(vehicle_data_for_control) >= 2:
                            gap_m, ttc_sec = compute_gap_and_ttc(vehicle_data_for_control)
                        if args.sensor_debug:
                            logger.info(f"[method=2] t={t:.2f}s gap={gap_m:.2f}m TTC={ttc_sec:.2f}s danger_since={sensor_danger_since} braked={follower_braked}")
                        triggered = False
                        if args.sensor_gap_threshold > 0 and gap_m < args.sensor_gap_threshold:
                            triggered = True
                        if args.sensor_ttc_threshold > 0 and ttc_sec < args.sensor_ttc_threshold and ttc_sec < 1e6:
                            triggered = True
                        if triggered and sensor_danger_since is None:
                            sensor_danger_since = t
                            logger.info(f"t={t:.1f}s: method=2 danger triggered (gap={gap_m:.1f}m, TTC={ttc_sec:.1f}s).")
                        if sensor_danger_since is not None and t >= sensor_danger_since + args.sensor_reaction_delay:
                            follower_braked = True
                            logger.info(f"t={t:.1f}s: method=2 Follower brake applied (reaction delay {args.sensor_reaction_delay}s, brake {args.brake_follower}).")
                        if follower_braked:
                            apply_control(follower, brake=args.brake_follower)
                        else:
                            apply_control(follower, throttle=DEFAULT_THROTTLE)
                elif args.method in (3, 4):
                    # V2X: method=3 is 802.11p, method=4 is currently a stub version of 5G NR
                    # Both trigger follower braking via brake_warning returned by NS-3. For future 5G-LENA integration, 
                    # we only need to differentiate the protocol stack on the NS-3 side.
                    warnings = bridge.get_and_clear_brake_warnings()
                    if warnings:
                        if brake_warning_first_seen_t is None:
                            brake_warning_first_seen_t = t
                        follower_braked = True
                        if follower_brake_applied_t is None:
                            follower_brake_applied_t = t
                        logger.info(
                            f"t={t:.1f}s: method={args.method} Follower brake applied "
                            f"(received {len(warnings)} brake_warning(s), brake {args.brake_follower})."
                        )
                    if follower_braked:
                        apply_control(follower, brake=args.brake_follower)
                    else:
                        apply_control(follower, throttle=DEFAULT_THROTTLE)

            # Simulation step + sampling / sending (avoid 0,0,0 false frames caused by CARLA not advancing at t=0)
            # 1) advance CARLA to next tick
            world.tick()
            t += DEFAULT_DT

            # 2) collect state at sim time t, send to ns-3
            vehicle_data = collect_vehicle_data(all_vehicles)
            vehicle_data_for_control = vehicle_data  # For use in next iteration of method=2

            if args.lockstep:
                ok = bridge.send_tick(tick_seq, t, vehicle_data)
                if not ok:
                    logger.error("Lock-step: failed to send tick to ns-3.")
                    break
                ack = bridge.wait_for_tick_ack(tick_seq, timeout_sec=10.0)
                if ack is None:
                    logger.error(f"Lock-step: timeout waiting for tick_ack seq={tick_seq}.")
                    break
                tick_seq += 1
            else:
                bridge.send_vehicle_states(vehicle_data)

            # 3) update safety metrics for state at time t
            safety_eval.update(t, vehicle_data)

            # 4) optional wall-clock pacing (not required for correctness when lockstep=1)
            if (not args.lockstep) and args.sleep_per_tick_sec and args.sleep_per_tick_sec > 0:
                time.sleep(args.sleep_per_tick_sec)

    except KeyboardInterrupt:
        logger.info("Simulation interrupted by user")
    finally:
        try:
            bridge.stop()
            # Save safety metrics (in same directory as v2x_eval)
            run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            safety_path = safety_eval.write_summary(
                str(run_dir),
                run_timestamp=run_ts,
                run_label=run_tag,
                method=args.method,
                world_duration_sec=t,
                extra_params={
                    "v0_leader": args.v0_leader,
                    "v0_follower": args.v0_follower,
                    "brake_leader": args.brake_leader,
                    "brake_follower": args.brake_follower,
                    "t0": args.t0,
                    "gap": args.gap,
                    "scene": args.scene,
                    "overhead_height_m": args.overhead_height,
                    "channel_regime": args.regime,  # Step 4.5: 1=Good, 2=Medium, 3=Bad; None for M1/M2
                    # network diagnostics (to judge if caused by "environment" or "abnormal reasons")
                    "method3_brake_warning_first_seen_t": brake_warning_first_seen_t,
                    "method3_follower_brake_applied_t": follower_brake_applied_t,
                    "ns3_diag": bridge.get_network_diagnostics() if args.method in (3, 4) else None,
                }
            )
            if safety_path:
                logger.info(f"Safety eval summary written to {safety_path}")
            destroy_actors(all_vehicles)
            destroy_sensors(all_sensors)
            logger.info("Generating visualization plots...")
            visualizer = VehicleDataVisualizer(vehicle_data_logger.file_path)
            visualizer.generate_all_plots()
            logger.info("Plots saved to temp/plots/")
            # Use ffmpeg to combine overhead camera frames into a video (if ffmpeg exists)
            try:
                if video_frames_dir.exists():
                    video_dir = run_dir / "Video"
                    video_dir.mkdir(parents=True, exist_ok=True)
                    video_path = video_dir / "overhead.mp4"
                    fps = int(1.0 / DEFAULT_DT)
                    cmd = [
                        "ffmpeg",
                        "-y",
                        "-framerate",
                        str(fps),
                        "-i",
                        str(video_frames_dir / "overhead_%06d.png"),
                        "-c:v",
                        "libx264",
                        "-pix_fmt",
                        "yuv420p",
                        str(video_path),
                    ]
                    subprocess.run(
                        cmd,
                        check=True,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    logger.info(f"Overhead video saved to {video_path}")
                else:
                    logger.info("No overhead camera frames found; skip video generation.")
            except FileNotFoundError:
                logger.warning("ffmpeg not found in PATH; skip overhead video generation.")
            except subprocess.CalledProcessError as e:
                logger.warning(f"ffmpeg failed to generate overhead video: {e}")
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
    return 0


if __name__ == "__main__":
    exit(main())
