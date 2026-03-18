import carla
import random
import time
import math
from pathlib import Path
from typing import Tuple, List, Optional
import shutil
import queue
import threading

from src.common.logger import logger

# Global queue for image saving to decouple network thread from disk I/O
# maxsize=0 表示无限容量，不再因为队列满而丢弃帧
_IMAGE_QUEUE = queue.Queue(maxsize=0)
_STOP_EVENT = threading.Event()

def _image_saver_worker():
    """Worker thread to save images to disk without blocking the network thread."""
    while not _STOP_EVENT.is_set():
        try:
            task = _IMAGE_QUEUE.get(timeout=0.5)
            if task is None: continue
            
            image, path = task
            try:
                image.save_to_disk(path)
            except Exception as e:
                logger.error(f"Error saving image to disk: {e}")
            finally:
                _IMAGE_QUEUE.task_done()
        except queue.Empty:
            continue
        except Exception as e:
            logger.error(f"Unexpected error in image saver worker: {e}")

# Start the worker thread
_WORKER_THREAD = threading.Thread(target=_image_saver_worker, daemon=True, name="ImageSaverWorker")
_WORKER_THREAD.start()


def connect_to_carla(host: str, port: int, timeout: float, synchronous: bool = False, 
                    fixed_delta_seconds: float = 0.05) -> Tuple[Optional[carla.Client], Optional[carla.World]]:
    """
    Connect to a running Carla simulator
    
    Args:
        host: Carla server host
        port: Carla server port
        timeout: Connection timeout in seconds
        synchronous: If True, run CARLA in synchronous mode
        fixed_delta_seconds: Fixed time step for synchronous mode
        
    Returns:
        Tuple of (client, world) if connection successful, (None, None) otherwise
    """
    try:
        client = carla.Client(host, port)
        client.set_timeout(timeout)
        world = client.get_world()
        
        if synchronous:
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = fixed_delta_seconds
            world.apply_settings(settings)
            logger.info(f"CARLA running in synchronous mode with {fixed_delta_seconds}s fixed time step")
            
        return client, world
    except Exception as e:
        logger.error(f"Error connecting to Carla: {e}")
        return None, None
        
def spawn_vehicle(world: carla.World, vehicle_type: str = None) -> Optional[carla.Vehicle]:
    """
    Spawn a vehicle in the world
    
    Args:
        world: Carla world
        vehicle_type: Type of vehicle to spawn (e.g., 'model3')
        
    Returns:
        Vehicle actor if spawned successfully, None otherwise
    """
    try:
        blueprint_library = world.get_blueprint_library()
        
        if vehicle_type:
            vehicle_blueprint = blueprint_library.filter(vehicle_type)[0]
        else:
            vehicle_blueprint = random.choice(blueprint_library.filter('vehicle'))
            
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_blueprint, spawn_point)
        return vehicle
    except Exception as e:
        logger.error(f"Error spawning vehicle: {e}")
        return None

def spawn_vehicles(world: carla.World, num_vehicles: int, vehicle_types: "list[str]" = None) -> "list[carla.Vehicle]":
    """
    Spawn a number of vehicles in the world
    
    Args:
        world: Carla world
        num_vehicles: Number of vehicles to spawn
        vehicle_types: List of vehicle types to spawn
        
    Returns:
        List of spawned vehicles
    """
    try:
        blueprint_library = world.get_blueprint_library()
        spawned_vehicles = []

        if vehicle_types:
            vehicle_blueprints = [blueprint_library.filter(vehicle_type)[0] for vehicle_type in vehicle_types]
        else:
            vehicle_blueprints = blueprint_library.filter('*vehicle*')
        
        spawn_points = world.get_map().get_spawn_points()
        
        for i in range(num_vehicles):
            try: 
                vehicle = world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
                if vehicle:
                    spawned_vehicles.append(vehicle)
            except Exception as e:
                logger.error(f"Error spawning vehicle: {e}")
                continue
                
        return spawned_vehicles
    except Exception as e:
        logger.error(f"Error spawning vehicles: {e}")
        return []


def spawn_leader_follower_pair(
    world: carla.World,
    gap_m: float = 20.0,
    vehicle_type: str = "coupe_2020",
    spawn_point_index: int = 0,
) -> Tuple[Optional[carla.Vehicle], Optional[carla.Vehicle]]:
    """
    在一条车道上生成 Leader（前车）和 Follower（后车），同向、同车道，Follower 在 Leader 后方 gap_m 米。
    使用地图 spawn 点的位置与朝向，保证两车都在道路上且行驶方向正确。

    Args:
        world: Carla world
        gap_m: 两车初始间距（米）
        vehicle_type: 车辆蓝图类型
        spawn_point_index: 使用第几个 spawn 点作为 Leader 位置（0 表示第一个）

    Returns:
        (leader, follower)，若生成失败则为 (None, None)
    """
    try:
        blueprint_library = world.get_blueprint_library()
        vehicle_blueprint = blueprint_library.filter(vehicle_type)[0]
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            logger.error("No spawn points on map.")
            return None, None

        # 可多试几个 spawn 点，直到两车都成功生成
        for idx in range(spawn_point_index, min(spawn_point_index + 30, len(spawn_points))):
            leader_transform = spawn_points[idx]
            loc = leader_transform.location
            yaw_deg = leader_transform.rotation.yaw
            yaw_rad = math.radians(yaw_deg)
            # CARLA 车头朝向：沿 spawn 的 yaw 方向。后方 = 反方向
            backward_x = -math.cos(yaw_rad) * gap_m
            backward_y = -math.sin(yaw_rad) * gap_m
            follower_loc = carla.Location(
                x=loc.x + backward_x,
                y=loc.y + backward_y,
                z=loc.z,
            )
            follower_transform = carla.Transform(
                follower_loc,
                leader_transform.rotation,
            )

            leader = None
            follower = None
            try:
                leader = world.try_spawn_actor(vehicle_blueprint, leader_transform)
                if not leader:
                    continue
                follower = world.try_spawn_actor(vehicle_blueprint, follower_transform)
                if not follower:
                    if leader.is_alive:
                        leader.destroy()
                    continue
                logger.info(
                    f"spawn_leader_follower_pair: Leader at spawn[{idx}], Follower {gap_m}m behind (same lane)."
                )
                return leader, follower
            except Exception as e:
                logger.warning(f"spawn_leader_follower_pair try idx={idx}: {e}")
                if leader and leader.is_alive:
                    leader.destroy()
                if follower and follower.is_alive:
                    follower.destroy()
                continue

        logger.error("spawn_leader_follower_pair: Could not spawn pair on any tried spawn point.")
        return None, None
    except Exception as e:
        logger.error(f"Error in spawn_leader_follower_pair: {e}")
        return None, None


# California-style fixed positions (Town10HD): X=-50 车道，行车方向为 Y 负方向。
# CARLA 约定：车头 +X 为 yaw=0；yaw=90° 为 +Y，yaw=180° 为 -X，yaw=270°(-90°) 为 -Y。
# 故 yaw=-90° 使车头朝向 -Y，与初速度 (0,-v,0) 一致。
# 使用 get_waypoint 获取路面高度 z，避免车辆悬空或穿模。
_CALIFORNIA_SCENES = {
    "california_urban": {
        "leader_xy": (-50.0, 50.0),
        "yaw_deg": -90.0,
        "description": "Urban: Leader (X=-50,Y=50), Follower = Leader_Y + gap_m, 朝向 -Y",
    },
    "california_highway": {
        "leader_xy": (-50.0, 45.0),
        "yaw_deg": -90.0,
        "description": "Highway: Leader (X=-50,Y=45), Follower = Leader_Y + gap_m, 朝向 -Y",
    },
}


def _get_road_z_at(world: carla.World, x: float, y: float) -> float:
    """
    获取地图在 (x, y) 处的路面高度 z。先尝试 get_waypoint（多试几个 z 以利投影），
    若均失败则用最近 spawn 点的 z 作为备用。
    """
    m = world.get_map()
    for z_probe in (0.0, 5.0, 10.0, 20.0, 50.0):
        wp = m.get_waypoint(carla.Location(x=x, y=y, z=z_probe))
        if wp is not None:
            return wp.transform.location.z
    # 备用：取离 (x,y) 最近的 spawn 点的 z
    spawns = m.get_spawn_points()
    if not spawns:
        return 0.5
    best_z = 0.5
    best_d2 = float("inf")
    for sp in spawns:
        loc = sp.location
        d2 = (loc.x - x) ** 2 + (loc.y - y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_z = loc.z
    logger.warning(f"get_waypoint failed at ({x},{y}), using nearest spawn z={best_z}")
    return best_z


def spawn_leader_follower_pair_fixed(
    world: carla.World,
    scene: str = "california_urban",
    vehicle_type: str = "coupe_2020",
    gap_m: float = 18.0,
) -> Tuple[Optional[carla.Vehicle], Optional[carla.Vehicle]]:
    """
    在指定场景的固定坐标生成 Leader 与 Follower，朝向固定为 Y 负方向（行车方向），避免冲出路缘。
    Follower 位置 = Leader_Y + gap_m（沿 -Y 行驶时，后方车 Y 更大）。
    """
    if scene not in _CALIFORNIA_SCENES:
        logger.error(f"Unknown scene '{scene}'. Choose from {list(_CALIFORNIA_SCENES.keys())}")
        return None, None
    cfg = _CALIFORNIA_SCENES[scene]
    lx, ly = cfg["leader_xy"]
    # Follower 在 Leader 后方（+Y 方向），间距 gap_m
    fx, fy = lx, ly + gap_m
    yaw_deg = cfg["yaw_deg"]
    try:
        lz = _get_road_z_at(world, lx, ly)
        fz = _get_road_z_at(world, fx, fy)
        # 略高于路面，避免 spawn 时与地面嵌在一起导致失败
        z_offset = 0.5
        lz += z_offset
        fz += z_offset
        rotation = carla.Rotation(pitch=0.0, yaw=yaw_deg, roll=0.0)
        leader_t = carla.Transform(carla.Location(x=lx, y=ly, z=lz), rotation)
        follower_t = carla.Transform(carla.Location(x=fx, y=fy, z=fz), rotation)

        blueprint_library = world.get_blueprint_library()
        vehicle_blueprint = blueprint_library.filter(vehicle_type)[0]
        leader = world.try_spawn_actor(vehicle_blueprint, leader_t)
        if not leader:
            logger.error(
                "Failed to spawn leader at fixed position (x=%.1f y=%.1f z=%.2f). "
                "Try another map or check coordinates.",
                lx, ly, lz,
            )
            return None, None
        follower = world.try_spawn_actor(vehicle_blueprint, follower_t)
        if not follower:
            if leader.is_alive:
                leader.destroy()
            logger.error(
                "Failed to spawn follower at fixed position (x=%.1f y=%.1f z=%.2f).",
                fx, fy, fz,
            )
            return None, None
        logger.info(f"spawn_leader_follower_pair_fixed: scene={scene} gap={gap_m}m Leader({lx},{ly}) Follower({fx},{fy})")
        return leader, follower
    except Exception as e:
        logger.error(f"spawn_leader_follower_pair_fixed({scene}): {e}")
        return None, None


def apply_control(
    vehicle: carla.Vehicle,
    throttle: float = 0.0,
    brake: float = 0.0,
    steer: float = 0.0,
) -> None:
    """对单车施加油门/制动/转向（与 test_brake 一致，便于主循环使用）。"""
    if not vehicle.is_alive:
        return
    try:
        control = carla.VehicleControl(
            throttle=float(throttle),
            brake=float(brake),
            steer=float(steer),
        )
        vehicle.apply_control(control)
    except Exception as e:
        logger.warning(f"apply_control vehicle {vehicle.id}: {e}")


def set_autopilot(vehicles: "list[carla.Vehicle]", enable: bool = True) -> None:
    """
    Set autopilot mode for all vehicles in the world
    
    Args:
        vehicles: List of vehicles to set autopilot for
        enable: If True, enable autopilot; if False, disable
    """
    try:
        for vehicle in vehicles:
            if vehicle.is_alive:
                vehicle.set_autopilot(enable)
                logger.info(f"Set autopilot for vehicle {vehicle.id} to {enable}")
            else:
                logger.warning(f"Vehicle {vehicle.id} is not alive, skipping")
    except Exception as e:
        logger.error(f"Error setting autopilot: {e}")
        return None

def destroy_actors(vehicles_to_destroy: "list[carla.Vehicle]" = None) -> None:
    """
    Destroy specified vehicles and reset spectator camera
    
    Args:
        vehicles_to_destroy: List of vehicles to destroy. If None, destroys all vehicles.
    """
    try:
        if vehicles_to_destroy is None:
            return
        
        for vehicle in vehicles_to_destroy:
            try:
                if vehicle.is_alive:
                    vehicle.set_autopilot(False)
            except Exception as e:
                logger.warning(f"Could not disable autopilot for vehicle {vehicle.id}: {e}")
        
        for vehicle in vehicles_to_destroy:
            try:
                if vehicle.is_alive:
                    vehicle.destroy()
                    logger.info(f"Destroyed vehicle {vehicle.id}")
            except Exception as e:
                logger.warning(f"Could not destroy vehicle {vehicle.id}: {e}")
                
        logger.info("All specified vehicles destroyed successfully")
    except Exception as e:
        logger.error(f"Error during vehicle destruction: {e}")
        return None

def follow_vehicle(world: carla.World, vehicle: carla.Vehicle, stop_event=None) -> None:
    """
    Set spectator to follow a vehicle
    
    Args:
        world: Carla world object
        vehicle: Carla vehicle object
        stop_event: Threading event to signal when to stop following
    """
    spectator = world.get_spectator()
    
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-4, z=4), carla.Rotation(pitch=-25))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    spectator.set_transform(camera.get_transform())

    try:
        while True:
            try:
                if stop_event and stop_event.is_set():
                    break
                    
                spectator.set_transform(camera.get_transform())
                time.sleep(0.01)
            except KeyboardInterrupt:
                logger.info("Camera follow interrupted by user")
                break

    except Exception as e:
        logger.error(f"Error setting spectator transform: {e}")

def add_camera_to_vehicle(world: carla.World, vehicle: carla.Vehicle) -> Optional[carla.Sensor]:
    """Attach an RGB camera to *vehicle*.

    Args:
        world (carla.World): Carla world object.
        vehicle (carla.Vehicle): Vehicle to attach the camera to.

    Returns:
        carla.Sensor: The camera sensor actor.
    """

    try:
        project_root = Path(__file__).resolve().parents[2]
        temp_dir = project_root / "temp"
        camera_dir = temp_dir / "camera"

        if camera_dir.exists():
            shutil.rmtree(camera_dir)

        camera_dir.mkdir(parents=True, exist_ok=True)

        camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
        # Set sensor tick to 0.1s (10 FPS) to reduce rendering load on server
        camera_bp.set_attribute("sensor_tick", "0.1")

        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))

        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        logger.info(f"Camera attached to vehicle {vehicle.id}")

        def _save_camera(image: carla.Image) -> None:
            if _STOP_EVENT.is_set():
                return
            try:
                # Use queue to decouple network thread from disk I/O
                path = str(camera_dir / f"camera_{image.frame}.png")
                _IMAGE_QUEUE.put_nowait((image, path))
            except queue.Full:
                # logger.warning(f"Image queue full ({_IMAGE_QUEUE.qsize()} items), dropping frame {image.frame}")
                pass
            except Exception as e:
                logger.error(f"Error queuing image for save: {e}")

        camera.listen(_save_camera)
        logger.info("Camera attached. Image saving enabled (sensor tick 10 FPS, using async queue).")

        return camera

    except Exception as exc:
        logger.error(f"Error adding camera to vehicle: {exc}")
        return None

def destroy_sensors(sensors: "list[carla.Sensor]") -> None:
    """
    Destroy all sensors in the list
    
    Args:
        sensors: List of sensors to destroy
    """
    try:
        for sensor in sensors:
            if sensor.is_alive:
                sensor.destroy()
                logger.info(f"Destroyed sensor {sensor.id}")
            else:
                logger.warning(f"Sensor {sensor.id} is not alive, skipping")
                
        logger.info("All sensors destroyed successfully")
    except Exception as e:
        logger.error(f"Error during sensor destruction: {e}")
        return None


def add_overhead_camera_to_vehicle(
    world: carla.World,
    vehicle: carla.Vehicle,
    output_dir: Optional[Path] = None,
    height: float = 30.0,
    sensor_tick: float = 0.1,
) -> Optional[carla.Sensor]:
    """Attach a top-down RGB camera above *vehicle* and save frames to output_dir."""
    try:
        if output_dir is None:
            project_root = Path(__file__).resolve().parents[2]
            temp_dir = project_root / "temp"
            camera_dir = temp_dir / "camera_overhead"
        else:
            camera_dir = Path(output_dir)

        if camera_dir.exists():
            shutil.rmtree(camera_dir)
        camera_dir.mkdir(parents=True, exist_ok=True)

        camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
        camera_bp.set_attribute("sensor_tick", str(sensor_tick))

        camera_transform = carla.Transform(
            carla.Location(x=0.0, y=0.0, z=height),
            carla.Rotation(pitch=-90.0),
        )
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        logger.info(f"Overhead camera attached to vehicle {vehicle.id} at height {height}m")

        counter = {"i": 0}

        def _save_overhead(image: carla.Image) -> None:
            if _STOP_EVENT.is_set():
                return
            try:
                idx = counter["i"]
                counter["i"] += 1
                path = str(camera_dir / f"overhead_{idx:06d}.png")
                # 俯视相机直接同步落盘，避免队列丢帧导致编号断档
                image.save_to_disk(path)
            except Exception as e:
                logger.error(f"Error saving overhead image to disk: {e}")

        camera.listen(_save_overhead)
        logger.info(
            f"Overhead camera image saving enabled (sensor tick {sensor_tick}s, direct to disk)."
        )
        return camera

    except Exception as exc:
        logger.error(f"Error adding overhead camera to vehicle: {exc}")
        return None