
import time
import carla

from src.carla.carla_connector import connect_to_carla, spawn_vehicle
from src.common.logger import logger
from config.settings import CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT, MAP_NAME


def main():
    logger.info("Connecting to CARLA...")
    client, world = connect_to_carla(CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT)
    if not client or not world:
        logger.error("Failed to connect to CARLA. Make sure the simulator is running.")
        return 1

    logger.info("Connected to CARLA.")

    if MAP_NAME not in world.get_map().name:
        logger.info(f"Loading map {MAP_NAME}...")
        try:
            world = client.load_world(MAP_NAME)
        except RuntimeError as e:
            logger.error(f"Failed to load map: {e}")
            return 1

    vehicle = spawn_vehicle(world, "coupe_2020")
    if not vehicle:
        logger.error("Failed to spawn vehicle.")
        return 1

    logger.info(f"Vehicle spawned (id={vehicle.id}). No autopilot; control from Python.")

    try:
        # 短时油门让车动起来
        control = carla.VehicleControl(throttle=0.4, brake=0.0, steer=0.0)
        vehicle.apply_control(control)
        logger.info("Applied throttle 0.4 for 2.5s...")
        time.sleep(2.5)

        # 全制动
        control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
        vehicle.apply_control(control)
        logger.info("Applied brake=1.0. Waiting for vehicle to stop...")

        for _ in range(15):
            time.sleep(0.5)
            v = vehicle.get_velocity()
            speed = (v.x**2 + v.y**2 + v.z**2) ** 0.5
            logger.info(f"  Speed = {speed:.2f} m/s")
            if speed < 0.5:
                logger.info("Vehicle stopped (speed < 0.5 m/s). Task 3.1 passed.")
                break
        else:
            logger.warning("Vehicle may not have fully stopped within 7.5s; check brake application.")

    finally:
        if vehicle.is_alive:
            vehicle.destroy()
            logger.info("Vehicle destroyed.")

    logger.info("test_brake.py finished.")
    return 0


if __name__ == "__main__":
    exit(main())
