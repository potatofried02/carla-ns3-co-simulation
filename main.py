import time
from src.carla.carla_connector import connect_to_carla, spawn_vehicle, spawn_vehicles, set_autopilot, destroy_actors, add_camera_to_vehicle, destroy_sensors
from src.bridge.carla_ns3_bridge import CarlaNs3Bridge
from src.common.logger import logger
from src.carla.vehicle_data import collect_vehicle_data
from src.common.vehicle_data_logger import vehicle_data_logger
from src.common.visualization import VehicleDataVisualizer
from config.settings import CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT, MAP_NAME
from typing import List, Optional # Python 3.8 Fixing
def main():
    logger.info("Connecting to Carla simulator")
    client, world = connect_to_carla(CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT)
    
    if not client or not world:
        logger.error("Failed to connect to Carla. Make sure the simulator is running.")
        return
    
    logger.info("Successfully connected to Carla simulator!")
    
    world = client.load_world(MAP_NAME)
    if not world:
        logger.error(f"Failed to load map {MAP_NAME}.")
        return

    logger.info(f"Map {MAP_NAME} loaded successfully!")

    all_vehicles = []
    all_sensors = []

    ego_vehicle = spawn_vehicle(world, 'coupe_2020')
    if not ego_vehicle:
        logger.error("Failed to spawn ego vehicle")
        # bridge.stop()
        return

    all_vehicles.append(ego_vehicle)
    all_sensors.append(add_camera_to_vehicle(world, ego_vehicle))
    
    all_vehicles.extend(spawn_vehicles(world, 2, ['cooper_s']))

    set_autopilot(all_vehicles, True)

    bridge = CarlaNs3Bridge()
    bridge.start()
    
    try:
        while bridge.is_simulation_running():
            vehicle_data = collect_vehicle_data(all_vehicles)
            bridge.send_vehicle_states(vehicle_data)
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("Simulation interrupted by user")
    finally:
        try:
            bridge.stop()
            destroy_actors(all_vehicles)
            destroy_sensors(all_sensors)

            logger.info("Generating visualization plots...")
            visualizer = VehicleDataVisualizer(vehicle_data_logger.file_path)
            visualizer.generate_all_plots()
            logger.info(f"Plots have been saved")
            
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
    
    logger.info("Simulation ended")

if __name__ == '__main__':
    main()
