#!/usr/bin/env python3
"""
Export Town10 (or current map) as a top-down PNG for trajectory overlay.
Run with CARLA already running. Uses config/settings MAP_* for center and extent.
Output: maps/town10_map.png (or maps/<MAP_IMAGE_NAME>).
"""
import os
import sys
import queue

# Project root and CARLA egg
_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _project_root)

try:
    import carla
except ImportError:
    egg = os.path.join(_project_root, "PythonAPI", "carla", "dist")
    for f in os.listdir(egg) if os.path.isdir(egg) else []:
        if f.endswith(".egg"):
            sys.path.insert(0, os.path.join(egg, f))
            break
    import carla

from config.settings import (
    CARLA_HOST,
    CARLA_PORT,
    CARLA_TIMEOUT,
    MAP_NAME,
    MAP_IMAGE_NAME,
    MAP_X_MIN,
    MAP_X_MAX,
    MAP_Y_MIN,
    MAP_Y_MAX,
)


def main():
    maps_dir = os.path.join(_project_root, "maps")
    os.makedirs(maps_dir, exist_ok=True)
    out_path = os.path.join(maps_dir, MAP_IMAGE_NAME)

    center_x = (MAP_X_MIN + MAP_X_MAX) / 2
    center_y = (MAP_Y_MIN + MAP_Y_MAX) / 2
    # Height so FOV covers the map range (approx: height ~ max(dx,dy)/2 for 90 deg FOV)
    extent_x = MAP_X_MAX - MAP_X_MIN
    extent_y = MAP_Y_MAX - MAP_Y_MIN
    height = max(extent_x, extent_y) / 2 + 20
    height = min(max(height, 80), 200)

    print(f"Connecting to CARLA at {CARLA_HOST}:{CARLA_PORT} ...")
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(CARLA_TIMEOUT)
    world = client.get_world()

    if MAP_NAME not in world.get_map().name:
        print(f"Loading map {MAP_NAME} ...")
        world = client.load_world(MAP_NAME)

    # Synchronous mode so we can capture one frame
    settings = world.get_settings()
    was_sync = settings.synchronous_mode
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.1
    world.apply_settings(settings)

    image_queue = queue.Queue(maxsize=2)

    try:
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", "1024")
        camera_bp.set_attribute("image_size_y", "1024")
        camera_bp.set_attribute("fov", "90")

        transform = carla.Transform(
            carla.Location(x=center_x, y=center_y, z=float(height)),
            carla.Rotation(pitch=-90, yaw=0, roll=0),
        )
        camera = world.spawn_actor(camera_bp, transform)
        camera.listen(image_queue.put)

        # Tick until we get one image
        for _ in range(20):
            world.tick()
            try:
                image = image_queue.get(timeout=0.5)
                image.save_to_disk(out_path)
                print(f"Saved map image to {out_path}")
                break
            except queue.Empty:
                continue
        else:
            print("Failed to capture image after 20 ticks.", file=sys.stderr)
            sys.exit(1)
    finally:
        if camera.is_alive:
            camera.destroy()
        settings.synchronous_mode = was_sync
        world.apply_settings(settings)

    print("Done. Use this file as maps/<MAP_IMAGE_NAME> for trajectory overlay.")


if __name__ == "__main__":
    main()
