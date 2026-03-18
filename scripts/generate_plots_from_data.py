#!/usr/bin/env python3
"""
Regenerate trajectory / speed / heading plots from existing temp/vehicle_data.json.
No CARLA or NS-3 needed. Use after a simulation run to re-plot or when you only have data.
Usage: from project root, run_simulation.sh env then:
  python scripts/generate_plots_from_data.py
  python scripts/generate_plots_from_data.py [path/to/vehicle_data.json]
"""
import os
import sys

_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _project_root)

from src.common.visualization import VehicleDataVisualizer
from src.common.logger import logger

DEFAULT_DATA_FILE = os.path.join(_project_root, "temp", "vehicle_data.json")


def main():
    data_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DATA_FILE
    if not os.path.isfile(data_file):
        print(f"Data file not found: {data_file}")
        print("Run a simulation first (./run_simulation.sh) or pass a path to vehicle_data.json.")
        sys.exit(1)
    logger.info(f"Generating plots from {data_file}")
    vis = VehicleDataVisualizer(data_file)
    vis.generate_all_plots()
    out_dir = os.path.join(os.path.dirname(data_file), "plots")
    print(f"Plots saved to {out_dir}")


if __name__ == "__main__":
    main()
