import json
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os
from datetime import datetime
from src.common.logger import logger
from config.settings import MAP_IMAGE_NAME, MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX

class VehicleDataVisualizer:
    def __init__(self, data_file: str):
        """Initialize the visualizer with vehicle data file
        
        Args:
            data_file: Path to the vehicle data JSON file
        """
        self.data_file = data_file
        with open(data_file, 'r') as f:
            self.data = json.load(f)
        
        self.output_dir = os.path.join(os.path.dirname(data_file), 'plots')
        os.makedirs(self.output_dir, exist_ok=True)
        
        plt.style.use('seaborn-v0_8-paper')
        plt.rcParams.update({
            'font.size': 12,
            'axes.labelsize': 12,
            'axes.titlesize': 14,
            'xtick.labelsize': 10,
            'ytick.labelsize': 10,
            'legend.fontsize': 10,
            'figure.figsize': (12, 12),
            'figure.dpi': 300,
            'savefig.dpi': 300,
            'savefig.format': 'pdf',
            'savefig.bbox': 'tight',
            'savefig.pad_inches': 0.1
        })

    def _load_map(self):
        """Load and return the map image with its bounds"""

        current_dir = os.path.dirname(os.path.abspath(__file__))
        maps_dir = os.path.join(current_dir, '..', '..', 'maps')
        map_path = os.path.join(maps_dir, MAP_IMAGE_NAME)
        
        try:
            map_img = mpimg.imread(map_path)
            return map_img
        except Exception as e:
            print(f"Error loading map: {e}")
            return None

    def plot_vehicle_trajectories(self):
        """Plot vehicle trajectories on the map (or axes with same extent if no map image)."""
        trajectories = {}
        for frame in self.data["frames"]:
            for vehicle in frame["vehicles"]:
                vid = vehicle["id"]
                pos = vehicle["position"]
                if vid not in trajectories:
                    trajectories[vid] = {"x": [], "y": []}
                trajectories[vid]["x"].append(pos["x"])
                trajectories[vid]["y"].append(pos["y"])

        fig, ax = plt.subplots(figsize=(10, 8))
        map_img = self._load_map()
        if map_img is not None:
            ax.imshow(map_img, extent=[MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX], origin='lower', alpha=0.8, aspect='equal')
            ax.set_title("Vehicle Trajectories on Map")
        else:
            ax.set_xlim(MAP_X_MIN, MAP_X_MAX)
            ax.set_ylim(MAP_Y_MIN, MAP_Y_MAX)
            ax.set_aspect('equal')
            ax.set_title("Vehicle Trajectories (no map image; add maps/town10_map.png for road layer)")
            logger.warning("No map image found; trajectories plotted on empty axes. Add maps/town10_map.png for road layer.")

        for vid, coords in trajectories.items():
            ax.plot(coords["x"], coords["y"], label=f'Vehicle {vid}', linestyle='--')
        ax.legend(loc='lower left')
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'trajectories.pdf'))
        plt.close()

    def plot_speed_over_time(self):
        fig, ax = plt.subplots()
        
        vehicle_ids = set()
        for frame in self.data['frames']:
            for vehicle in frame['vehicles']:
                vehicle_ids.add(vehicle['id'])
        
        start_time = datetime.strptime(self.data['simulation_start'], '%Y%m%d_%H%M%S')
        times = []
        for frame in self.data['frames']:
            frame_time = datetime.strptime(frame['timestamp'], '%Y-%m-%d %H:%M:%S')
            times.append((frame_time - start_time).total_seconds())
        
        for vehicle_id in vehicle_ids:
            speeds = []
            for frame in self.data['frames']:
                for vehicle in frame['vehicles']:
                    if vehicle['id'] == vehicle_id:
                        speeds.append(vehicle['speed'])
                        break
            
            ax.plot(times, speeds, label=f'Vehicle {vehicle_id}', linewidth=1.5)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)')
        ax.set_title('Vehicle Speeds Over Time')
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend()
        
        plt.savefig(os.path.join(self.output_dir, 'speeds.pdf'))
        plt.close()

    def plot_heading_over_time(self):
        """Plot vehicle headings over time"""
        fig, ax = plt.subplots()
        
        vehicle_ids = set()
        for frame in self.data['frames']:
            for vehicle in frame['vehicles']:
                vehicle_ids.add(vehicle['id'])
        
        start_time = datetime.strptime(self.data['simulation_start'], '%Y%m%d_%H%M%S')
        times = []
        for frame in self.data['frames']:
            frame_time = datetime.strptime(frame['timestamp'], '%Y-%m-%d %H:%M:%S')
            times.append((frame_time - start_time).total_seconds())
        
        for vehicle_id in vehicle_ids:
            headings = []
            for frame in self.data['frames']:
                for vehicle in frame['vehicles']:
                    if vehicle['id'] == vehicle_id:
                        headings.append(vehicle['heading'])
                        break
            
            ax.plot(times, headings, label=f'Vehicle {vehicle_id}', linewidth=1.5)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Heading (degrees)')
        ax.set_title('Vehicle Headings Over Time')
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend()
        
        plt.savefig(os.path.join(self.output_dir, 'headings.pdf'))
        plt.close()

    def generate_all_plots(self):
        """Generate all available plots"""
        self.plot_vehicle_trajectories()
        self.plot_speed_over_time()
        self.plot_heading_over_time() 