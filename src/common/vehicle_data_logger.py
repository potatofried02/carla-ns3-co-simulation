import json
import os
from datetime import datetime
from typing import List, Dict

class VehicleDataLogger:
    def __init__(self):
        # 默认输出目录：项目根下的 temp/
        self.temp_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "temp")
        os.makedirs(self.temp_dir, exist_ok=True)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.file_path = os.path.join(self.temp_dir, "vehicle_data.json")
        self.data = {
            "simulation_start": self.timestamp,
            "frames": [],
        }

    def set_run_dir(self, run_dir: str) -> None:
        """设置本次仿真的输出目录（例如 temp/Simu02251801），并确保目录存在。"""
        if not run_dir:
            return
        self.temp_dir = run_dir
        os.makedirs(self.temp_dir, exist_ok=True)
        self.file_path = os.path.join(self.temp_dir, "vehicle_data.json")

    def reset(self) -> None:
        """新一次仿真前调用，清空帧数据并更新 simulation_start。"""
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.data = {
            "simulation_start": self.timestamp,
            "frames": []
        }

    def log_frame(self, vehicle_data: List[Dict]):
        """Log a single frame of vehicle data
        
        Args:
            vehicle_data: List of dictionaries containing vehicle data
        """
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if not self.data["frames"]:
            self.data["simulation_start"] = datetime.now().strftime("%Y%m%d_%H%M%S")
        frame_data = {
            "timestamp": now_str,
            "vehicles": vehicle_data
        }
        self.data["frames"].append(frame_data)
        
        with open(self.file_path, 'w') as f:
            json.dump(self.data, f, indent=2)

vehicle_data_logger = VehicleDataLogger() 