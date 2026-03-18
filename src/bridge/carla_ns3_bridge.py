import json
import os
import socket
import threading
import time
import sys
from datetime import datetime
from typing import Optional, List, Any
from src.common.logger import logger
from config.settings import NS3_HOST, NS3_SEND_PORT, NS3_RECV_PORT

# Evaluation: project root and temp dir for v2x_eval_summary.json
def _get_temp_dir():
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    temp_dir = os.path.join(project_root, "temp")
    os.makedirs(temp_dir, exist_ok=True)
    return temp_dir

class CarlaNs3Bridge:
    """Bridge for communication between CARLA and ns-3 using standard sockets"""
    
    def __init__(
        self,
        ns3_host: str = NS3_HOST,
        ns3_send_port: int = NS3_SEND_PORT,
        ns3_recv_port: int = NS3_RECV_PORT,
        run_dir: Optional[str] = None,
    ):
        self.ns3_host = ns3_host
        self.ns3_send_port = ns3_send_port
        self.ns3_recv_port = ns3_recv_port
        # 若提供 run_dir，则 v2x_eval_summary.json 写到该目录；否则写到默认 temp/
        self._run_dir = run_dir
        self.socket = None
        self.receiver_socket = None
        self.connected = False
        self.running = True
        self.reconnect_thread = None
        self.receiver_thread = None
        self.received_messages = []
        # V2X evaluation stats (see 项目大纲/评估标准与数据产出.md)
        self._v2x_count = 0
        self._v2x_first_ts = None
        self._v2x_last_ts = None
        # CAM diagnostics: did Follower receive Leader CAM?
        # v2x_message uses: receiver_id=ns3 node id, sender_id=CAM vehicleId (Leader=1, Follower=2)
        self._leader_cam_to_follower_count = 0
        self._leader_cam_to_follower_first_ns3_time = None
        self._leader_cam_to_follower_last_ns3_time = None
        # brake_warning diagnostics
        self._bw_count = 0
        self._bw_first_ts = None
        self._bw_last_ts = None
        self._bw_first_ns3_time = None
        self._bw_last_ns3_time = None
        self._bw_sources = {}  # source -> count
        # method=3: brake_warning 队列，NS-3 经 CAM 判定 Leader 制动后发来
        self._pending_brake_warnings: List[Any] = []
        self._brake_warnings_lock = threading.Lock()
        # lock-step: tick_ack queue
        self._tick_ack_lock = threading.Lock()
        self._tick_ack_cv = threading.Condition(self._tick_ack_lock)
        self._tick_acks = {}  # seq -> message dict

    def _connect(self) -> bool:
        """Connect to ns-3 server"""
        if self.socket:
            self.socket.close()
            
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.ns3_host, self.ns3_send_port))
            self.connected = True
            return True
        except Exception as e:
            logger.error(f"Error connecting to NS-3 bridge: {e}")
            self.connected = False
            return False

    def _reconnect_loop(self):
        """Try to reconnect periodically"""
        while self.running and not self.connected:
            if self._connect():
                break
            time.sleep(5)

    def ensure_connection(self):
        """Ensure there's a connection to ns-3, try to reconnect if not"""
        if not self.connected and not self.reconnect_thread:
            self.reconnect_thread = threading.Thread(target=self._reconnect_loop)
            self.reconnect_thread.daemon = True
            self.reconnect_thread.start()

    def _listen_for_messages(self):
        """Listen for messages from ns-3"""
        try:
            self.receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.receiver_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.receiver_socket.bind((self.ns3_host, self.ns3_recv_port))
            self.receiver_socket.listen(1)
            logger.info(f"Listening for NS-3 feedback on port {self.ns3_recv_port}")
            
            # Set a timeout so we can check self.running periodically if no connection comes
            self.receiver_socket.settimeout(1.0)
            
            while self.running:
                try:
                    client_socket, addr = self.receiver_socket.accept()
                    logger.info(f"NS-3 connected from {addr}")
                    
                    # Connection established, read data
                    client_socket.settimeout(None) # Blocking read
                    buffer = ""
                    while self.running:
                        try:
                            data = client_socket.recv(4096)
                            if not data:
                                break
                            
                            buffer += data.decode('utf-8')
                            
                            while '\n' in buffer:
                                line, buffer = buffer.split('\n', 1)
                                line = line.strip()
                                if not line:
                                    continue
                                
                                try:
                                    message = json.loads(line)
                                    msg_type = message.get("type")
                                    
                                    if msg_type == "simulation_end":
                                        logger.info("Received simulation end signal from NS-3")
                                        self.running = False
                                        # Important: Close socket before returning to avoid hangs
                                        try:
                                            client_socket.shutdown(socket.SHUT_RDWR)
                                        except:
                                            pass
                                        client_socket.close()
                                        return 
                                        
                                    elif msg_type == "v2x_message":
                                        # Evaluation: count and timestamps for v2x_eval_summary.json
                                        self._v2x_count += 1
                                        t = time.time()
                                        if self._v2x_first_ts is None:
                                            self._v2x_first_ts = t
                                        self._v2x_last_ts = t
                                        # Diagnostics: count Leader CAM received by Follower node
                                        try:
                                            if message.get("message_content") == "CAM":
                                                receiver_id = message.get("receiver_id")
                                                sender_id = message.get("sender_id")
                                                ns3_time = message.get("timestamp")  # seconds
                                                if receiver_id == 1 and sender_id == 1:
                                                    self._leader_cam_to_follower_count += 1
                                                    if self._leader_cam_to_follower_first_ns3_time is None:
                                                        self._leader_cam_to_follower_first_ns3_time = ns3_time
                                                    self._leader_cam_to_follower_last_ns3_time = ns3_time
                                        except Exception:
                                            pass
                                        logger.info(f"[V2X RX] {message}")
                                        print(f"[V2X RX] {message}", flush=True)  # Ensure it prints to stdout

                                    elif msg_type == "brake_warning":
                                        # Diagnostics: brake_warning stats
                                        self._bw_count += 1
                                        t = time.time()
                                        if self._bw_first_ts is None:
                                            self._bw_first_ts = t
                                        self._bw_last_ts = t
                                        ns3_time = message.get("ns3_time")
                                        if ns3_time is not None:
                                            if self._bw_first_ns3_time is None:
                                                self._bw_first_ns3_time = ns3_time
                                            self._bw_last_ns3_time = ns3_time
                                        src = message.get("source", "unknown")
                                        self._bw_sources[src] = self._bw_sources.get(src, 0) + 1
                                        with self._brake_warnings_lock:
                                            self._pending_brake_warnings.append(message)
                                        logger.info(f"[V2X RX] brake_warning from vehicle {message.get('from_vehicle_id', '?')}")
                                        
                                    elif msg_type == "tick_ack":
                                        # lock-step ack from ns-3 main loop
                                        seq = message.get("seq")
                                        if seq is not None:
                                            with self._tick_ack_cv:
                                                self._tick_acks[int(seq)] = message
                                                self._tick_ack_cv.notify_all()

                                    else:
                                        logger.debug(f"Received unknown message: {message}")
                                        
                                except json.JSONDecodeError:
                                    logger.warning(f"Received invalid JSON: {line}")
                                    
                        except socket.error as e:
                            logger.error(f"Error reading from NS-3 socket: {e}")
                            break
                            
                    try:
                        client_socket.close()
                    except:
                        pass
                    logger.info("NS-3 disconnected")
                    
                except socket.timeout:
                    # Just a timeout check for the loop condition
                    continue
                except socket.error as e:
                    if self.running:
                        logger.error(f"Accept failed: {e}")
                    break
        except Exception as e:
            logger.error(f"Error in receiver thread: {e}")
        finally:
            if self.receiver_socket:
                try:
                    self.receiver_socket.close()
                except:
                    pass

    def _start_receiver(self):
        """Start threads to receive messages from ns-3"""
            
        if not self.receiver_thread:
            self.receiver_thread = threading.Thread(target=self._listen_for_messages)
            self.receiver_thread.daemon = True
            self.receiver_thread.start()

    def send_vehicle_states(self, vehicles):
        """Send vehicle states to ns-3"""
        if not self.running:
            logger.info("Simulation ended, not sending more vehicle states")
            return False
            
        if not self.connected:
            logger.warning("Not connected, attempting to reconnect...")
            self.ensure_connection()
            if not self.connected:
                logger.error("Failed to reconnect")
                return False
        
        try:
            # Perform ID Mapping: Map arbitrary Carla IDs to 0..N-1 for NS-3
            # We sort by ID to ensure deterministic mapping across frames
            if not vehicles:
                return True
                
            sorted_vehicles = sorted(vehicles, key=lambda x: x['id'])
            mapped_vehicles = []
            
            for i, v in enumerate(sorted_vehicles):
                # Create a copy to avoid modifying the original data if needed elsewhere
                v_mapped = v.copy()
                v_mapped['original_id'] = v['id']
                v_mapped['id'] = i  # Map to 0, 1, 2...
                mapped_vehicles.append(v_mapped)

            message = json.dumps(mapped_vehicles)
            self.socket.sendall((message + "\n").encode('utf-8'))
            logger.info(f"Sent {len(message)} bytes to NS-3 successfully (mapped {len(vehicles)} vehicles)")
            return True
        except Exception as e:
            logger.error(f"Error sending vehicle states: {e}")
            self.connected = False
            return False

    def send_tick(self, seq: int, carla_time: float, vehicles: List[dict]) -> bool:
        """Lock-step protocol: send one tick message to ns-3. ns-3 will respond with tick_ack(seq)."""
        if not self.running:
            return False
        if not self.connected:
            self.ensure_connection()
            if not self.connected:
                return False

        try:
            if not vehicles:
                payload = {"type": "tick", "seq": int(seq), "carla_time": float(carla_time), "vehicles": []}
            else:
                sorted_vehicles = sorted(vehicles, key=lambda x: x["id"])
                mapped_vehicles = []
                for i, v in enumerate(sorted_vehicles):
                    v_mapped = v.copy()
                    v_mapped["original_id"] = v["id"]
                    v_mapped["id"] = i
                    mapped_vehicles.append(v_mapped)
                payload = {
                    "type": "tick",
                    "seq": int(seq),
                    "carla_time": float(carla_time),
                    "vehicles": mapped_vehicles,
                }
            msg = json.dumps(payload)
            self.socket.sendall((msg + "\n").encode("utf-8"))
            return True
        except Exception as e:
            logger.error(f"Error sending tick: {e}")
            self.connected = False
            return False

    def wait_for_tick_ack(self, seq: int, timeout_sec: float = 5.0) -> Optional[dict]:
        """Wait for tick_ack with matching seq. Returns message dict or None on timeout."""
        deadline = time.time() + timeout_sec
        with self._tick_ack_cv:
            while True:
                if seq in self._tick_acks:
                    return self._tick_acks.pop(seq)
                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                self._tick_ack_cv.wait(timeout=remaining)

    def get_v2x_stats(self):
        """Return V2X reception stats for evaluation (see 评估标准与数据产出.md)."""
        duration_sec = 0.0
        rate_per_sec = 0.0
        if self._v2x_first_ts is not None and self._v2x_last_ts is not None and self._v2x_count > 0:
            duration_sec = self._v2x_last_ts - self._v2x_first_ts
            rate_per_sec = self._v2x_count / duration_sec if duration_sec > 0 else 0.0
        bw_duration_sec = 0.0
        bw_rate_per_sec = 0.0
        if self._bw_first_ts is not None and self._bw_last_ts is not None and self._bw_count > 0:
            bw_duration_sec = self._bw_last_ts - self._bw_first_ts
            bw_rate_per_sec = self._bw_count / bw_duration_sec if bw_duration_sec > 0 else 0.0
        return {
            "run_timestamp": datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S"),
            "run_label": os.environ.get("EVAL_LABEL", ""),
            "v2x_total_count": self._v2x_count,
            "v2x_first_ts": self._v2x_first_ts,
            "v2x_last_ts": self._v2x_last_ts,
            "v2x_duration_sec": duration_sec,
            "v2x_rate_per_sec": round(rate_per_sec, 4),
            "leader_cam_to_follower_count": self._leader_cam_to_follower_count,
            "leader_cam_to_follower_first_ns3_time": self._leader_cam_to_follower_first_ns3_time,
            "leader_cam_to_follower_last_ns3_time": self._leader_cam_to_follower_last_ns3_time,
            "brake_warning_total_count": self._bw_count,
            "brake_warning_first_ts": self._bw_first_ts,
            "brake_warning_last_ts": self._bw_last_ts,
            "brake_warning_duration_sec": bw_duration_sec,
            "brake_warning_rate_per_sec": round(bw_rate_per_sec, 4),
            "brake_warning_first_ns3_time": self._bw_first_ns3_time,
            "brake_warning_last_ns3_time": self._bw_last_ns3_time,
            "brake_warning_sources": self._bw_sources,
        }

    def get_network_diagnostics(self) -> dict:
        """Return lightweight diagnostics for 'environment vs abnormal' attribution."""
        return {
            "v2x_total_count": self._v2x_count,
            "leader_cam_to_follower_count": self._leader_cam_to_follower_count,
            "leader_cam_to_follower_first_ns3_time": self._leader_cam_to_follower_first_ns3_time,
            "leader_cam_to_follower_last_ns3_time": self._leader_cam_to_follower_last_ns3_time,
            "brake_warning_total_count": self._bw_count,
            "brake_warning_first_ns3_time": self._bw_first_ns3_time,
            "brake_warning_last_ns3_time": self._bw_last_ns3_time,
            "brake_warning_sources": self._bw_sources,
        }

    def _write_v2x_eval_summary(self):
        """Write V2X evaluation summary to temp/v2x_eval_summary.json."""
        try:
            summary = self.get_v2x_stats()
            # 若指定了 run_dir，则写入该目录；否则回退到默认 temp/
            temp_dir = self._run_dir or _get_temp_dir()
            path = os.path.join(temp_dir, "v2x_eval_summary.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump(summary, f, ensure_ascii=False, indent=2)
            logger.info(f"V2X eval summary written to {path} (total received: {summary['v2x_total_count']})")
        except Exception as e:
            logger.warning(f"Failed to write v2x_eval_summary.json: {e}")

    def stop(self):
        """Stop the bridge"""
        self.running = False
        self._write_v2x_eval_summary()
        if self.socket:
            self.socket.close()
        if self.receiver_socket:
            self.receiver_socket.close()
        if self.reconnect_thread:
            self.reconnect_thread.join(timeout=1.0)
        if self.receiver_thread:
            self.receiver_thread.join(timeout=1.0)

    def _start_sender(self):
        """Start the sender"""
        self._connect()

    def start(self):
        """Start the bridge"""
        self._start_receiver()
        self._start_sender()
        logger.info("Bridge started")

    def is_simulation_running(self) -> bool:
        """Check if the simulation is running"""
        return self.running

    def get_and_clear_brake_warnings(self) -> List[Any]:
        """Return and clear all pending brake_warning messages (for method=3). Thread-safe."""
        with self._brake_warnings_lock:
            out = self._pending_brake_warnings[:]
            self._pending_brake_warnings.clear()
        return out
