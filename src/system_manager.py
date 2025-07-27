"""
System manager for coordinating all robot control components
"""
import threading
import time
import os
import shutil
from lerobot.common.robot_devices.control_configs import PreTrainedConfig
from lerobot.common.policies.factory import make_policy
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

from config import (
    IDLE, CLEAN_MODE_1, CLEAN_MODE_2, FLATTEN_MODE, RESET_MODE, 
    ERROR, STOP, CONTROL_FILE_PATH, STATUS_FILE_PATH, CACHE_PATH
)
from utils import read_json_file, write_json_file
from robot_controller import RobotController
from voice_control import VoiceController
from teleoperate_pico import TeleoperationController


class SystemManager:
    """Main system manager coordinating all components"""
    
    def __init__(self):
        """Initialize system manager"""
        self.global_dict = {
            'running_state': IDLE,
            'last_mtime': 0,
            'enable_voice_control': True,
            'enable_teleoperate': False,
            'stop_lock': threading.Lock(),
            'stop_flag': False
        }
        
        # Initialize components
        self.robot_controller = RobotController()
        self.voice_controller = VoiceController(command_callback=self._handle_voice_command)
        self.teleoperation_controller = None
        
        # Initialize policies
        self._init_policies()
        
        # Initialize dataset
        self._init_dataset()
        
    def _init_policies(self):
        """Initialize cleaning policies"""
        try:
            self.clean_policy_config_1 = PreTrainedConfig.from_pretrained(
                pretrained_name_or_path="outputs/train/act_test_407_dual_cam_001/checkpoints/last/pretrained_model",
                cli_overrides=[]
            )
            self.clean_policy_config_2 = PreTrainedConfig.from_pretrained(
                pretrained_name_or_path="outputs/train/act_test_407_dual_cam_002/checkpoints/last/pretrained_model",
                cli_overrides=[]
            )
        except Exception as e:
            print(f"Warning: Could not load policy configs: {e}")
            self.clean_policy_config_1 = None
            self.clean_policy_config_2 = None
            
    def _init_dataset(self):
        """Initialize dataset"""
        try:
            # Clean cache if exists
            if os.path.exists(CACHE_PATH):
                shutil.rmtree(CACHE_PATH)

            self.dataset = LeRobotDataset.create(
                repo_id="localuser/cache",
                fps=30,
                root=None,
                robot=self.robot_controller.robot,
                use_videos=True,
                image_writer_processes=0,
                image_writer_threads=4
            )
            
            # Create policies
            self.clean_policy_1 = None if self.clean_policy_config_1 is None else make_policy(
                self.clean_policy_config_1, ds_meta=self.dataset.meta
            )
            self.clean_policy_2 = None if self.clean_policy_config_2 is None else make_policy(
                self.clean_policy_config_2, ds_meta=self.dataset.meta
            )
            
        except Exception as e:
            print(f"Warning: Could not initialize dataset and policies: {e}")
            self.dataset = None
            self.clean_policy_1 = None
            self.clean_policy_2 = None

    def _handle_voice_command(self, command_dict):
        """Handle voice command from voice controller"""
        if 'task_time' in command_dict:
            # Delayed command execution
            delay_thread = threading.Thread(
                target=self._cast_delayed_command, 
                args=(command_dict,)
            )
            delay_thread.start()
        else:
            # Immediate command execution
            self._update_system_state(command_dict)

    def _cast_delayed_command(self, command_dict):
        """Execute command after delay"""
        task_time = command_dict.get("task_time", 0)
        print(f"Waiting for {task_time} seconds before executing command...")
        time.sleep(task_time)
        self._update_system_state(command_dict)

    def _update_system_state(self, command_dict):
        """Update system state from command dictionary"""
        with self.global_dict['stop_lock']:
            if 'running_state' in command_dict:
                self.global_dict['running_state'] = command_dict['running_state']
                if self.global_dict['running_state'] == STOP:
                    self.global_dict['stop_flag'] = True
                    
            if 'enable_teleoperate' in command_dict:
                self.global_dict['enable_teleoperate'] = command_dict['enable_teleoperate']
                
            if 'enable_voice_control' in command_dict:
                self.global_dict['enable_voice_control'] = command_dict['enable_voice_control']

    def _monitor_commands(self):
        """Monitor control file for commands"""
        while True:
            try:
                if not os.path.exists(CONTROL_FILE_PATH):
                    time.sleep(0.5)
                    continue
                    
                mtime = os.path.getmtime(CONTROL_FILE_PATH)
                if mtime == self.global_dict['last_mtime']:
                    time.sleep(0.5)
                    continue
                    
                self.global_dict['last_mtime'] = mtime
                print(f"[{time.time():.2f}] Monitoring control file update...")

                command_dict = read_json_file(CONTROL_FILE_PATH)
                
                # Convert string values to appropriate types
                for key, value in command_dict.items():
                    if value == "True":
                        value = True
                    elif value == "False":
                        value = False
                    if key == "running":
                        key = "running_state"
                    command_dict[key] = value

                self._update_system_state(command_dict)

            except Exception as e:
                print(f"[{time.time():.2f}] Error in monitor_commands: {e}")

            time.sleep(0.5)

    def _check_stop_condition(self):
        """Check if system should stop"""
        with self.global_dict['stop_lock']:
            return self.global_dict['stop_flag']

    def _check_voice_enabled(self):
        """Check if voice control is enabled"""
        return self.global_dict['enable_voice_control']

    def _check_teleoperate_enabled(self):
        """Check if teleoperation is enabled"""
        return self.global_dict['enable_teleoperate']

    def _write_status(self):
        """Write current status to file"""
        keys_to_keep = ["running_state", "enable_teleoperate", "enable_voice_control"]
        filtered_dict = {
            k: self.global_dict[k] for k in keys_to_keep 
            if k in self.global_dict
        }
        write_json_file(STATUS_FILE_PATH, filtered_dict)

    def run(self):
        """Main system run loop"""
        # Start monitoring threads
        monitor_thread = threading.Thread(target=self._monitor_commands, daemon=True)
        voice_thread = threading.Thread(
            target=self.voice_controller.start_voice_control,
            args=(self._check_voice_enabled,),
            daemon=True
        )
        
        monitor_thread.start()
        voice_thread.start()
        
        print("System manager started. Entering main control loop...")
        
        # Main control loop
        while True:
            try:
                with self.global_dict['stop_lock']:
                    current_state = self.global_dict['running_state']

                # Handle teleoperation mode
                if self.global_dict['enable_teleoperate']:
                    if self.teleoperation_controller is None:
                        self.teleoperation_controller = TeleoperationController(self.robot_controller)
                    
                    self.teleoperation_controller.start_teleoperation(
                        stop_callback=self._check_stop_condition,
                        enable_callback=self._check_teleoperate_enabled
                    )
                    
                # Handle different robot states
                elif current_state == IDLE:
                    print(f"[{time.time():.2f}] Robot is idle. Waiting for commands...")
                    with self.global_dict['stop_lock']:
                        self.global_dict['stop_flag'] = False
                    time.sleep(1)
                    
                elif current_state == CLEAN_MODE_1:
                    print(f"[{time.time():.2f}] Starting cleaning mode 1...")
                    if self.clean_policy_1:
                        self.robot_controller.clean(self.clean_policy_1)
                    else:
                        print("Clean policy 1 not available")
                    with self.global_dict['stop_lock']:
                        if self.global_dict['stop_flag']:
                            self.global_dict['running_state'] = IDLE
                            
                elif current_state == CLEAN_MODE_2:
                    print(f"[{time.time():.2f}] Starting cleaning mode 2...")
                    if self.clean_policy_2:
                        self.robot_controller.clean(self.clean_policy_2)
                    else:
                        print("Clean policy 2 not available")
                    with self.global_dict['stop_lock']:
                        if self.global_dict['stop_flag']:
                            self.global_dict['running_state'] = IDLE
                            
                elif current_state == FLATTEN_MODE:
                    print(f"[{time.time():.2f}] Flattening the area...")
                    self.robot_controller.flatten_area()
                    with self.global_dict['stop_lock']:
                        self.global_dict['running_state'] = IDLE
                        
                elif current_state == RESET_MODE:
                    print(f"[{time.time():.2f}] Resetting the robot...")
                    self.robot_controller.reset_robot()
                    with self.global_dict['stop_lock']:
                        self.global_dict['running_state'] = IDLE
                        
                elif current_state == ERROR:
                    print(f"[{time.time():.2f}] An error occurred. Exiting...")
                    break
                    
                elif current_state == STOP:
                    with self.global_dict['stop_lock']:
                        self.global_dict['stop_flag'] = True
                    time.sleep(0.5)
                    
                # Write status
                self._write_status()
                
            except KeyboardInterrupt:
                print("Keyboard interrupt received. Shutting down...")
                break
            except Exception as e:
                print(f"Error in main loop: {e}")
                with self.global_dict['stop_lock']:
                    self.global_dict['running_state'] = ERROR
                
        # Cleanup
        self.shutdown()

    def shutdown(self):
        """Shutdown system and cleanup resources"""
        print("Shutting down system...")
        
        # Stop components
        if hasattr(self, 'voice_controller'):
            self.voice_controller.stop()
            
        if hasattr(self, 'teleoperation_controller') and self.teleoperation_controller:
            self.teleoperation_controller.stop()
            
        if hasattr(self, 'robot_controller'):
            self.robot_controller.disconnect()
            
        print("System shutdown complete.")