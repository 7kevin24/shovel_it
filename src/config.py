"""
Configuration file for robot control system
"""
import torch
import numpy as np

# ============= Robot Configuration =============
RESET_STATE = torch.tensor([-1.8457, 194.5898, 187.6465, -112.6758, -2.9883, 1.1527], dtype=torch.float32)

# ============= Working Modes =============
IDLE = 0
CLEAN_MODE_1 = 1
CLEAN_MODE_2 = 2
FLATTEN_MODE = 3
RESET_MODE = 4
ERROR = 5
STOP = 6

# ============= Robot Kinematics Parameters =============
JOINT_OFFSETS = np.array([
    -2,
    -163,
    -190,
    -20,
    5,
    10,
], dtype=np.float32)

# Joints that need inversion
INVERT_JOINTS = [1, 4]

# ============= XR Control Parameters =============
R_HEADSET_TO_WORLD = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
])

SCALE_FACTOR = 0.5

# ============= Voice Control Configuration =============
ACCESS_TOKEN = "**YOUR_ACCESS_TOKEN**"  # Replace with your actual COZE access token
BOT_ID = "7520930058191126537"
HEADERS = {
    "Authorization": f"Bearer {ACCESS_TOKEN}",
    "Content-Type": "application/json"
}
WS_URL = "wss://ws.coze.cn/v1/audio/transcriptions"

# Audio parameters
SAMPLE_RATE = 48000
CHUNK = 3200
CHANNELS = 1
NO_CHANGE_TIMEOUT = 1.0
FORMAT = np.int16

# ============= File paths =============
CONTROL_FILE_PATH = "./control.json"
STATUS_FILE_PATH = "./status.json"
CACHE_PATH = "./cache"
URDF_PATH = "./so100.urdf"

# ============= Default Parameters =============
DEFAULT_FPS = 30
RESET_THRESHOLD = 100
CONTROL_STEP = 5