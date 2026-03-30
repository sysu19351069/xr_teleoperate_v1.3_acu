import numpy as np
import time
import argparse
import cv2
from multiprocessing import shared_memory, Value, Array, Lock
import threading
import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController, Acu_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK, Acu_ArmIK
from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller, Dex1_1_Gripper_Controller
from teleop.robot_control.robot_hand_inspire import Inspire_Controller
from teleop.robot_control.robot_hand_brainco import Brainco_Controller
from teleop.image_server.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.ipc import IPC_Server
from sshkeyboard import listen_keyboard, stop_listening
from teleop.robot_control.acu_retargeting import AcuNeedleRetargeting
from teleop.robot_control.acu_adaptive_impedance import (
    AcuAdaptiveImpedanceController,
    VelocityOutputLimits,
    SimContactEnv,
    AcuNeedleTeleopImpedanceRunner,
)

# for simulation
# NOTE: unitree_sdk2py is only required in simulation/motion scenarios.
# Import lazily to avoid import errors in environments without unitree_sdk2py.
# from unitree_sdk2py.core.channel import ChannelPublisher
# from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

def publish_reset_category(category: int, publisher):  # Scene Reset signal
    # String_ type is provided by unitree_sdk2py in sim mode.
    msg = publisher._msg_type(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

def _make_pose_dict(position_xyz, quat_xyzw):
    return {
        "position": np.asarray(position_xyz, dtype=float).reshape(3),
        "quat": np.asarray(quat_xyzw, dtype=float).reshape(4),
    }


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def _interp_linear(p0: np.ndarray, p1: np.ndarray, t: float) -> np.ndarray:
    t = float(np.clip(t, 0.0, 1.0))
    return (1.0 - t) * p0 + t * p1


def _rot_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> quaternion (xyzw)."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    t = float(np.trace(R))
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        # find the major diagonal element
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

    q = np.array([x, y, z, w], dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def _yaw_quat(delta_yaw_rad: float) -> np.ndarray:
    """z-axis yaw quaternion (xyzw)."""
    half = 0.5 * float(delta_yaw_rad)
    return np.array([0.0, 0.0, np.sin(half), np.cos(half)], dtype=float)


def _roll_quat(delta_roll_rad: float) -> np.ndarray:
    """x-axis roll quaternion (xyzw)."""
    half = 0.5 * float(delta_roll_rad)
    return np.array([np.sin(half), 0.0, 0.0, np.cos(half)], dtype=float)


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Quaternion multiply (xyzw)."""
    x1, y1, z1, w1 = np.asarray(q1, dtype=float).reshape(4)
    x2, y2, z2, w2 = np.asarray(q2, dtype=float).reshape(4)
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )


def _quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between quaternions (xyzw)."""
    t = float(np.clip(t, 0.0, 1.0))
    q0 = _quat_normalize(q0)
    q1 = _quat_normalize(q1)
    dot = float(np.dot(q0, q1))
    # handle double-cover: pick the shortest path
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        # very close: fallback to lerp
        return _quat_normalize((1.0 - t) * q0 + t * q1)

    theta_0 = float(np.arccos(dot))
    sin_theta_0 = float(np.sin(theta_0))
    theta = theta_0 * t
    sin_theta = float(np.sin(theta))

    s0 = float(np.sin(theta_0 - theta) / sin_theta_0)
    s1 = float(sin_theta / sin_theta_0)
    return _quat_normalize(s0 * q0 + s1 * q1)


# state transition
START          = False  # Enable to start robot following VR user motion  
STOP           = False  # Enable to begin system exit procedure
RECORD_TOGGLE  = False  # [Ready] ⇄ [Recording] ⟶ [AutoSave] ⟶ [Ready]         (⇄ manual) (⟶ auto)
RECORD_RUNNING = False  # True if [Recording]
RECORD_READY   = True   # True if [Ready], False if [Recording] / [AutoSave]
# task info
TASK_NAME = None
TASK_DESC = None
ITEM_ID = None
def on_press(key):
    global STOP, START, RECORD_TOGGLE
    if key == 'r':
        START = True
    elif key == 'q':
        START = False
        STOP = True
    elif key == 's' and START == True:
        RECORD_TOGGLE = True
    else:
        logger_mp.warning(f"[on_press] {key} was pressed, but no action is defined for this key.")

def on_info(info):
    """Only handle CMD_TOGGLE_RECORD's task info"""
    global TASK_NAME, TASK_DESC, ITEM_ID
    TASK_NAME   = info.get("task_name")
    TASK_DESC   = info.get("task_desc")
    ITEM_ID     = info.get("item_id")
    logger_mp.debug(f"[on_info] Updated globals: {TASK_NAME}, {TASK_DESC}, {ITEM_ID}")

def get_state() -> dict:
    """Return current heartbeat state"""
    global START, STOP, RECORD_RUNNING, RECORD_READY
    return {
        "START": START,
        "STOP": STOP,
        "RECORD_RUNNING": RECORD_RUNNING,
        "RECORD_READY": RECORD_READY,
    }

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'save data\'s frequency')

    # basic control parameters
    parser.add_argument('--xr-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device tracking source')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1', 'ACU'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex3', 'inspire1', 'brainco'], help='Select end effector controller')
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--affinity', action = 'store_true', help = 'Enable high priority and set CPU affinity')
    parser.add_argument('--ipc', action = 'store_true', help = 'Enable IPC server to handle input; otherwise enable sshkeyboard')
    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording')
    parser.add_argument('--task-dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task name for recording')
    parser.add_argument('--task-desc', type = str, default = 'e.g. pick the red cube on the table.', help = 'task goal for recording')

    # teleop_api path-following test inputs (ACU only)
    parser.add_argument('--mpc-teleop-api', action='store_true', help='Use mpc_py.teleop_api TeleopAcuMPC with path following (ACU only)')
    parser.add_argument('--mpc-test-input', action='store_true', help='Feed built-in waypoint test inputs instead of real XR right wrist pose (ACU only)')
    parser.add_argument('--mpc-test-hold', type=float, default=1.0, help='Seconds to hold each test segment end before switching')
    parser.add_argument('--mpc-test-speed', type=float, default=0.05, help='m/s along the segment for generating intermediate points')

    # gate teleop reference streaming by controller trigger
    parser.add_argument('--mpc-stream-on-trigger', action='store_true', help='Only ingest XR teleop reference when right controller trigger is pressed (ACU + controller mode)')
    parser.add_argument('--mpc-trigger-threshold', type=float, default=0.2, help='Trigger threshold in [0,10] to start ingesting teleop reference')
    parser.add_argument('--mpc-stream-toggle', action='store_true', help='Latch teleop streaming ON after the first right-trigger press (one-shot start)')

    # Acu needle manipulation (行针手法) tracking (hand mode)
    parser.add_argument('--acu-needle-teleop', action='store_true', help='Enable Acu needle manipulation retargeting (insert+twist) from right hand tracking; overrides MPC ingest/PF when enabled')
    parser.add_argument('--acu-needle-mode', type=str, default='incremental', choices=['absolute', 'incremental'], help='Needle retargeting mode (absolute vs incremental)')
    parser.add_argument('--acu-needle-insert-angle-deg', type=float, default=35.0, help='Insert gate angle threshold (deg) for dv_index vs horizontal')
    parser.add_argument('--acu-needle-twist-angle-deg', type=float, default=20.0, help='Twist gate angle threshold (deg) for horizontal motion')
    parser.add_argument('--acu-needle-circumference-mm', type=float, default=2.0, help='Needle circumference in mm for mapping lateral motion to twist angle')
    parser.add_argument('--acu-needle-insert-gain', type=float, default=1.0, help='Insert gain')
    parser.add_argument('--acu-needle-twist-gain', type=float, default=1.0, help='Twist gain')
    parser.add_argument('--acu-needle-insert-positive-down', action='store_true', help='Define insertion positive direction as -Z of TeleVuer coordinates')

    args = parser.parse_args()
    logger_mp.info(f"args: {args}")

    # ---- local helpers for needle retargeting (keep lightweight; no extra process) ----
    def _angle_to_horizontal_rad(v: np.ndarray, eps: float = 1e-9) -> float:
        vz = float(v[2])
        vxy = float(np.linalg.norm(v[:2]))
        return float(np.arctan2(abs(vz), max(vxy, eps)))

    def _signed_angle_2d(a: np.ndarray, b: np.ndarray, eps: float = 1e-9) -> float:
        a = np.asarray(a, dtype=float).reshape(2)
        b = np.asarray(b, dtype=float).reshape(2)
        na = float(np.linalg.norm(a))
        nb = float(np.linalg.norm(b))
        if na < eps or nb < eps:
            return 0.0
        au = a / na
        bu = b / nb
        cross = float(au[0] * bu[1] - au[1] * bu[0])
        dot = float(np.clip(float(au[0] * bu[0] + au[1] * bu[1]), -1.0, 1.0))
        return float(np.arctan2(cross, dot))

    # needle retargeting shared memory
    needle_right_hand_pos_array = None
    needle_lock = None
    needle_state_array = None
    needle_action_array = None
    needle_meta_array = None
    needle_retargeter = None

    # adaptive impedance (acu needle teleop)
    acu_imp = None
    acu_imp_runner = None

    # ------------------------------
    # MPC (from mpc_py/main.py) setup
    # Only enabled for ACU arm.
    # NOTE: initialization must happen after arm_ctrl is created.
    # ------------------------------
    mpc_ctl = None
    mpc_qk = None
    mpc_rough_plan = None
    mpc_target_rough = None
    mpc_step_i = 0

    # teleop_api wrapper (preferred)
    teleop_mpc_api = None

    # ------------------------------
    # ACU: run MPC path-following + arm control in a higher-rate thread
    # (ingest teleop pose stays in main loop)
    # ------------------------------
    mpc_pf_thread = None
    mpc_pf_stop = threading.Event()

    try:
        # ipc communication. client usage: see utils/ipc.py
        if args.ipc:
            ipc_server = IPC_Server(on_press=on_press, on_info=on_info, get_state=get_state)
            ipc_server.start()
        # sshkeyboard communication
        else:
            listen_keyboard_thread = threading.Thread(target=listen_keyboard, kwargs={"on_press": on_press, "until": None, "sequential": False,}, daemon=True)
            listen_keyboard_thread.start()

        # image client: img_config should be the same as the configuration in image_server.py (of Robot's development computing unit)
        if args.sim:
            img_config = {
                'fps': 30,
                'head_camera_type': 'opencv',
                'head_camera_image_shape': [480, 640],  # Head camera resolution
                'head_camera_id_numbers': [0],
                'wrist_camera_type': 'opencv',
                'wrist_camera_image_shape': [480, 640],  # Wrist camera resolution
                'wrist_camera_id_numbers': [2, 4],
            }
        else:
            img_config = {
                'fps': 30,
                'head_camera_type': 'opencv',
                'head_camera_image_shape': [480, 1280],  # Head camera resolution
                'head_camera_id_numbers': [0],
                'wrist_camera_type': 'opencv',
                'wrist_camera_image_shape': [480, 640],  # Wrist camera resolution
                'wrist_camera_id_numbers': [2, 4],
            }


        ASPECT_RATIO_THRESHOLD = 2.0 # If the aspect ratio exceeds this value, it is considered binocular
        if len(img_config['head_camera_id_numbers']) > 1 or (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
            BINOCULAR = True
        else:
            BINOCULAR = False
        if 'wrist_camera_type' in img_config:
            WRIST = True
        else:
            WRIST = False
        
        if BINOCULAR and not (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
            tv_img_shape = (img_config['head_camera_image_shape'][0], img_config['head_camera_image_shape'][1] * 2, 3)
        else:
            tv_img_shape = (img_config['head_camera_image_shape'][0], img_config['head_camera_image_shape'][1], 3)

        tv_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(tv_img_shape) * np.uint8().itemsize)
        tv_img_array = np.ndarray(tv_img_shape, dtype = np.uint8, buffer = tv_img_shm.buf)

        if WRIST and args.sim:
            wrist_img_shape = (img_config['wrist_camera_image_shape'][0], img_config['wrist_camera_image_shape'][1] * 2, 3)
            wrist_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(wrist_img_shape) * np.uint8().itemsize)
            wrist_img_array = np.ndarray(wrist_img_shape, dtype = np.uint8, buffer = wrist_img_shm.buf)
            img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name, 
                                    wrist_img_shape = wrist_img_shape, wrist_img_shm_name = wrist_img_shm.name, server_address="127.0.0.1")
        elif WRIST and not args.sim:
            wrist_img_shape = (img_config['wrist_camera_image_shape'][0], img_config['wrist_camera_image_shape'][1] * 2, 3)
            wrist_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(wrist_img_shape) * np.uint8().itemsize)
            wrist_img_array = np.ndarray(wrist_img_shape, dtype = np.uint8, buffer = wrist_img_shm.buf)
            img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name, 
                                    wrist_img_shape = wrist_img_shape, wrist_img_shm_name = wrist_img_shm.name)
        else:
            img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name)

        image_receive_thread = threading.Thread(target = img_client.receive_process, daemon = True)
        image_receive_thread.daemon = True
        image_receive_thread.start()

        is_acu = (args.arm == 'ACU')
        # television: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
        tv_wrapper = TeleVuerWrapper(binocular=BINOCULAR, use_hand_tracking=args.xr_mode == "hand", img_shape=tv_img_shape, img_shm_name=tv_img_shm.name, 
                                    return_state_data=True, return_hand_rot_data = False,
                                    # enable Acu controller teleop mapping only for Acu + controller tracking
                                    use_acu_controller_teleop=(is_acu and args.xr_mode == 'controller'),
                                    use_acu_hand_teleop=(is_acu and args.xr_mode == 'hand'),)

        # arm
        if args.arm == "G1_29":
            arm_ik = G1_29_ArmIK()
            arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "G1_23":
            arm_ik = G1_23_ArmIK()
            arm_ctrl = G1_23_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1_2":
            arm_ik = H1_2_ArmIK()
            arm_ctrl = H1_2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1":
            arm_ik = H1_ArmIK()
            arm_ctrl = H1_ArmController(simulation_mode=args.sim)
        elif args.arm == "ACU":
            arm_ik = Acu_ArmIK()
            arm_ctrl = Acu_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        
        current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
        arm_ik.reset_init_joint_pos(current_lr_arm_q)

        # ------------------------------
        # MPC init (moved here, after arm_ctrl exists)
        # ------------------------------
        if args.arm == 'ACU':
            if args.mpc_teleop_api:
                try:
                    from mpc_py.teleop_api import TeleopAcuMPC, TeleopMPCConfig

                    urdf_path = os.path.join(parent_dir, 'assets', 'acu', 'acu.urdf')
                    teleop_mpc_api = TeleopAcuMPC(
                        TeleopMPCConfig(
                            urdf_path=urdf_path,
                            ee_frame='Acu_ee',
                            dt=0.1,
                            horizon_T=1.0,
                            solver_backend='osqp',
                        )
                    )

                    _init_q = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)
                    _init_q[6] = 0.0  
                    _init_q = np.array([-0.0, 0.63453, 1.19118, -0.0, 1.29998, 3.13, -0.0, 0.0], dtype=float)
                    q0 = _init_q[:8].copy() if _init_q.size >= 8 else np.array([-0.0, 0.63453, 1.19118, -0.0, 1.29998, 3.13, -0.0, 0.0], dtype=float)

                    # rough goal only used to initialize internal mode/state; then we drive via path following.
                    teleop_mpc_api.reset_with_rough_goal(q0=q0, goal_xyz=np.array([0.250, 0.450, -0.35], dtype=float))

                    teleop_mpc_api.switch_to_accurate(v=np.array([-0.0, -0.5, 2.0], dtype=float), r=0.05)

                    logger_mp.info('[MPC] teleop_api TeleopAcuMPC initialized.')
                except Exception as e:
                    logger_mp.error(f"[MPC] Failed to init teleop_api TeleopAcuMPC, fallback. err={e}")
                    teleop_mpc_api = None

            # keep legacy init for backward compatibility
            if teleop_mpc_api is None:
                try:
                    from mpc_py.controller import AcuMPCController, MPCMode
                    from mpc_py.planner_sd import compute_sd_rough, init_rough_plan

                    try:
                        import pinocchio as pin
                    except Exception:  # pragma: no cover
                        import pin  # type: ignore

                    urdf_path = os.path.join(parent_dir, 'assets', 'acu', 'acu.urdf')
                    mpc_ctl = AcuMPCController(
                        urdf_path=urdf_path,
                        dt=0.1,
                        horizon_T=1.0,
                        ee_frame='Acu_ee',
                        solver_backend='osqp',
                    )

                    _init_q = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)
                    if _init_q.size >= 8:
                        mpc_qk = _init_q[:8].copy()
                    else:
                        mpc_qk = np.array([-0.0, 0.63453, 1.19118, -0.0, 1.29998, 0.0, -0.0, 0.0], dtype=float)

                    mpc_qk = np.array([-0.0, 0.63453, 1.19118, -0.0, 1.29998, 3.13, -0.0, 0.0], dtype=float)
                    curpos = mpc_ctl.robot.fk(mpc_qk)
                    cur_p = np.asarray(curpos.translation).reshape(3)
                    O = np.array([0.250, 0.450, -0.35], dtype=float)
                    mpc_rough_plan = init_rough_plan(P0=cur_p, O=O)
                    mpc_target_rough = pin.SE3(np.asarray(curpos.rotation), O)

                    logger_mp.info('[MPC] ACU MPC controller initialized.')
                except Exception as e:
                    logger_mp.error(f"[MPC] Failed to init ACU MPC controller, fallback to IK. err={e}")
                    mpc_ctl = None

        # end-effector
        if args.ee == "dex3":
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
            hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "dex1":
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
            gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim)
        elif args.ee == "inspire1":
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "brainco":
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        else:
            pass
        
        # affinity mode (if you dont know what it is, then you probably don't need it)
        if args.affinity:
            import psutil
            p = psutil.Process(os.getpid())
            p.cpu_affinity([0,1,2,3]) # Set CPU affinity to cores 0-3
            try:
                p.nice(-20) # Set highest priority
                logger_mp.info("Set high priority successfully.")
            except psutil.AccessDenied:
                logger_mp.warning("Failed to set high priority. Please run as root.")
                
            for child in p.children(recursive=True):
                try:
                    logger_mp.info(f"Child process {child.pid} name: {child.name()}")
                    child.cpu_affinity([5,6])
                    child.nice(-20)
                except psutil.AccessDenied:
                    pass

        # simulation mode
        if args.sim:
            try:
                from unitree_sdk2py.core.channel import ChannelPublisher
                from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

                reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
                # stash msg type for publish_reset_category
                reset_pose_publisher._msg_type = String_
                reset_pose_publisher.Init()
                from teleop.utils.sim_state_topic import start_sim_state_subscribe
                sim_state_subscriber = start_sim_state_subscribe()
            except Exception as e:
                logger_mp.error(f"unitree_sdk2py not available for --sim mode: {e}")
                args.sim = False

        # controller + motion mode
        if args.xr_mode == "controller" and args.motion:
            try:
                from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
                sport_client = LocoClient()
                sport_client.SetTimeout(0.0001)
                sport_client.Init()
            except Exception as e:
                logger_mp.error(f"unitree_sdk2py not available for --motion mode: {e}")
                args.motion = False

        # record + headless mode
        if args.record and args.headless:
            recorder = EpisodeWriter(task_dir = args.task_dir + args.task_name, task_goal = args.task_desc, frequency = args.frequency, rerun_log = False)
        elif args.record and not args.headless:
            recorder = EpisodeWriter(task_dir = args.task_dir + args.task_name, task_goal = args.task_desc, frequency = args.frequency, rerun_log = True)

        # after arm_ctrl is created and before main loop
        if args.arm == 'ACU' and args.acu_needle_teleop:
            needle_right_hand_pos_array = Array('d', 75, lock=True)
            needle_lock = Lock()
            needle_state_array = Array('d', 2, lock=False)
            needle_action_array = Array('d', 2, lock=False)
            needle_meta_array = Array('d', 3, lock=False)  # [ts, insert_vel, twist_vel]
            needle_retargeter = AcuNeedleRetargeting(
                needle_right_hand_pos_array,
                data_lock=needle_lock,
                needle_state_array_out=needle_state_array,
                needle_action_array_out=needle_action_array,
                needle_meta_array_out=needle_meta_array,
                fps=float(max(1.0, args.frequency)),
                needle_circumference_m=float(max(1e-9, args.acu_needle_circumference_mm * 1e-3)),
                insert_angle_threshold_deg=float(args.acu_needle_insert_angle_deg),
                twist_angle_threshold_deg=float(args.acu_needle_twist_angle_deg),
                insert_gain=float(args.acu_needle_insert_gain),
                twist_gain=float(args.acu_needle_twist_gain),
                mode=str(args.acu_needle_mode),
                insert_positive_down=bool(args.acu_needle_insert_positive_down),
            )

            # Init adaptive impedance controller (use simple default stable numbers; can be tuned)
            # Filter coefficients: placeholder 2nd order IIR; replace with paper-identified coefficients if available.
            acu_imp = AcuAdaptiveImpedanceController(
                kf=1.0,
                f_safe=5.0,  # N
                filter_coeffs=(2.732e-5, 5.464e-5, 2.932e-5, -0.8415, -0.0929),
                fr=5.0,  # N target during impedance mode
                dt=float(1.0 / max(args.frequency, 1e-6)),
                hysteresis=0.2,
                latch=False,
                v_limits=VelocityOutputLimits(vmax=100),
                integrator_leak=0.02,
            )
            acu_imp_runner = AcuNeedleTeleopImpedanceRunner(
                controller=acu_imp,
                # env=SimContactEnv(ke=1.4, be=0.1, xe=0.0, mode='const', noise_mode='gaussian', noise_sigma=0.05, noise_seed=0),
                env=SimContactEnv(ke=0.14, be=0.01, xe=0.0),
                arm_ctrl=arm_ctrl,
                needle_lock=needle_lock,
                needle_action_array=needle_action_array,
                needle_meta_array=needle_meta_array,
                frequency_hz=float(max(1.0, args.frequency)),
                joint_index=6,
                reference_source='retargeting',
                stop_after_s=10.0,
                plot_save_path=os.path.join(os.getcwd(), 'acu_adaptive_impedance_retargeting_summary.png'),
            )

        logger_mp.info("Please enter the start signal (enter 'r' to start the subsequent program)")
        while not START and not STOP:
            time.sleep(0.01)
        logger_mp.info("start program.")
        arm_ctrl.speed_gradual_max()

        time.sleep(3.0)
        # For incremental Acu IK: cache last right wrist pose. The first iteration only records it.
        right_wrist_pose_last = None

        # MPC teleop streaming gate state
        mpc_ref_enabled = False
        mpc_ref_prev_pressed = False
        mpc_ref_latched = False
        # Hand mode: run the "clear path + anchors + seed" init only once per session
        # to avoid repeated triggers due to pinch jitter.
        mpc_hand_init_done = False

        # ACU needle teleop: start only after a single pinch trigger in hand mode
        needle_ref_enabled = False
        needle_init_done = False
        needle_base_q = None  # cache once: first 6 DOF (and other non-needle joints) baseline
        # step_once scheme: no background loop; runner is called once per main tick

        # Anchors for ACU XR->robot mapping (only set when trigger is pressed)
        args._acu_xr_p0 = None
        args._acu_ee_p0 = None

        # teleop_api test inputs (ACU only)
        # will be overwritten by the initial EE FK orientation when the test path is initialized.
        test_quat_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        test_waypoints = None
        test_seg_i = 0
        test_seg_start_t = None
        test_seg_p0 = None

        while not STOP:
            start_time = time.time()
            now = time.time()


            if not args.headless:
                tv_resized_image = cv2.resize(tv_img_array, (tv_img_shape[1] // 2, tv_img_shape[0] // 2))
                cv2.imshow("record image", tv_resized_image)
                # opencv GUI communication
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    START = False
                    STOP = True
                    if args.sim:
                        publish_reset_category(2, reset_pose_publisher)
                elif key == ord('s'):
                    RECORD_TOGGLE = True
                elif key == ord('a'):
                    if args.sim:
                        publish_reset_category(2, reset_pose_publisher)

            if args.record and RECORD_TOGGLE:
                RECORD_TOGGLE = False
                if not RECORD_RUNNING:
                    if recorder.create_episode():
                        RECORD_RUNNING = True
                    else:
                        logger_mp.error("Failed to create episode. Recording not started.")
                else:
                    RECORD_RUNNING = False
                    recorder.save_episode()
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)

                # If recording action triggers a sim reset / scene reset, also allow re-init next time.
                # (Keeps behavior intuitive when user starts a new episode.)
                if args.arm == 'ACU' and args.xr_mode == 'hand':
                    mpc_hand_init_done = False
                    needle_ref_enabled = False  # 如果不是遥操获取，需要设置为True
                    needle_init_done = False
                    needle_base_q = None
                    if acu_imp is not None:
                        acu_imp.reset()
                    if acu_imp_runner is not None:
                        acu_imp_runner.reset()
            # If ACU needle impedance runner is active but has stopped, exit.
            try:
                if args.arm == 'ACU' and args.acu_needle_teleop and (acu_imp_runner is not None):
                    if hasattr(acu_imp_runner, '_running') and (acu_imp_runner._running is False) and needle_ref_enabled:
                        
                        # STOP = True
                        continue
            except Exception:
                pass

            # get input data
            tele_data = tv_wrapper.get_motion_state_data()
            # ACU incremental mode: first frame only records last pose and skips IK/control.
            if args.arm == 'ACU' and right_wrist_pose_last is None:
                right_wrist_pose_last = tele_data.right_arm_pose
                # still allow recording/images/etc; just skip arm IK/control this cycle
                time_elapsed = time.time() - start_time
                time.sleep(max(0.0, (1.0 / args.frequency) - time_elapsed))
                continue
            if (args.ee == "dex3" or args.ee == "inspire1" or args.ee == "brainco") and args.xr_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
            elif args.ee == "dex1" and args.xr_mode == "controller":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_trigger_value
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_trigger_value
            elif args.ee == "dex1" and args.xr_mode == "hand":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_pinch_value
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_pinch_value
            else:
                pass        
            
            # high level control
            if args.xr_mode == "controller" and args.motion:
                # quit teleoperate
                if tele_data.tele_state.right_aButton:
                    START = False
                    STOP = True
                # command robot to enter damping mode. soft emergency stop function
                if tele_data.tele_state.left_thumbstick_state and tele_data.tele_state.right_thumbstick_state:
                    sport_client.Damp()
                # control, limit velocity to within 0.3
                sport_client.Move(-tele_data.tele_state.left_thumbstick_value[1]  * 0.3,
                                  -tele_data.tele_state.left_thumbstick_value[0]  * 0.3,
                                  -tele_data.tele_state.right_thumbstick_value[0] * 0.3)

            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # ------------------------------
            # ACU needle manipulation retargeting override
            # ------------------------------
            if args.arm == 'ACU' and args.acu_needle_teleop:
                # In this mode, do NOT use teleop_api MPC ingest/PF.
                if args.xr_mode != 'hand':
                    logger_mp.warning('[acu-needle-teleop] requires --xr-mode hand; fallback to normal pipeline.')
                else:
                    # Gate needle retargeting by pinch (trigger once to start)
                    try:
                        pinch_v = float(getattr(tele_data, 'right_pinch_value', 1e9))
                    except Exception:
                        pinch_v = 1e9

                    pressed_now = (pinch_v <= float(args.mpc_trigger_threshold))
                    if pressed_now and (not needle_init_done):
                        needle_ref_enabled = True
                        needle_init_done = True
                        try:
                            needle_base_q = np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[:8].copy()
                        except Exception:
                            needle_base_q = None
                        if acu_imp is not None:
                            try:
                                acu_imp.reset(x0=float(np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[6]))
                            except Exception:
                                acu_imp.reset()
                        if acu_imp_runner is not None:
                            try:
                                # acu_imp_runner.reset(
                                #     x0=float(np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[6]),
                                #     base_q8=np.asarray(current_lr_arm_q, dtype=float).reshape(-1)[:8].copy(),
                                # )
                                acu_imp_runner.start_session(now=float(now))
                            except Exception:
                                acu_imp_runner.reset()
                        logger_mp.info('[acu-needle-teleop] right pinch pressed: needle teleop enabled.')

                    # Not enabled yet -> skip this override and fall back to normal pipeline.
                    if not needle_ref_enabled:
                        pass
                    else:
                        try:
                            # Exit if runner has stopped (e.g., trajectory finished)
                            if acu_imp_runner is not None and hasattr(acu_imp_runner, '_running') and (acu_imp_runner._running is False):
                                STOP = True
                                break

                             # feed retargeter input
                            rh = tele_data.right_hand_pos
                            if rh is None:
                                raise RuntimeError('right_hand_pos is None')
                            rh = np.asarray(rh, dtype=float).reshape(25, 3)
                            with needle_right_hand_pos_array.get_lock():
                                needle_right_hand_pos_array[:] = rh.reshape(-1)

                            if acu_imp_runner is None:
                                raise RuntimeError('acu_imp_runner is None')
                            try:
                                acu_imp_runner.step_once(now=float(now))
                            except StopIteration:
                                STOP = True
                                break

                            # time_elapsed = time.time() - start_time
                            # time.sleep(max(0.0, (1.0 / float(args.frequency)) - time_elapsed))
                            # continue
                        except Exception as e:
                            logger_mp.error(f"[acu-needle-teleop] failed, fallback to normal pipeline. err={e}")
                            break
            
            sol_q = None
            # record data
            if args.record:
                RECORD_READY = recorder.is_ready()
                # dex hand or gripper
                if args.ee == "dex3" and args.xr_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:7]
                        right_ee_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[:7]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.xr_mode == "hand":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[0]]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.xr_mode == "controller":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [-tele_data.tele_state.left_thumbstick_value[1]  * 0.3,
                                               -tele_data.tele_state.left_thumbstick_value[0]  * 0.3,
                                               -tele_data.tele_state.right_thumbstick_value[0] * 0.3]
                elif (args.ee == "inspire1" or args.ee == "brainco") and args.xr_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[:6]
                        current_body_state = []
                        current_body_action = []
                elif args.arm == 'ACU' and args.acu_needle_teleop and args.xr_mode == 'hand' and needle_right_hand_pos_array is not None:
                    with needle_right_hand_pos_array.get_lock():
                        right_ee_state = list(needle_right_hand_pos_array[:])
                        left_ee_state = []
                        left_hand_action = []
                        right_hand_action = []
                        current_body_state = []
                        current_body_action = []
                else:
                    left_ee_state = []
                    right_ee_state = []
                    left_hand_action = []
                    right_hand_action = []
                    current_body_state = []
                    current_body_action = []
                # head image
                current_tv_image = tv_img_array.copy()
                # wrist image
                if WRIST:
                    current_wrist_image = wrist_img_array.copy()
                # arm state and action
                if args.arm == 'ACU':
                    # single arm: store in right_arm by convention (keep schema stable)
                    left_arm_state = []
                    left_arm_action = []
                    right_arm_state = current_lr_arm_q.tolist()
                    right_arm_action = sol_q.tolist() if sol_q is not None else []
                else:
                    left_arm_state  = current_lr_arm_q[:7]
                    right_arm_state = current_lr_arm_q[-7:]
                    left_arm_action = sol_q[:7]
                    right_arm_action = sol_q[-7:]
                if RECORD_RUNNING:
                    colors = {}
                    depths = {}
                    if BINOCULAR:
                        colors[f"color_{0}"] = current_tv_image[:, :tv_img_shape[1]//2]
                        colors[f"color_{1}"] = current_tv_image[:, tv_img_shape[1]//2:]
                        if WRIST:
                            colors[f"color_{2}"] = current_wrist_image[:, :wrist_img_shape[1]//2]
                            colors[f"color_{3}"] = current_wrist_image[:, wrist_img_shape[1]//2:]
                    else:
                        colors[f"color_{0}"] = current_tv_image
                        if WRIST:
                            colors[f"color_{1}"] = current_wrist_image[:, :wrist_img_shape[1]//2]
                            colors[f"color_{2}"] = current_wrist_image[:, wrist_img_shape[1]//2:]
                    states = {
                        "left_arm": {                                                                    
                            "qpos":   left_arm_state if args.arm == 'ACU' else left_arm_state.tolist(),    # numpy.array -> list
                            "qvel":   [],                          
                            "torque": [],                        
                        }, 
                        "right_arm": {                                                                    
                            "qpos":   right_arm_state if args.arm == 'ACU' else right_arm_state.tolist(),       
                            "qvel":   [],                          
                            "torque": [],                         
                        },                        
                        "left_ee": {                                                                    
                            "qpos":   left_ee_state,           
                            "qvel":   [],                           
                            "torque": [],                          
                        }, 
                        "right_ee": {                                                                    
                            "qpos":   right_ee_state,       
                            "qvel":   [],                           
                            "torque": [],  
                        }, 
                        "body": {
                            "qpos": current_body_state,
                        }, 
                    }
                    actions = {
                        "left_arm": {                                   
                            "qpos":   left_arm_action if args.arm == 'ACU' else left_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],      
                        }, 
                        "right_arm": {                                   
                            "qpos":   right_arm_action if args.arm == 'ACU' else right_arm_action.tolist(),  
                            "qvel":   [],       
                            "torque": [],       
                        },                         
                        "left_ee": {                                   
                            "qpos":   left_hand_action,       
                            "qvel":   [],       
                            "torque": [],       
                        }, 
                        "right_ee": {                                   
                            "qpos":   right_hand_action,       
                            "qvel":   [],       
                            "torque": [], 
                        }, 
                        "body": {
                            "qpos": current_body_action,
                        }, 
                    }
                    if args.sim:
                        sim_state = sim_state_subscriber.read_data()            
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, sim_state=sim_state)
                    else:
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("KeyboardInterrupt, exiting program...")
    except Exception as e:
        logger_mp.error(f"main process exception: {e}")
    finally:
        try:
            if acu_imp_runner is not None:
                acu_imp_runner.stop()
                acu_imp_runner.join(timeout=1.0)
        except Exception:
            pass
        arm_ctrl.ctrl_dual_arm_go_home()

        if args.ipc:
            ipc_server.stop()
        else:
            stop_listening()
            listen_keyboard_thread.join()

        if args.sim:
            sim_state_subscriber.stop_subscribe()
        tv_img_shm.close()
        tv_img_shm.unlink()
        if WRIST:
            wrist_img_shm.close()
            wrist_img_shm.unlink()

        if args.record:
            recorder.close()

        try:
            mpc_pf_stop.set()
            if mpc_pf_thread is not None:
                mpc_pf_thread.join(timeout=1.0)
        except Exception:
            pass

        logger_mp.info("Finally, exiting program.")
        exit(0)
