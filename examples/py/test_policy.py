import time
from typing import List
import sys
import os
import numpy as np
from enum import Enum

import yaml
import onnxruntime as ort
from dataclasses import dataclass, field
from typing import Dict, Optional
from scipy.spatial.transform import Rotation as R
import threading

sys.path.append(os.path.abspath("./build"))
from lowcontrol_py import *

DTYPE = np.float32


@dataclass
class ControlCfg:
    stiffness: Dict[str, float] = field(default_factory=dict)

    damping: Dict[str, float] = field(default_factory=dict)

    action_scale: float = 0.25

    decimation: int = 4

    user_torque_limit: float = 0.0

    user_power_limit: float = 0.0

    cycle_time: float = 0.002


@dataclass
class Proprioception:
    joint_pos: np.ndarray = field(
        default_factory=lambda: np.zeros(10, dtype=np.float32)
    )
    joint_vel: np.ndarray = field(
        default_factory=lambda: np.zeros(10, dtype=np.float32)
    )

    base_ang_vel: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float32)
    )
    base_euler_xyz: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float32)
    )

    projected_gravity: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float32)
    )


@dataclass
class Command:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    lock: threading.Lock = field(default_factory=threading.Lock)


@dataclass
class InitState:
    arm_l1_joint: float = 0.0
    arm_l2_joint: float = 0.0
    arm_l3_joint: float = 0.0
    arm_l4_joint: float = 0.0

    leg_l1_joint: float = 0.0
    leg_l2_joint: float = 0.0
    leg_l3_joint: float = 0.0
    leg_l4_joint: float = 0.0
    leg_l5_joint: float = 0.0

    arm_r1_joint: float = 0.0
    arm_r2_joint: float = 0.0
    arm_r3_joint: float = 0.0
    arm_r4_joint: float = 0.0

    leg_r1_joint: float = 0.0
    leg_r2_joint: float = 0.0
    leg_r3_joint: float = 0.0
    leg_r4_joint: float = 0.0
    leg_r5_joint: float = 0.0


@dataclass
class ObsScales:
    lin_vel: float = 1.0
    ang_vel: float = 1.0
    dof_pos: float = 1.0
    dof_vel: float = 1.0
    quat: float = 1.0
    height_measurements: float = 1.0


@dataclass
class JointState:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(18, dtype=np.float32))

    vel: np.ndarray = field(default_factory=lambda: np.zeros(18, dtype=np.float32))

    tau: np.ndarray = field(default_factory=lambda: np.zeros(18, dtype=np.float32))


class WorkMode(Enum):
    DEFAULT = 0
    LIE = 1
    STAND = 2
    USERWALK = 3


@dataclass
class RobotCfg:
    #
    # flags
    #
    encoder_nomalize: bool = False

    #
    # clip
    #
    clip_actions: float = 100.0

    clip_obs: float = 100.0

    #
    # sub-config
    #
    init_state: InitState = field(default_factory=InitState)

    obs_scales: ObsScales = field(default_factory=ObsScales)

    control_cfg: ControlCfg = field(default_factory=ControlCfg)

    #
    # thread / loop
    #
    loophz: int = 500

    cycletimeerror_threshold: float = 0.0

    thread_priority: int = 0


# NOTE: arith funtion


def get_rotation_matrix_from_zyx_euler(euler):
    return R.from_euler("ZYX", euler).as_matrix()


def quat_to_zyx(quat):
    """
    quat: [x,y,z,w]
    """
    return R.from_quat(quat).as_euler("ZYX")


def quat_to_xyz(quat):
    return R.from_quat(quat).as_euler("XYZ")


# NOTE: global var

JOINT_NAMES = [
    "arm_l1_joint",
    "arm_l2_joint",
    "arm_l3_joint",
    "arm_l4_joint",
    "leg_l1_joint",
    "leg_l2_joint",
    "leg_l3_joint",
    "leg_l4_joint",
    "leg_l5_joint",
    "arm_r1_joint",
    "arm_r2_joint",
    "arm_r3_joint",
    "arm_r4_joint",
    "leg_r1_joint",
    "leg_r2_joint",
    "leg_r3_joint",
    "leg_r4_joint",
    "leg_r5_joint",
]

#
# low controller
#

ctrl = None

#
# model
#

modelname = ""

count = 0

robotconfig = RobotCfg()

policy_session_ptr = None

policy_input_names = []

policy_output_names = []

policy_input_node_name_allocated_strings = []

policy_output_node_name_allocated_strings = []

policy_input_shapes = []

policy_output_shapes = []

#
# dof / size
#

actuated_dof_num = 0

actions_size = 0

observation_size = 0

stack_size = 0

#
# scale
#

scalez = 0.0

scalex = 0.0

scaley = 0.0

#
# actions / obs
#

last_actions = np.array([], dtype=DTYPE)

default_joint_angles = np.array([], dtype=DTYPE)

walkdefault_joint_angles = np.array([], dtype=DTYPE)

actions = np.array([], dtype=DTYPE)

policy_observations = np.array([], dtype=DTYPE)

#
# history
#

proprio_history_buffer = np.array([], dtype=DTYPE)

#
# flags
#

isfirst_rec_obs = None

isfirst_comp_act = True

#
# command
#

command = np.zeros(3, dtype=DTYPE)

#
# proprioception
#

propri = Proprioception()

#
# phase
#

phase = 0.0

#
# remote
#

remote_data = None

#
# stand
#

stand_percent = 0.0

stand_duration = 0.0

#
# lie / stand pose
#

lie_joint_state = np.array(
    [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ],
    dtype=DTYPE,
)

stand_joint_state = np.array(
    [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.1495,
        0.3215,
        -0.1720,
        0.0,
        -0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.1495,
        0.3215,
        -0.1720,
    ],
    dtype=DTYPE,
)

#
# mode
#

mode = WorkMode.DEFAULT

is_change_mode = False

startcontrol = False

initfinish = 0

#
# pose buffers
#

lie_joint_angles = np.array([], dtype=DTYPE)

stand_joint_angles = np.array([], dtype=DTYPE)

#
# onnx
#

memory_info = ort.OrtMemoryInfo(
    "Cpu",
    ort.OrtAllocatorType.ORT_ARENA_ALLOCATOR,
    0,
    ort.OrtMemType.DEFAULT,
)

onnx_env_ptr = None


# NOTE: Funtion
def init():
    global stand_duration
    global stand_percent

    global lie_joint_angles
    global stand_joint_angles

    global lie_joint_state
    global stand_joint_state

    buf = os.getcwd()

    path = str(buf)

    print(f"cur path is {path}")

    acconfig = yaml.safe_load(open(path + "/config/ning_user.yaml", "r"))

    stand_duration = 1000

    stand_percent = 0

    lie_joint_angles = np.zeros(18, dtype=DTYPE)

    stand_joint_angles = np.zeros(18, dtype=DTYPE)

    LieState = lie_joint_state

    StandState = stand_joint_state

    lie_joint_angles[:] = [
        LieState[0],
        LieState[1],
        LieState[2],
        LieState[3],
        LieState[4],
        LieState[5],
        LieState[6],
        LieState[7],
        LieState[8],
        LieState[9],
        LieState[10],
        LieState[11],
        LieState[12],
        LieState[13],
        LieState[14],
        LieState[15],
        LieState[16],
        LieState[17],
    ]

    stand_joint_angles[:] = [
        StandState[0],
        StandState[1],
        StandState[2],
        StandState[3],
        StandState[4],
        StandState[5],
        StandState[6],
        StandState[7],
        StandState[8],
        StandState[9],
        StandState[10],
        StandState[11],
        StandState[12],
        StandState[13],
        StandState[14],
        StandState[15],
        StandState[16],
        StandState[17],
    ]

    return True


def setparameter(cmd, isfirst):
    global isfirst_rec_obs
    global isfirst_comp_act
    global command

    isfirst_rec_obs = isfirst

    isfirst_comp_act = isfirst_rec_obs

    command[0] = cmd.x
    command[1] = cmd.y
    command[2] = cmd.yaw


def update_state_estimation():
    global ctrl
    global propri
    global phase
    global actuated_dof_num

    joint_pos_noarm = np.zeros(10, dtype=DTYPE)
    joint_vel_noarm = np.zeros(10, dtype=DTYPE)

    quat = np.zeros(4, dtype=DTYPE)

    angular_vel = np.zeros(3, dtype=DTYPE)
    linear_accel = np.zeros(3, dtype=DTYPE)

    joint_state = ctrl.get_joint_state()

    j = 0

    #
    # left leg: 4~8
    # right leg: 13~17
    #
    for i in range(actuated_dof_num):
        if 4 <= i <= 8:
            joint_pos_noarm[j] = joint_state[i].pos
            joint_vel_noarm[j] = joint_state[i].vel
            j += 1

        if 13 <= i <= 17:
            joint_pos_noarm[j] = joint_state[i].pos
            joint_vel_noarm[j] = joint_state[i].vel
            j += 1

    #
    # imu
    #
    imudata = ctrl.get_imu_data()

    for i in range(4):
        quat[i] = imudata.ori[i]

    for i in range(3):
        angular_vel[i] = imudata.angular_vel[i]
        linear_accel[i] = imudata.linear_acc[i]

    #
    # proprioception
    #
    propri.joint_pos = joint_pos_noarm
    propri.joint_vel = joint_vel_noarm

    propri.base_ang_vel = angular_vel

    #
    # euler
    #
    propri.base_euler_xyz = quat_to_xyz(quat)

    #
    # VERY IMPORTANT
    #
    # 不要再自己构造 phase
    # 直接用系统时间秒
    #
    phase = time.time()

    return True


def compute_actions():
    global memory_info

    global policy_observations

    global policy_input_shapes

    global policy_session_ptr

    global policy_input_names

    global policy_output_names

    global isfirst_comp_act

    global observation_size

    global actions_size

    global actions

    input_tensor = policy_observations.reshape(policy_input_shapes[0]).astype(DTYPE)

    output_values = policy_session_ptr.run(
        policy_output_names,
        {policy_input_names[0]: input_tensor},
    )

    if isfirst_comp_act:
        for i in range(policy_observations.size):
            print(policy_observations[i], end=" ")

            if ((i + 1) % observation_size) == 0:
                print()

        isfirst_comp_act = False

    output_tensor = output_values[0].reshape(-1)

    for i in range(actions_size):
        actions[i] = output_tensor[i]


def compute_observation():
    global robotconfig
    global observation_size
    global stack_size
    global isfirst_rec_obs
    global proprio_history_buffer
    global policy_observations
    global command
    global scalex, scaley
    global phase
    global propri
    global walkdefault_joint_angles
    global last_actions

    obs_scales = robotconfig.obs_scales

    # =========================
    # 1. FIXED PHASE (核心修复)
    # =========================
    dt = robotconfig.control_cfg.cycle_time  # 0.002

    # ❌ 不要 time.time()
    # ✅ 正确：积分型 phase
    phase += dt
    gait_period = 0.64
    phase_norm = (phase % gait_period) / gait_period

    cmd = np.zeros(5, dtype=DTYPE)
    cmd[0] = np.sin(2 * np.pi * phase_norm)
    cmd[1] = np.cos(2 * np.pi * phase_norm)

    cmd_x = command[0] * scalex
    cmd_y = command[1] * scaley

    # deadzone（减少 jitter）
    if abs(cmd_x) < 0.05:
        cmd_x = 0.0
    if abs(cmd_y) < 0.05:
        cmd_y = 0.0

    cmd[2] = cmd_x * obs_scales.lin_vel
    cmd[3] = cmd_y * obs_scales.lin_vel
    cmd[4] = command[2] * obs_scales.ang_vel

    # =========================
    # 2. STATE FEATURES
    # =========================
    base_ang_vel = propri.base_ang_vel * obs_scales.ang_vel

    # ⚠️ 保持训练一致（只用前2维）
    base_euler = propri.base_euler_xyz
    base_euler = np.array(
        [base_euler[0] * obs_scales.quat, base_euler[1] * obs_scales.quat], dtype=DTYPE
    )

    joint_pos = (propri.joint_pos - walkdefault_joint_angles) * obs_scales.dof_pos
    joint_vel = propri.joint_vel * obs_scales.dof_vel

    actions_buf = last_actions.copy()

    proprio_obs = np.concatenate(
        [cmd, base_ang_vel, base_euler, joint_pos, joint_vel, actions_buf]
    ).astype(DTYPE)

    # =========================
    # 3. HISTORY BUFFER (稳定版)
    # =========================
    if isfirst_rec_obs:
        for i in range(stack_size):
            s = i * observation_size
            e = s + observation_size
            proprio_history_buffer[s:e] = proprio_obs
        isfirst_rec_obs = False

    # ✔ stable shift (避免 view alias bug)
    proprio_history_buffer = np.roll(proprio_history_buffer, -observation_size)
    proprio_history_buffer[-observation_size:] = proprio_obs

    policy_observations[:] = proprio_history_buffer

    # =========================
    # 4. CLIP
    # =========================
    policy_observations[:] = np.clip(
        policy_observations, -robotconfig.clip_obs, robotconfig.clip_obs
    )


def handle_default_mode():
    global ctrl

    motorcmd = []

    for j in range(18):
        cmd = MotorCmd()

        cmd.kd = 0.1
        cmd.pos = 0.0
        cmd.kp = 0.0
        cmd.motor_id = j
        cmd.vel = 0.0
        cmd.tau = 0.0

        motorcmd.append(cmd)

    ctrl.set_joint(motorcmd)


def handle_stand_mode():
    global ctrl

    global stand_percent
    global stand_duration

    global lie_joint_angles
    global stand_joint_angles

    motorcmd = []

    if stand_percent <= 1.0:
        for j in range(18):
            cmd = MotorCmd()

            pos_des = (
                lie_joint_angles[j] * (1.0 - stand_percent)
                + stand_joint_angles[j] * stand_percent
            )

            #
            # left arm
            #
            if j < 4:
                cmd.pos = pos_des
                cmd.kp = 20.0
                cmd.kd = 0.1

            #
            # left leg
            #
            elif (j > 4) and (j < 9):
                cmd.pos = pos_des
                cmd.kp = 50.0
                cmd.kd = 1.0

            #
            # right arm
            #
            elif (j > 9) and (j < 13):
                cmd.pos = pos_des
                cmd.kp = 20.0
                cmd.kd = 0.1

            #
            # right leg
            #
            else:
                cmd.pos = pos_des
                cmd.kp = 50.0
                cmd.kd = 1.0

            cmd.motor_id = j
            cmd.vel = 0.0
            cmd.tau = 0.0

            motorcmd.append(cmd)

        ctrl.set_joint(motorcmd)

        stand_percent += 1.0 / stand_duration

        stand_percent = min(stand_percent, 1.0)


def handle_lie_mode():
    global ctrl

    global stand_percent
    global stand_duration

    global lie_joint_angles

    motorcmd = []

    ms = ctrl.get_joint_state()

    if stand_percent <= 1.0:
        for j in range(18):
            cmd = MotorCmd()

            pos_des = (
                ms[j].pos * (1.0 - stand_percent) + lie_joint_angles[j] * stand_percent
            )

            #
            # left arm
            #
            if j < 4:
                cmd.pos = pos_des
                cmd.kp = 20.0
                cmd.kd = 0.1

            #
            # left leg
            #
            elif (j > 4) and (j < 9):
                cmd.pos = pos_des
                cmd.kp = 50.0
                cmd.kd = 1.0

            #
            # right arm
            #
            elif (j > 9) and (j < 13):
                cmd.pos = pos_des
                cmd.kp = 20.0
                cmd.kd = 0.1

            #
            # right leg
            #
            else:
                cmd.pos = pos_des
                cmd.kp = 50.0
                cmd.kd = 1.0

            cmd.motor_id = j
            cmd.vel = 0.0
            cmd.tau = 0.0

            motorcmd.append(cmd)

        ctrl.set_joint(motorcmd)

        stand_percent += 1.0 / stand_duration

        stand_percent = min(stand_percent, 1.0)


def handle_walk_mode():
    global ctrl

    global count

    global robotconfig

    global actions
    global actions_size

    global last_actions

    global default_joint_angles

    global JOINT_NAMES

    global is_change_mode

    #
    # state estimation
    #
    if update_state_estimation() is False:
        return False

    #
    # compute obs & actions
    #
    if (count % robotconfig.control_cfg.decimation) == 0:
        count = 0

        compute_observation()

        compute_actions()

        #
        # limit action range
        #
        action_min = -robotconfig.clip_actions

        action_max = robotconfig.clip_actions

        actions[:] = np.clip(
            actions,
            action_min,
            action_max,
        )

    #
    # build motor commands
    #
    motorcmd = [MotorCmd() for _ in range(18)]

    #
    # leg policy outputs
    #
    for i in range(actions_size):
        #
        # map 10-dim action -> 18 joints
        #
        if i < 5:
            j = i + 4
        else:
            j = i + 8

        part_name = JOINT_NAMES[j]

        pos_des = (
            actions[i] * robotconfig.control_cfg.action_scale + default_joint_angles[j]
        )

        stiffness = robotconfig.control_cfg.stiffness[part_name]

        damping = robotconfig.control_cfg.damping[part_name]

        motorcmd[j].pos = pos_des
        motorcmd[j].kp = stiffness
        motorcmd[j].kd = damping
        motorcmd[j].motor_id = j
        motorcmd[j].vel = 0.0
        motorcmd[j].tau = 0.0

        #
        # save last action
        #
        last_actions[i] = actions[i]

    #
    # keep arm joints near default pose
    #
    ms = ctrl.get_joint_state()

    for i in range(8):
        #
        # left arm: 0~3
        # right arm: 9~12
        #
        if i < 4:
            j = i
        else:
            j = i + 5

        part_name = JOINT_NAMES[j]

        cur_pos = ms[j].pos - default_joint_angles[j]

        pos_des = 0.75 * cur_pos + 0.25 * default_joint_angles[j]

        stiffness = robotconfig.control_cfg.stiffness[part_name]

        damping = robotconfig.control_cfg.damping[part_name]

        motorcmd[j].pos = pos_des
        motorcmd[j].kp = stiffness
        motorcmd[j].kd = damping
        motorcmd[j].motor_id = j
        motorcmd[j].vel = 0.0
        motorcmd[j].tau = 0.0

    #
    # send commands
    #
    ctrl.set_joint(motorcmd)

    count += 1

    return True


def process():
    global initfinish

    global ctrl

    global remote_data

    global startcontrol

    global stand_percent

    global mode

    global is_change_mode

    #
    # static button debounce
    #
    if not hasattr(process, "keyflag"):
        process.keyflag = [0] * 14

    keyflag = process.keyflag

    if initfinish == 0:
        return

    cmd = Command()

    #
    # joystick
    #
    remote_data = ctrl.from_dds_get_joydata()

    cmd.x = remote_data.axes[1]

    cmd.y = 0.0

    cmd.yaw = remote_data.axes[0]

    #
    # button[9]
    # start / stop control
    #
    if (remote_data.button[9] == 1) and (keyflag[9] == 0):
        if not startcontrol:
            startcontrol = True

            stand_percent = 0.0

            mode = WorkMode.LIE

            keyflag[9] = 1

            print("TO LIE MODE")

        else:
            startcontrol = False

            mode = WorkMode.DEFAULT

            keyflag[9] = 1

            print("STOP CONTROL")

    elif remote_data.button[9] == 0:
        keyflag[9] = 0

    #
    # button[10] + button[2]
    # stand mode
    #
    if (
        (remote_data.button[10] == 1)
        and (remote_data.button[2] == 1)
        and (keyflag[10] == 0)
    ):
        if startcontrol is True:
            if mode != WorkMode.STAND:
                stand_percent = 0.0

                mode = WorkMode.STAND

                print("TO STAND MODE")

            elif mode == WorkMode.LIE:
                stand_percent = 0.0

                mode = WorkMode.STAND

                print("LIE2STAND")

            keyflag[10] = 1

    elif remote_data.button[10] == 0:
        keyflag[10] = 0

    #
    # button[5] + button[2]
    # enter walk mode
    #
    if (
        (remote_data.button[5] == 1)
        and (remote_data.button[2] == 1)
        and (keyflag[5] == 0)
    ):
        if mode == WorkMode.STAND:
            stand_percent = 0.0

            is_change_mode = True

            mode = WorkMode.USERWALK

            keyflag[5] = 1

            print("TO USERWALK MODE")

    elif remote_data.button[5] == 0:
        keyflag[5] = 0

    #
    # button[11]
    # walk -> stand
    # default -> lie
    #
    if (remote_data.button[11] == 1) and (keyflag[11] == 0):
        if mode == WorkMode.USERWALK:
            is_change_mode = True

            mode = WorkMode.STAND

            print("WALK2STAND")

        elif mode == WorkMode.DEFAULT:
            stand_percent = 0.0

            is_change_mode = True

            mode = WorkMode.LIE

            print("DEFAULT2LIE")

        keyflag[11] = 1

    elif remote_data.button[11] == 0:
        keyflag[11] = 0

    #
    # mode switch
    #
    if mode == WorkMode.STAND:
        handle_stand_mode()

    elif mode == WorkMode.LIE:
        handle_lie_mode()

    elif mode == WorkMode.USERWALK:
        setparameter(
            cmd,
            is_change_mode,
        )

        handle_walk_mode()

    elif mode == WorkMode.DEFAULT:
        handle_default_mode()

    else:
        print(f"Unexpected mode encountered: {mode}")


def onnx_data_init():
    global policy_session_ptr

    global policy_input_names
    global policy_output_names

    global policy_input_shapes
    global policy_output_shapes

    #
    # clear
    #
    policy_input_names.clear()
    policy_output_names.clear()

    policy_input_shapes.clear()
    policy_output_shapes.clear()

    #
    # inputs
    #
    inputs = policy_session_ptr.get_inputs()

    for i, inp in enumerate(inputs):
        policy_input_names.append(inp.name)

        policy_input_shapes.append(inp.shape)

        print(f"Input[{i}] name: {inp.name}")

        print(f"Policy Shape: {inp.shape}")

    #
    # outputs
    #
    outputs = policy_session_ptr.get_outputs()

    for i, out in enumerate(outputs):
        policy_output_names.append(out.name)

        policy_output_shapes.append(out.shape)

        print(f"Output[{i}] name: {out.name}")

        print(f"Policy Shape: {out.shape}")


def get_model_param():
    global modelname

    global robotconfig

    global actions_size
    global observation_size
    global stack_size

    global scalez
    global scalex
    global scaley

    global actions

    global actuated_dof_num

    global policy_observations

    global last_actions

    global proprio_history_buffer

    global default_joint_angles
    global walkdefault_joint_angles

    buf = os.getcwd()

    conpath = str(buf)

    acconfig = yaml.safe_load(open(conpath + "/config/ning_user.yaml", "r"))

    cfg = acconfig[modelname]

    #
    # shortcut
    #
    init_state = robotconfig.init_state

    control_cfg = robotconfig.control_cfg

    obs_scales = robotconfig.obs_scales

    #
    # init state
    #
    default_joint_angle = cfg["init_state"]["default_joint_angle"]

    for k, v in default_joint_angle.items():
        setattr(init_state, k, float(v))

    #
    # stiffness
    #
    for k, v in cfg["control"]["stiffness"].items():
        control_cfg.stiffness[k] = float(v)

    #
    # damping
    #
    for k, v in cfg["control"]["damping"].items():
        control_cfg.damping[k] = float(v)

    #
    # control
    #
    control_cfg.action_scale = float(cfg["control"]["action_scale"])

    control_cfg.decimation = int(cfg["control"]["decimation"])

    control_cfg.cycle_time = float(cfg["control"]["cycle_time"])

    #
    # clip
    #
    robotconfig.clip_obs = float(
        cfg["normalization"]["clip_scales"]["clip_observations"]
    )

    robotconfig.clip_actions = float(
        cfg["normalization"]["clip_scales"]["clip_actions"]
    )

    #
    # obs scales
    #
    obs_cfg = cfg["normalization"]["obs_scales"]

    obs_scales.lin_vel = float(obs_cfg["lin_vel"])

    obs_scales.ang_vel = float(obs_cfg["ang_vel"])

    obs_scales.dof_pos = float(obs_cfg["dof_pos"])

    obs_scales.dof_vel = float(obs_cfg["dof_vel"])

    obs_scales.height_measurements = float(obs_cfg["height_measurements"])

    obs_scales.quat = float(obs_cfg["quat"])

    #
    # sizes
    #
    actions_size = int(cfg["size"]["actions_size"])

    observation_size = int(cfg["size"]["observations_size"])

    stack_size = int(cfg["size"]["stack_size"])

    #
    # axis mapping
    #
    scalez = float(cfg["axis_mappings"]["scalez"])

    scaley = float(cfg["axis_mappings"]["scaley"])

    scalex = float(cfg["axis_mappings"]["scalex"])

    #
    # allocate buffers
    #
    actions = np.zeros(actions_size, dtype=DTYPE)

    actuated_dof_num = 18

    policy_observations = np.zeros(
        observation_size * stack_size,
        dtype=DTYPE,
    )

    last_actions = np.zeros(
        actions_size,
        dtype=DTYPE,
    )

    proprio_history_buffer = np.zeros(
        observation_size * stack_size,
        dtype=DTYPE,
    )

    #
    # default joint angles
    #
    default_joint_angles = np.array(
        [
            init_state.arm_l1_joint,
            init_state.arm_l2_joint,
            init_state.arm_l3_joint,
            init_state.arm_l4_joint,
            init_state.leg_l1_joint,
            init_state.leg_l2_joint,
            init_state.leg_l3_joint,
            init_state.leg_l4_joint,
            init_state.leg_l5_joint,
            init_state.arm_r1_joint,
            init_state.arm_r2_joint,
            init_state.arm_r3_joint,
            init_state.arm_r4_joint,
            init_state.leg_r1_joint,
            init_state.leg_r2_joint,
            init_state.leg_r3_joint,
            init_state.leg_r4_joint,
            init_state.leg_r5_joint,
        ],
        dtype=DTYPE,
    )

    walkdefault_joint_angles = np.array(
        [
            init_state.leg_l1_joint,
            init_state.leg_l2_joint,
            init_state.leg_l3_joint,
            init_state.leg_l4_joint,
            init_state.leg_l5_joint,
            init_state.leg_r1_joint,
            init_state.leg_r2_joint,
            init_state.leg_r3_joint,
            init_state.leg_r4_joint,
            init_state.leg_r5_joint,
        ],
        dtype=DTYPE,
    )

    return True


def load_model(modelpath):
    global onnx_env_ptr

    global policy_session_ptr

    global modelname

    global command

    global isfirst_comp_act

    global isfirst_rec_obs

    global count

    global initfinish

    print(f"Load Onnx model from path : {modelpath}")

    #
    # session options
    #
    sess_options = ort.SessionOptions()

    sess_options.inter_op_num_threads = 1

    #
    # create session
    #
    policy_session_ptr = ort.InferenceSession(
        modelpath,
        sess_options=sess_options,
        providers=["CPUExecutionProvider"],
    )

    if policy_session_ptr is None:
        print("load run model failed")

        return False

    #
    # reset
    #
    modelname = "user"

    command[:] = 0.0

    isfirst_comp_act = True

    isfirst_rec_obs = True

    count = 0

    #
    # init onnx io
    #
    onnx_data_init()

    #
    # load yaml params
    #
    get_model_param()

    initfinish = 1

    print("Load Onnx run model successfully !!!")

    return True


def main():
    global ctrl

    path = os.getcwd()

    ddsxml = "file://" + path + "/config/dds.xml"

    os.environ["CYCLONEDDS_URI"] = ddsxml

    print(f"cur path is {path}")

    #
    # low controller
    #
    ctrl = LowController.instance()

    ctrl.init()

    #
    # load onnx
    #
    load_model(path + "/policy/policy_user.onnx")

    #
    # init pose
    #
    init()

    #
    # loop
    #
    while True:
        start_time = time.perf_counter()

        process()

        elapsed = time.perf_counter() - start_time

        sleep_time = 0.002 - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
