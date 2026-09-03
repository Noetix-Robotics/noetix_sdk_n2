import os
from time import sleep
import sys

os.environ["CYCLONEDDS_URI"] = "file://config/dds.xml"
sys.path.append(os.path.abspath("./build"))
from highcontrol_py import *

ctrl = HighController.instance()
ctrl.init()

# 按键状态
key_updown = [0] * 14
key_inuse = [0] * 14

fileindex = 0

# 共享状态，由回调更新
remote_data = None
curmode = -1


def on_robot_hardware_status(status: RobotHardwareStatus):
    """硬件状态回调：更新遥控器数据和模式"""
    global remote_data, curmode
    remote_data = status.remote_data
    if status.workmode != curmode:
        curmode = status.workmode
        print(f"[PYTHON DEBUG]: curmode is {curmode}")


# 注册回调
ctrl.subscribe_robot_hardware_status(on_robot_hardware_status)


Key0 = 0
Key1 = 1
Key2 = 2
Key5 = 5
Key6 = 6
Key7 = 7
Key8 = 8
Key9 = 9
Key10 = 10
lasmode = curmode = 0
action = ControlCmd.DEFAULT

while True:
    if remote_data is None:
        sleep(0.01)
        continue

    axes = remote_data.axes
    buttons = remote_data.button

    # 更新按键状态
    for i in range(14):
        if buttons[i] == 0:
            key_updown[i] = 0
            key_inuse[i] = 0
        elif buttons[i] == 1:
            key_updown[i] = 1

    x = axes[1]
    y = axes[2]
    yaw = axes[0]

    action = ControlCmd.DEFAULT

    # ===== 动作逻辑 =====
    if (
        key_updown[Key2] == 0
        and key_updown[Key5] == 1
        and key_inuse[Key5] == 0
        and key_updown[Key0] == 0
    ):
        action = ControlCmd.STARTTEACH
        key_inuse[Key5] = 1

    elif (
        key_updown[Key6] == 1
        and key_updown[Key2] == 0
        and key_inuse[Key6] == 0
        and key_updown[Key0] == 0
        and key_updown[Key1] == 0
    ):
        action = ControlCmd.SWING
        key_inuse[Key6] = 1
        print("swing")

    elif (
        key_updown[Key7] == 1
        and key_updown[Key2] == 0
        and key_inuse[Key7] == 0
        and key_updown[Key0] == 0
    ):
        action = ControlCmd.SHAKE
        key_inuse[Key7] = 1
        print("shake")

    elif (
        key_updown[Key8] == 1
        and key_updown[Key2] == 0
        and key_inuse[Key8] == 0
        and key_updown[Key0] == 0
        and key_updown[Key1] == 0
    ):
        action = ControlCmd.CHEER
        key_inuse[Key8] = 1
        print("cheer")

    elif key_updown[Key9] == 1 and key_inuse[Key9] == 0:
        action = ControlCmd.START
        key_inuse[Key9] = 1
        print("start")

    elif key_updown[Key10] == 1 and key_inuse[Key10] == 0:
        action = ControlCmd.SWITCH
        key_inuse[Key10] = 1
        print("switch")

    elif key_updown[Key2] == 1 and key_updown[Key5] == 1 and key_inuse[Key5] == 0:
        action = ControlCmd.WALK
        key_inuse[Key5] = 1
        print("walk")

    elif key_updown[Key1] == 1 and key_updown[Key6] == 1 and key_inuse[Key6] == 0:
        action = ControlCmd.SAVETEACH
        key_inuse[Key6] = 1
        fileindex += 1
        print("saveteach")

    elif key_updown[Key1] == 1 and key_updown[Key7] == 1 and key_inuse[Key7] == 0:
        action = ControlCmd.ENDTEACH
        key_inuse[Key7] = 1
        print("endteach")

    elif key_updown[Key1] == 1 and key_updown[Key8] == 1 and key_inuse[Key8] == 0:
        action = ControlCmd.PLAYTEACH
        key_inuse[Key8] = 1
        fileindex = 1
        print("playteach")

    elif key_updown[Key2] == 1 and key_updown[Key6] == 1 and key_inuse[Key6] == 0:
        action = ControlCmd.DANCE1
        key_inuse[Key6] = 1
        print("dance1")

    elif key_updown[Key2] == 1 and key_updown[Key7] == 1 and key_inuse[Key7] == 0:
        action = ControlCmd.DANCE2
        key_inuse[Key7] = 1
        print("dance2")

    elif key_updown[Key2] == 1 and key_updown[Key8] == 1 and key_inuse[Key8] == 0:
        action = ControlCmd.DANCE3
        key_inuse[Key8] = 1
        print("dance3")

    elif key_updown[Key0] == 1 and key_updown[Key6] == 1 and key_inuse[Key6] == 0:
        action = ControlCmd.DANCE4
        key_inuse[Key6] = 1
        print("dance4")

    elif key_updown[Key0] == 1 and key_updown[Key7] == 1 and key_inuse[Key7] == 0:
        action = ControlCmd.DANCE5
        key_inuse[Key7] = 1
        print("dance5")

    elif key_updown[Key0] == 1 and key_updown[Key8] == 1 and key_inuse[Key8] == 0:
        action = ControlCmd.CF
        key_inuse[Key8] = 1
        print("cf")

    elif key_updown[Key0] == 1 and key_updown[Key5] == 1 and key_inuse[Key5] == 0:
        action = ControlCmd.TANG
        key_inuse[Key5] = 1
        print("tang")

    # ===== 发送控制 =====
    ctrl.publish_cmd(x, y, yaw, action, fileindex)

    # ⚠️ 关键：不能太快
    sleep(0.002)  # 等价 sleep_ms(2)
