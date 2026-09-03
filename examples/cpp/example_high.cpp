#include "highcontroller.h"
#include <unistd.h>

using namespace noetix;

#define Key0 0
#define Key1 1   // 上扳机键
#define Key2 2   // 下扳机键
#define Key5 5   // 拇指上左
#define Key6 6   // 拇指上右
#define Key7 7   // 拇指下左
#define Key8 8   // 拇指下右
#define Key9 9   // 左侧上
#define Key10 10 // 左侧下
#define Key11 11 // 右侧上
#define Key12 12 //  右侧下

static joydata remote_data;
static int curmode = 0;

int main(int argc, char *argv[]) {
        char buf[256];
        bool ret = true;
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        std::string ddsxml = "file://" + path + "/config/dds.xml";
        setenv("CYCLONEDDS_URI", ddsxml.c_str(), 1);
        printf("cur path is %s\n", path.c_str());
        HighController *ctrl = HighController::Instance();
        ControlCmd action;
        int fileindex = 0;
        Command cmd;
        ctrl->init();
        char key_updown[14], key_inuse[14];

        ctrl->subscribe_robot_hardware_status([](const RobotHardwareStatus &status) {
                static int lastmode = -1;
                // 更新遥控器数据
                remote_data = status.remote_data;
                // 更新模式
                curmode = status.workmode;
                if (curmode != lastmode) {
                        lastmode = curmode;
                        printf("[DEBUG]: callback: current mode is %d\r\n",
                               curmode);
                }
        });

        while (true) {
                for (int i = 0; i < 14; i++) {
                        if (remote_data.button[i] == 0) {
                                key_updown[i] = 0;
                                key_inuse[i] = 0;
                        } else if (remote_data.button[i] == 1) {
                                key_updown[i] = 1;
                        }
                }

                cmd.x = remote_data.axes[1];
                cmd.y = remote_data.axes[2];
                cmd.yaw = remote_data.axes[0];
                action = ControlCmd::DEFAULT;

                if (key_updown[Key2] == 0 && key_updown[Key5] == 1 &&
                    (key_inuse[Key5] == 0) && (key_updown[Key0] == 0)) {
                        action = ControlCmd::STARTTEACH;
                        key_inuse[Key5] = 1;
                } else if ((key_updown[Key6] == 1) && key_updown[Key2] == 0 &&
                           (key_inuse[Key6] == 0) && (key_updown[Key0] == 0) &&
                           (key_updown[Key1] == 0)) {
                        action = ControlCmd::SWING;
                        key_inuse[Key6] = 1;
                        printf("swing \n");

                } else if ((key_updown[Key7] == 1) && key_updown[Key2] == 0 &&
                           (key_inuse[Key7] == 0) && (key_updown[Key0] == 0)) {
                        action = ControlCmd::SHAKE;
                        key_inuse[Key7] = 1;
                        printf("shake \n");
                } else if ((key_updown[Key8] == 1) && key_updown[Key2] == 0 &&
                           (key_inuse[Key8] == 0) && (key_updown[Key0] == 0) &&
                           (key_updown[Key1] == 0)) {
                        action = ControlCmd::CHEER;
                        key_inuse[Key8] = 1;
                        printf("cheer \n");
                } else if ((key_updown[Key9] == 1) && (key_inuse[Key9] == 0)) {
                        key_inuse[Key9] = 1;
                        action = ControlCmd::START;
                        printf("start \n");
                } else if (key_updown[Key10] == 1 && (key_inuse[Key10] == 0)) {
                        action = ControlCmd::SWITCH;
                        key_inuse[Key10] = 1;
                        printf("SWITCH \n");
                } else if (key_updown[Key2] == 1 && key_updown[Key5] == 1 &&
                           (key_inuse[Key5] == 0)) {
                        action = ControlCmd::WALK;
                        key_inuse[Key5] = 1;
                        printf("WALK \n");
                } else if (key_updown[Key1] == 1 && key_updown[Key6] == 1 &&
                           (key_inuse[Key6] == 0)) {
                        action = ControlCmd::SAVETEACH;
                        key_inuse[Key6] = 1;
                        fileindex++;
                        printf("SAVETEACH \n");
                } else if (key_updown[Key1] == 1 && key_updown[Key7] == 1 &&
                           (key_inuse[Key7] == 0)) {
                        action = ControlCmd::ENDTEACH;
                        key_inuse[Key7] = 1;
                        printf("ENDTEACH \n");
                } else if (key_updown[Key1] == 1 && key_updown[Key8] == 1 &&
                           (key_inuse[Key8] == 0)) {
                        action = ControlCmd::PLAYTEACH;
                        key_inuse[Key8] = 1;
                        fileindex = 1;
                        printf("PLAYTEACH \n");
                } else if (key_updown[Key2] == 1 && key_updown[Key6] == 1 &&
                           (key_inuse[Key6] == 0)) {
                        action = ControlCmd::DANCE1;
                        key_inuse[Key6] = 1;
                        printf("DANCE1 \n");
                } else if (key_updown[Key2] == 1 && key_updown[Key7] == 1 &&
                           (key_inuse[Key7] == 0)) {
                        action = ControlCmd::DANCE2;
                        key_inuse[Key7] = 1;
                        printf("DANCE2 \n");
                } else if (key_updown[Key2] == 1 && key_updown[Key8] == 1 &&
                           (key_inuse[Key8] == 0)) {
                        action = ControlCmd::DANCE3;
                        key_inuse[Key8] = 1;
                        printf("DANCE3 \n");
                } else if (key_updown[Key0] == 1 && key_updown[Key6] == 1 &&
                           (key_inuse[Key6] == 0)) {
                        action = ControlCmd::DANCE4;
                        key_inuse[Key6] = 1;
                        printf("DANCE4 \n");
                } else if (key_updown[Key0] == 1 && key_updown[Key7] == 1 &&
                           (key_inuse[Key7] == 0)) {
                        action = ControlCmd::DANCE5;
                        key_inuse[Key7] = 1;
                        printf("DANCE5\n");
                } else if (key_updown[Key0] == 1 && key_updown[Key8] == 1 &&
                           (key_inuse[Key8] == 0)) {
                        action = ControlCmd::CF;
                        key_inuse[Key8] = 1;
                        printf("cf\n");
                } else if (key_updown[Key0] == 1 && key_updown[Key5] == 1 &&
                           (key_inuse[Key5] == 0)) {
                        action = ControlCmd::TANG;
                        key_inuse[Key5] = 1;
                        printf("TANG\n");
                }
                ctrl->publish_cmd(cmd.x, cmd.y, cmd.yaw, action, fileindex);
                sleep_ms(2);
        }

        return 0;
}
