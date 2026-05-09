#include "highcontroller.h"

using namespace org::eclipse::cyclonedds;

namespace legged {
DataBuffer<std::array<MotorState, 18>> motor_state_buffer_;
DataBuffer<joydata> joy_buffer_;
DataBuffer<NingImuData> imu_buffer_;
bool HighController::init() {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        printf("cur path is %s\n", path.c_str());

        // remotedriver.init("/dev/input/js0",115200);
        RobotSetMode::SetMode cmode;
        cmode.mode(1);
        ddswrapper.publishModeData(cmode);

        ddswrapper.subscribeRobotStatus(
            [](const RobotStatus::StatusData &ddsdata) {
                    std::array<MotorState, 18> data;
                    joydata remote_data;
                    NingImuData imudata;
                    int curmode = 30;
                    static int modeorg = 30;
                    int i = 0;
                    for (const auto &state :
                         ddsdata.motorstatearray().motorstates()) {
                            data[i].pos = state.pos();
                            data[i].vel = state.vel();
                            data[i].tau = state.tau();
                            data[i].motor_id = state.motor_id();
                            data[i].error = state.error();
                            data[i].temperature = state.temperature();

                            // std::cout<< "motorstate  id: "<<
                            // motorstate_[i].motor_id<<";pos " <<
                            // motorstate_[i].pos << ";vel " <<
                            // motorstate_[i].vel
                            //<< ";tau "<<motorstate_[i].tau<<";error "
                            //<<motorstate_[i].error<<";temperature
                            //"<<motorstate_[i].temperature<<std::endl;
                            i++;
                    }
                    for (int i = 0; i < 4; i++) {
                            imudata.ori[i] = ddsdata.imudata().ori()[i];
                    }
                    for (int i = 0; i < 3; i++) {
                            imudata.angular_vel[i] =
                                ddsdata.imudata().angular_vel()[i];
                            imudata.linear_acc[i] =
                                ddsdata.imudata().linear_acc()[i];
                    }
                    for (int i = 0; i < 9; i++) {
                            imudata.ori_cov[i] = ddsdata.imudata().ori_cov()[i];
                            imudata.angular_vel_cov[i] =
                                ddsdata.imudata().angular_vel_cov()[i];
                            imudata.linear_acc_cov[i] =
                                ddsdata.imudata().linear_acc_cov()[i];
                    }
                    curmode = ddsdata.workmode();
                    if (modeorg == 30) {
                            modeorg = curmode;
                            printf("get mode is %d\n", curmode);
                    }
                    if (modeorg != curmode) {
                            modeorg = curmode;
                            printf("get mode is %d\n", curmode);
                    }

                    memcpy(remote_data.button, &ddsdata.joydata().button(),
                           sizeof(remote_data.button));
                    memcpy(remote_data.axes, &ddsdata.joydata().axes(),
                           sizeof(remote_data.axes));
                    // for(int i = 0;i<14;i++)
                    // {

                    //   if(remote_data.button[i] == 1)
                    //      printf("button %d  press\n",i);
                    // }

                    HighController::Instance()->set_robotstatusdata(
                        data, imudata, remote_data, curmode);
            });
        return true;
}

void HighController::set_robotstatusdata(std::array<MotorState, 18> data,
                                         NingImuData imudata, joydata joy_data,
                                         int mode) {
        motor_state_buffer_.SetData(data);
        imu_buffer_.SetData(imudata);
        joy_buffer_.SetData(joy_data);
        curmode_.store(mode);
}

joydata HighController::from_dds_get_joydata() {
        joydata jsdata;
        const std::shared_ptr<const joydata> jdata = joy_buffer_.GetData();
        if (jdata) {
                memcpy(jsdata.button, &(*jdata).button[0],
                       sizeof(jsdata.button));
                memcpy(jsdata.axes, &(*jdata).axes[0], sizeof(jsdata.axes));
        }
        return jsdata;
}

int HighController::get_mode() { return curmode_.load(); }

const NingImuData HighController::get_imu_data() {
        NingImuData imudata;
        const std::shared_ptr<const NingImuData> idata = imu_buffer_.GetData();
        if (idata) {
                for (int i = 0; i < 4; i++) {
                        imudata.ori[i] = (*idata).ori[i];
                }
                for (int i = 0; i < 3; i++) {
                        imudata.angular_vel[i] = (*idata).angular_vel[i];
                        imudata.linear_acc[i] = (*idata).linear_acc[i];
                }
                for (int i = 0; i < 9; i++) {
                        imudata.ori_cov[i] = (*idata).ori_cov[i];
                        imudata.angular_vel_cov[i] =
                            (*idata).angular_vel_cov[i];
                        imudata.linear_acc_cov[i] = (*idata).linear_acc_cov[i];
                }
        }
        return imudata;
}

void HighController::publish_cmd(double ver, double hor, double yaw,
                                 ControlCmd cmd, uint16_t index) {
        RobotControlCmd::ControlCmd controlcmd;

        auto now = Clock::now();
        long long timestamp =
            std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch())
                .count();

        controlcmd.axes()[0] = yaw;
        controlcmd.axes()[1] = ver;
        controlcmd.axes()[2] = hor;
        controlcmd.action() = (int)cmd;
        controlcmd.data() = index;
        ddswrapper.publishControlCmdData(controlcmd);
}

const std::array<MotorState, 18> HighController::get_joint_state() {
        std::array<MotorState, 18> motorstate;
        const std::shared_ptr<const std::array<MotorState, 18>> ms =
            motor_state_buffer_.GetData();
        if (ms) {
                for (int i = 0; i < 18; i++) {
                        motorstate[i].pos = ms->at(i).pos;
                        motorstate[i].vel = ms->at(i).vel;
                        motorstate[i].tau = ms->at(i).tau;
                        motorstate[i].motor_id = ms->at(i).motor_id;
                        motorstate[i].error = ms->at(i).error;
                        motorstate[i].temperature = ms->at(i).temperature;
                }
        }
        return motorstate;
}

} // namespace legged
