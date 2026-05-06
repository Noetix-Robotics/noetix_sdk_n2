#include "lowcontroller.h"
#include <chrono>
#include <stdio.h>
#include <thread>
#include <unistd.h>

namespace legged {
// Controllerbase controllerbase;
DataBuffer<std::array<MotorCmd, 18>> motor_cmd_buffer_;
DataBuffer<std::array<MotorState, 18>> motor_state_buffer_;
DataBuffer<joydata> joy_buffer_;
DataBuffer<NingImuData> imu_buffer_;

bool LowController::init() {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        printf("cur path is %s\n", path.c_str());
        RobotSetMode::SetMode cmode;
        cmode.mode(2);
        ddswrapper.publishModeData(cmode);
        ddswrapper.subscribeRobotStatus(
            [](const RobotStatus::StatusData &ddsdata) {
                    std::array<MotorState, 18> data;
                    joydata remote_data;
                    NingImuData imudata;
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
                    memcpy(remote_data.button, &ddsdata.joydata().button(),
                           sizeof(remote_data.button));
                    memcpy(remote_data.axes, &ddsdata.joydata().axes(),
                           sizeof(remote_data.axes));

                    LowController::Instance()->set_robotstatusdata(
                        data, imudata, remote_data);
            });

        send_thread_ = std::thread(&LowController::send_thread_func, this);
        sched_param ddssched{.sched_priority = 98};

        if (pthread_setschedparam(send_thread_.native_handle(), SCHED_FIFO,
                                  &ddssched) != 0) {
                printf(" failed to set threads priority\n");
        }
        return true;
}
void LowController::set_robotstatusdata(std::array<MotorState, 18> data,
                                        NingImuData imudata, joydata joy_data) {
        motor_state_buffer_.SetData(data);
        imu_buffer_.SetData(imudata);
        joy_buffer_.SetData(joy_data);
}

void LowController::send_thread_func() {
        while (1) {
                const std::shared_ptr<const std::array<MotorCmd, 18>> mc =
                    motor_cmd_buffer_.GetData();
                if (mc) {
                        RobotMotorCmd::MotorCmdArray cmdarray;
                        cmdarray.motorcmds().resize(18);
                        for (int i = 0; i < 18; i++) {
                                auto &cmd = cmdarray.motorcmds()[i];

                                cmd.pos() = mc->at(i).pos;
                                cmd.vel() = mc->at(i).vel;
                                cmd.tau() = mc->at(i).tau;
                                cmd.kp() = mc->at(i).kp;
                                cmd.kd() = mc->at(i).kd;
                                cmd.motor_id() = mc->at(i).motor_id;
                        }
                        auto now = Clock::now();
                        long long timestamp = std::chrono::duration_cast<
                                                  std::chrono::microseconds>(
                                                  now.time_since_epoch())
                                                  .count();
                        cmdarray.timestamp() = timestamp;
                        ddswrapper.publishMotorCmdData(cmdarray);
                }
                std::this_thread::sleep_for(std::chrono::microseconds(2000));
        }
}

void LowController::set_joint(std::array<MotorCmd, 18> motorcmd) {
        motor_cmd_buffer_.SetData(motorcmd);
}

const std::array<MotorState, 18> LowController::get_joint_state() {
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

joydata LowController::from_dds_get_joydata() {
        joydata jsdata;
        const std::shared_ptr<const joydata> jdata = joy_buffer_.GetData();
        if (jdata) {
                memcpy(jsdata.button, &(*jdata).button[0],
                       sizeof(jsdata.button));
                memcpy(jsdata.axes, &(*jdata).axes[0], sizeof(jsdata.axes));
        }
        return jsdata;
}

const NingImuData LowController::get_imu_data() {
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

} // namespace legged
