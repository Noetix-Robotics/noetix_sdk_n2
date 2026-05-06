#include "lowcontroller.h"
#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <common.h>
#include <iostream>
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <stdio.h>
#include <thread>
#include "RotationTools.h"
#include <unistd.h>

using namespace legged;

std::vector<std::string> jointNames{
    "arm_l1_joint", "arm_l2_joint", "arm_l3_joint", "arm_l4_joint",
    "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint",
    "leg_l5_joint", "arm_r1_joint", "arm_r2_joint", "arm_r3_joint",
    "arm_r4_joint", "leg_r1_joint", "leg_r2_joint", "leg_r3_joint",
    "leg_r4_joint", "leg_r5_joint"

};

LowController *ctrl;

std::string modelname;
int64_t count;
RobotCfg robotconfig;
std::unique_ptr<Ort::Session> policySessionPtr;
std::vector<const char *> policyInputNames_;
std::vector<const char *> policyOutputNames_;
std::vector<Ort::AllocatedStringPtr> policyInputNodeNameAllocatedStrings;
std::vector<Ort::AllocatedStringPtr> policyOutputNodeNameAllocatedStrings;
std::vector<std::vector<int64_t>> policyInputShapes_;
std::vector<std::vector<int64_t>> policyOutputShapes_;

vector_t lastActions_;
vector_t defaultJointAngles_;
vector_t walkdefaultJointAngles_;
int actuatedDofNum_;
bool *isfirstRecObs_;
int actionsSize_;
int observationSize_;
int stackSize_;
float scalez;
float scalex;
float scaley;
std::vector<tensor_element_t> actions_;
std::vector<tensor_element_t> policyObservations_;

Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;
bool isfirstCompAct_{true};
vector_t command_;
Proprioception propri_;
double phase_;
joydata remote_data;
double standPercent;
scalar_t standDuration;
JointState lieJointState{0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
JointState standJointStatie_{0.0,    0.0,     0.0,    0.0,    0.0, 0.0, -0.1495,
                             0.3215, -0.1720, 0.0,    -0.0,   0.0, 0.0, 0.0,
                             0.0,    -0.1495, 0.3215, -0.1720};
WorkMode mode_;
bool isChangeMode_ = false;
bool startcontrol = false;
int initfinish = 0;

vector_t lieJointAngles_;
vector_t standJointAngles_;
Ort::MemoryInfo memoryInfo =
    Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
std::shared_ptr<Ort::Env> onnxEnvPrt_;

bool init() {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        printf("cur path is %s\n", path.c_str());
        YAML::Node acconfig = YAML::LoadFile(path + "/config/ning_user.yaml");
        standDuration = 1000;
        standPercent = 0;
        lieJointAngles_.resize(18);
        standJointAngles_.resize(18);
        auto &LieState = lieJointState;
        auto &StandState = standJointStatie_;
        lieJointAngles_ << LieState.arm_l1_joint, LieState.arm_l2_joint,
            LieState.arm_l3_joint, LieState.arm_l4_joint, LieState.leg_l1_joint,
            LieState.leg_l2_joint, LieState.leg_l3_joint, LieState.leg_l4_joint,
            LieState.leg_l5_joint, LieState.arm_r1_joint, LieState.arm_r2_joint,
            LieState.arm_r3_joint, LieState.arm_r4_joint, LieState.leg_r1_joint,
            LieState.leg_r2_joint, LieState.leg_r3_joint, LieState.leg_r4_joint,
            LieState.leg_r5_joint;
        standJointAngles_ << StandState.arm_l1_joint, StandState.arm_l2_joint,
            StandState.arm_l3_joint, StandState.arm_l4_joint,
            StandState.leg_l1_joint, StandState.leg_l2_joint,
            StandState.leg_l3_joint, StandState.leg_l4_joint,
            StandState.leg_l5_joint, StandState.arm_r1_joint,
            StandState.arm_r2_joint, StandState.arm_r3_joint,
            StandState.arm_r4_joint, StandState.leg_r1_joint,
            StandState.leg_r2_joint, StandState.leg_r3_joint,
            StandState.leg_r4_joint, StandState.leg_r5_joint;

        return true;
}

void setparameter(Command &cmd, bool *isfirst) {
        isfirstRecObs_ = isfirst;
        isfirstCompAct_ = *isfirstRecObs_;
        command_[0] = cmd.x;
        command_[1] = cmd.y;
        command_[2] = cmd.yaw;
}

bool updateStateEstimation() {
        vector_t jointPosnoarm(10), jointVelnoarm(10), jointTornoarm(10);

        quaternion_t quat;
        vector3_t angularVel, linearAccel;
        static int num = 0;

        std::array<MotorState, 18> joint_state = ctrl->get_joint_state();

        std::chrono::microseconds now =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch());

        int j = 0;

        for (size_t i = 0; i < actuatedDofNum_; ++i) {
                if ((i >= 4) && (i <= 8)) {
                        jointPosnoarm(j) = joint_state[i].pos;
                        jointVelnoarm(j) = joint_state[i].vel;
                        jointTornoarm(j) = joint_state[i].tau;
                        j++;
                }
                if ((i >= 13) && (i <= 17)) {
                        jointPosnoarm(j) = joint_state[i].pos;
                        jointVelnoarm(j) = joint_state[i].vel;
                        jointTornoarm(j) = joint_state[i].tau;
                        j++;
                }
        }

        NingImuData imudata = ctrl->get_imu_data();

        // imudata = get_imu_data();
        for (size_t i = 0; i < 4; ++i) {
                quat.coeffs()(i) = imudata.ori[i];
        }
        for (size_t i = 0; i < 3; ++i) {
                angularVel(i) = imudata.angular_vel[i];
                linearAccel(i) = imudata.linear_acc[i];
        }

        propri_.jointPos = jointPosnoarm;
        propri_.jointVel = jointVelnoarm;
        propri_.baseAngVel = angularVel;

        vector3_t gravityVector(0, 0, -1);
        vector3_t zyx = quatToZyx(quat);
        matrix_t inverseRot =
            getRotationMatrixFromZyxEulerAngles(zyx).inverse();
        propri_.projectedGravity = inverseRot * gravityVector;
        propri_.baseEulerXyz = quatToXyz(quat);
        double seconds = now.count();
        phase_ = seconds / 1000000.0;

        return true;
}

void computeActions() {
        std::vector<Ort::Value> policyInputValues;
        policyInputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(
            memoryInfo, policyObservations_.data(), policyObservations_.size(),
            policyInputShapes_[0].data(), policyInputShapes_[0].size()));
        // run inference
        Ort::RunOptions runOptions;
        std::vector<Ort::Value> outputValues = policySessionPtr->Run(
            runOptions, policyInputNames_.data(), policyInputValues.data(), 1,
            policyOutputNames_.data(), 1);
        if (isfirstCompAct_) {
                for (int i = 0; i < policyObservations_.size(); ++i) {
                        std::cout << policyObservations_[i] << " ";
                        if ((i + 1) % observationSize_ == 0) {
                                std::cout << std::endl;
                        }
                }
                isfirstCompAct_ = false;
        }

        for (int i = 0; i < actionsSize_; i++) {
                actions_[i] =
                    *(outputValues[0].GetTensorMutableData<tensor_element_t>() +
                      i);
        }
}

void computeObservation() {

        std::atomic<scalar_t> comm_x;
        std::atomic<scalar_t> comm_y;

        RobotCfg::ObsScales &obsScales = robotconfig.obsScales;

        // command
        vector_t proprioObs(observationSize_);
        if (*isfirstRecObs_) {

                for (int i = 0; i < 40; i++) {
                        proprioObs(i, 0) = 0.0;
                }

                for (size_t i = 0; i < stackSize_; i++) {
                        proprioHistoryBuffer_.segment(i * observationSize_,
                                                      observationSize_) =
                            proprioObs.cast<tensor_element_t>();
                }
                *isfirstRecObs_ = false;
        }

        vector_t command(5);
        comm_x = command_[0] * scalex;
        comm_y = command_[1] * scaley;

        double phase = phase_ / 0.64;

        command[0] = sin(2 * M_PI * phase);
        command[1] = cos(2 * M_PI * phase);
        command[2] = comm_x * obsScales.linVel;
        command[3] = comm_y * obsScales.linVel;
        command[4] = command_[2] * obsScales.angVel;

        vector_t actions(lastActions_);

        matrix_t commandScaler = Eigen::DiagonalMatrix<scalar_t, 3>(
            obsScales.linVel, obsScales.linVel, obsScales.angVel);

        proprioObs << command,                        // 5
            propri_.baseAngVel * obsScales.angVel,    // 3
            propri_.baseEulerXyz(0) * obsScales.quat, // 1
            propri_.baseEulerXyz(1) * obsScales.quat, // 1
            (propri_.jointPos - walkdefaultJointAngles_) *
                obsScales.dofPos,                // 10
            propri_.jointVel * obsScales.dofVel, // 10
            actions;                             // 10

        proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() -
                                   observationSize_) =
            proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() -
                                       observationSize_);
        proprioHistoryBuffer_.tail(observationSize_) =
            proprioObs.cast<tensor_element_t>();

        //clang-format on

        for (size_t i = 0; i < (observationSize_ * stackSize_); i++) {
                policyObservations_[i] =
                    static_cast<tensor_element_t>(proprioHistoryBuffer_[i]);
                // if(i < observationSize_)
                // std::cout << i << "obs:::" << estObservations_[i] <<
                // std::endl;
        }
        // Limit observation range
        scalar_t obsMin = -robotconfig.clipObs;
        scalar_t obsMax = robotconfig.clipObs;
        std::transform(policyObservations_.begin(), policyObservations_.end(),
                       policyObservations_.begin(),
                       [obsMin, obsMax](scalar_t x) {
                               return std::max(obsMin, std::min(obsMax, x));
                       });
}

void handleDefautMode() {
        // MotorCmd  motorcmd;
        std::array<MotorCmd, 18> motorcmd;
        for (int j = 0; j < 18; j++) {
                motorcmd[j].kd = 0.1;
                motorcmd[j].pos = 0;
                motorcmd[j].kp = 0;
                motorcmd[j].motor_id = j;
                motorcmd[j].vel = 0;
                motorcmd[j].tau = 0;
        }
        ctrl->set_joint(motorcmd);
}

void handleStandMode() {
        // MotorCmd motorcmd;
        std::array<MotorCmd, 18> motorcmd;
        if (standPercent <= 1) {
                for (int j = 0; j < 18; j++) {
                        scalar_t pos_des =
                            lieJointAngles_[j] * (1 - standPercent) +
                            standJointAngles_[j] * standPercent;
                        if (j < 4) {

                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 20;
                                motorcmd[j].kd = 0.1;
                                motorcmd[j].motor_id = j;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;

                                // controllerbase.set_joint(motorcmd);
                                // set_joint(motorcmd);
                        } else if ((j > 4) && (j < 9)) {

                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 50;
                                motorcmd[j].kd = 1;
                                motorcmd[j].motor_id = j;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;

                                // controllerbase.set_joint(motorcmd);
                                // set_joint(motorcmd);
                        } else if ((j > 9) && (j < 13)) {

                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 20;
                                motorcmd[j].kd = 0.1;
                                motorcmd[j].motor_id = j;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;

                                // controllerbase.set_joint(motorcmd);
                                // set_joint(motorcmd);

                        } else {

                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 50;
                                motorcmd[j].kd = 1;
                                motorcmd[j].motor_id = j;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;

                                // controllerbase.set_joint(motorcmd);
                                // set_joint(motorcmd);
                        }
                }
                ctrl->set_joint(motorcmd);
                standPercent += 1 / standDuration;
                standPercent = std::min(standPercent, scalar_t(1));
        }
}

void handleLieMode() {
        // MotorCmd motorcmd;
        std::array<MotorCmd, 18> motorcmd;
        const std::array<MotorState, 18> ms = ctrl->get_joint_state();
        if (standPercent <= 1) {
                for (int j = 0; j < 18; j++) {
                        scalar_t pos_des = ms.at(j).pos * (1 - standPercent) +
                                           lieJointAngles_[j] * standPercent;
                        if (j < 4) {
                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 20;
                                motorcmd[j].kd = 0.1;
                                motorcmd[j].motor_id = j;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;

                        } else if ((j > 4) && (j < 9)) {
                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 50;
                                motorcmd[j].kd = 1;
                                motorcmd[j].motor_id = j;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;

                        } else if ((j > 9) && (j < 13)) {

                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 20;
                                motorcmd[j].kd = 0.1;
                                motorcmd[j].motor_id = j;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;

                        } else {

                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 50;
                                motorcmd[j].kd = 1;
                                motorcmd[j].motor_id = j;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;
                        }
                }
                ctrl->set_joint(motorcmd);
                standPercent += 1 / standDuration;
                standPercent = std::min(standPercent, double(1));
        }
}

bool handleWalkMode() {
        if (updateStateEstimation() == false)
                return false;
        // compute observation & actions
        if (count % robotconfig.controlCfg.decimation == 0) {
                count = 0;
                computeObservation();
                computeActions();
                // limit action range
                scalar_t actionMin = -robotconfig.clipActions;
                scalar_t actionMax = robotconfig.clipActions;
                std::transform(
                    actions_.begin(), actions_.end(), actions_.begin(),
                    [actionMin, actionMax](scalar_t x) {
                            return std::max(actionMin, std::min(actionMax, x));
                    });
        }
        // set action
        int j = 0;
        // MotorCmd motorcmd;
        std::array<MotorCmd, 18> motorcmd;
        for (int i = 0; i < actionsSize_; i++) {
                if (i < 5)
                        j = i + 4;
                if (i > 4)
                        j = i + 8;
                std::string partName = jointNames[j];
                scalar_t pos_des =
                    actions_[i] * robotconfig.controlCfg.actionScale +
                    defaultJointAngles_(j);
                double stiffness =
                    robotconfig.controlCfg
                        .stiffness[partName]; // 根据关节名称获取刚度
                double damping = robotconfig.controlCfg
                                     .damping[partName]; // 根据关节名称获取阻尼
                // std::cout << "joint_name:" << partName << "kp:" << stiffness
                // << " kd:" << damping << std::endl;
                motorcmd[j].pos = pos_des;
                motorcmd[j].kp = stiffness;
                motorcmd[j].kd = damping;
                motorcmd[j].motor_id = j;
                motorcmd[j].vel = 0;
                motorcmd[j].tau = 0;

                lastActions_(i, 0) = actions_[i];
        }

        for (int i = 0; i < 8; i++) {
                if (i < 4)
                        j = i;
                if (i >= 4)
                        j = i + 5;
                std::string partName = jointNames[j];
                scalar_t pos_des;
                const std::array<MotorState, 18> ms = ctrl->get_joint_state();
                if (!ms.empty()) {
                        double cur_pos = ms.at(j).pos - defaultJointAngles_(j);
                        pos_des =
                            0.75 * cur_pos + 0.25 * defaultJointAngles_(j);

                        double stiffness =
                            robotconfig.controlCfg
                                .stiffness[partName]; // 根据关节名称获取刚度
                        double damping =
                            robotconfig.controlCfg
                                .damping[partName]; // 根据关节名称获取阻尼
                        motorcmd[j].pos = pos_des;
                        motorcmd[j].kp = stiffness;
                        motorcmd[j].kd = damping;
                        motorcmd[j].motor_id = j;
                        motorcmd[j].vel = 0;
                        motorcmd[j].tau = 0;
                }
        }
        ctrl->set_joint(motorcmd);
        count++;
        return true;
}

void process() {
        static int keyflag[14];
        if (initfinish == 0)
                return;
        Command cmd;
        auto now = Clock::now();
        long starttimestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch())
                .count();

        remote_data = ctrl->from_dds_get_joydata();
        cmd.x = remote_data.axes[1];
        cmd.y = 0;
        cmd.yaw = remote_data.axes[0];

        if ((remote_data.button[9] == 1) && (keyflag[9] == 0)) {
                if (!startcontrol) {
                        startcontrol = true;
                        standPercent = 0;
                        mode_ = WorkMode::LIE;
                        keyflag[9] = 1;
                        printf("TO LIE MODE\n");

                } else {
                        startcontrol = false;
                        mode_ = WorkMode::DEFAULT;
                        keyflag[9] = 1;
                        printf("stop control\n");
                }

        } else if (remote_data.button[9] == 0)
                keyflag[9] = 0;
        if ((remote_data.button[10] == 1) && (remote_data.button[2] == 1) &&
            (keyflag[10] == 0)) {
                if (startcontrol == true) {
                        if (mode_ != WorkMode::STAND) {
                                standPercent = 0;
                                mode_ = WorkMode::STAND;
                                printf("STAND2LIE\n");
                        } else if (mode_ == WorkMode::LIE) {
                                standPercent = 0;
                                mode_ = WorkMode::STAND;
                                printf("LIE2STAND\n");
                        }
                }
        } else if (remote_data.button[10] == 0)
                keyflag[10] = 0;
        if ((remote_data.button[5] == 1) && (remote_data.button[2] == 1) &&
            (keyflag[5] == 0)) {

                if (mode_ == WorkMode::STAND) {
                        standPercent = 0;
                        isChangeMode_ = true;
                        mode_ = WorkMode::USERWALK;
                        keyflag[5] = 1;
                        printf("TO USERWALK MODE\n");
                }

        } else if (remote_data.button[5] == 0)
                keyflag[5] = 0;
        if ((remote_data.button[11] == 1) && (keyflag[11] == 0)) {
                if (mode_ == WorkMode::USERWALK) {
                        isChangeMode_ = true;
                        mode_ = WorkMode::STAND;
                        printf("WALK2STAND\n");

                } else if (mode_ == WorkMode::DEFAULT) {
                        standPercent = 0;
                        printf("deftolie\n");
                        isChangeMode_ = true;
                        mode_ = WorkMode::LIE;
                }
        }
        switch (mode_) {
        case WorkMode::STAND:
                handleStandMode();
                break;
        case WorkMode::LIE:
                handleLieMode();
                break;
        case WorkMode::USERWALK:
                setparameter(cmd, &isChangeMode_);
                handleWalkMode();
                break;
        case WorkMode::DEFAULT:
                handleDefautMode();
                break;
        default:
                printf("Unexpected mode encountered: %d\n",
                       static_cast<int>(mode_));
                break;
        }
}

void onnxdatainit() {
        Ort::AllocatorWithDefaultOptions allocator;
        // ROS_INFO_STREAM("count: " <<
        // poliycfg->policySessionPtr->GetOutputCount());
        for (int i = 0; i < policySessionPtr->GetInputCount(); i++) {
                auto policyInputnamePtr =
                    policySessionPtr->GetInputNameAllocated(i, allocator);
                policyInputNodeNameAllocatedStrings.push_back(
                    std::move(policyInputnamePtr));
                policyInputNames_.push_back(
                    policyInputNodeNameAllocatedStrings.back().get());
                // inputNames_.push_back(sessionPtr_->GetInputNameAllocated(i,
                // allocator).get());
                policyInputShapes_.push_back(
                    policySessionPtr->GetInputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape());
                std::vector<int64_t> policyShape =
                    policySessionPtr->GetInputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape();
                std::cerr << "Policy Shape: [";
                for (size_t j = 0; j < policyShape.size(); ++j) {
                        std::cout << policyShape[j];
                        if (j != policyShape.size() - 1) {
                                std::cerr << ", ";
                        }
                }
                std::cout << "]" << std::endl;
        }

        for (int i = 0; i < policySessionPtr->GetOutputCount(); i++) {
                auto policyOutputnamePtr =
                    policySessionPtr->GetOutputNameAllocated(i, allocator);
                policyOutputNodeNameAllocatedStrings.push_back(
                    std::move(policyOutputnamePtr));
                policyOutputNames_.push_back(
                    policyOutputNodeNameAllocatedStrings.back().get());
                // outputNames_.push_back(sessionPtr_->GetOutputNameAllocated(i,
                // allocator).get());
                std::cout << policySessionPtr
                                 ->GetOutputNameAllocated(i, allocator)
                                 .get()
                          << std::endl;
                policyOutputShapes_.push_back(
                    policySessionPtr->GetOutputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape());
                std::vector<int64_t> policyShape =
                    policySessionPtr->GetOutputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape();
                std::cerr << "Policy Shape: [";
                for (size_t j = 0; j < policyShape.size(); ++j) {
                        std::cout << policyShape[j];
                        if (j != policyShape.size() - 1) {
                                std::cerr << ", ";
                        }
                }
                std::cout << "]" << std::endl;
        }
}

bool getmodelparam() {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string conpath = std::string(buf);
        std::string path = modelname;
        RobotCfg::InitState &initState = robotconfig.initState;
        RobotCfg::ControlCfg &controlCfg = robotconfig.controlCfg;
        RobotCfg::ObsScales &obsScales = robotconfig.obsScales;

        YAML::Node acconfig =
            YAML::LoadFile(conpath + "/config/ning_user.yaml");

        int error = 0;

        initState.arm_l1_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["arm_l1_joint"]
                                             .as<double>();
        initState.arm_l2_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["arm_l2_joint"]
                                             .as<double>();
        initState.arm_l3_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["arm_l3_joint"]
                                             .as<double>();
        initState.arm_l4_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["arm_l4_joint"]
                                             .as<double>();
        initState.arm_r1_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["arm_r1_joint"]
                                             .as<double>();
        initState.arm_r2_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["arm_r2_joint"]
                                             .as<double>();
        initState.arm_r3_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["arm_r3_joint"]
                                             .as<double>();
        initState.arm_r4_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["arm_r4_joint"]
                                             .as<double>();

        initState.leg_l1_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["leg_l1_joint"]
                                             .as<double>();
        initState.leg_l2_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["leg_l2_joint"]
                                             .as<double>();
        initState.leg_l3_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["leg_l3_joint"]
                                             .as<double>();
        initState.leg_l4_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["leg_l4_joint"]
                                             .as<double>();
        initState.leg_l5_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["leg_l5_joint"]
                                             .as<double>();

        initState.leg_r1_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["leg_r1_joint"]
                                             .as<double>();
        initState.leg_r2_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["leg_r2_joint"]
                                             .as<double>();
        initState.leg_r3_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["leg_r3_joint"]
                                             .as<double>();
        initState.leg_r4_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["leg_r4_joint"]
                                             .as<double>();
        initState.leg_r5_joint = acconfig[modelname]["init_state"]
                                         ["default_joint_angle"]["leg_r5_joint"]
                                             .as<double>();

        for (const auto &pair : acconfig[modelname]["control"]["stiffness"]) {
                controlCfg.stiffness[pair.first.as<std::string>()] =
                    pair.second.as<float>();
        }

        for (const auto &pair : acconfig[modelname]["control"]["damping"]) {
                controlCfg.damping[pair.first.as<std::string>()] =
                    pair.second.as<float>();
        }

        controlCfg.actionScale =
            acconfig[modelname]["control"]["action_scale"].as<float>();
        controlCfg.decimation =
            acconfig[modelname]["control"]["decimation"].as<int>();
        controlCfg.cycle_time =
            acconfig[modelname]["control"]["cycle_time"].as<float>();

        robotconfig.clipObs = acconfig[modelname]["normalization"]
                                      ["clip_scales"]["clip_observations"]
                                          .as<double>();
        robotconfig.clipActions =
            acconfig[modelname]["normalization"]["clip_scales"]["clip_actions"]
                .as<double>();

        obsScales.linVel =
            acconfig[modelname]["normalization"]["obs_scales"]["lin_vel"]
                .as<double>();
        obsScales.angVel =
            acconfig[modelname]["normalization"]["obs_scales"]["ang_vel"]
                .as<double>();
        obsScales.dofPos =
            acconfig[modelname]["normalization"]["obs_scales"]["dof_pos"]
                .as<double>();
        obsScales.dofVel =
            acconfig[modelname]["normalization"]["obs_scales"]["dof_vel"]
                .as<double>();
        obsScales.heightMeasurements =
            acconfig[modelname]["normalization"]["obs_scales"]
                    ["height_measurements"]
                        .as<double>();
        obsScales.quat =
            acconfig[modelname]["normalization"]["obs_scales"]["quat"]
                .as<double>();

        actionsSize_ = acconfig[modelname]["size"]["actions_size"].as<int>();
        observationSize_ =
            acconfig[modelname]["size"]["observations_size"].as<int>();
        stackSize_ = acconfig[modelname]["size"]["stack_size"].as<int>();

        scalez = acconfig[modelname]["axis_mappings"]["scalez"].as<float>();
        scaley = acconfig[modelname]["axis_mappings"]["scaley"].as<float>();
        scalex = acconfig[modelname]["axis_mappings"]["scalex"].as<float>();

        actions_.resize(actionsSize_);

        std::vector<scalar_t> defaultJointAngles{
            robotconfig.initState.arm_l1_joint,
            robotconfig.initState.arm_l2_joint,
            robotconfig.initState.arm_l3_joint,
            robotconfig.initState.arm_l4_joint,
            robotconfig.initState.leg_l1_joint,
            robotconfig.initState.leg_l2_joint,
            robotconfig.initState.leg_l3_joint,
            robotconfig.initState.leg_l4_joint,
            robotconfig.initState.leg_l5_joint,
            robotconfig.initState.arm_r1_joint,
            robotconfig.initState.arm_r2_joint,
            robotconfig.initState.arm_r3_joint,
            robotconfig.initState.arm_r4_joint,
            robotconfig.initState.leg_r1_joint,
            robotconfig.initState.leg_r2_joint,
            robotconfig.initState.leg_r3_joint,
            robotconfig.initState.leg_r4_joint,
            robotconfig.initState.leg_r5_joint};

        std::vector<scalar_t> walkdefaultJointAngles{
            robotconfig.initState.leg_l1_joint,
            robotconfig.initState.leg_l2_joint,
            robotconfig.initState.leg_l3_joint,
            robotconfig.initState.leg_l4_joint,
            robotconfig.initState.leg_l5_joint,
            robotconfig.initState.leg_r1_joint,
            robotconfig.initState.leg_r2_joint,
            robotconfig.initState.leg_r3_joint,
            robotconfig.initState.leg_r4_joint,
            robotconfig.initState.leg_r5_joint};
        actuatedDofNum_ = 18;

        policyObservations_.resize(observationSize_ * stackSize_);

        std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);
        lastActions_.resize(actionsSize_);
        lastActions_.setZero();
        const int inputSize = stackSize_ * observationSize_;
        proprioHistoryBuffer_.resize(inputSize);
        defaultJointAngles_.resize(actuatedDofNum_);
        walkdefaultJointAngles_.resize(actionsSize_);
        for (int i = 0; i < actuatedDofNum_; i++) {
                defaultJointAngles_(i) = defaultJointAngles[i];
                // printf("defaultJointAngles[%d]
                // %f\n",i,modelcfg->defaultJointAngles_(i));
        }
        for (int i = 0; i < actionsSize_; i++) {
                walkdefaultJointAngles_(i) = walkdefaultJointAngles[i];
                // printf("defaultJointAngles[%d]
                // %f\n",i,modelcfg->defaultJointAngles_(i));
        }
        return true;
}

bool loadModel(std::string modelpath) {
        std::string estFilePath;
        // create session
        Ort::SessionOptions sessionOptions;
        bool ret;
        onnxEnvPrt_.reset(
            new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
        sessionOptions.SetInterOpNumThreads(1);
        if (onnxEnvPrt_ == NULL) {
                printf("onnxEnvPrt_  is null\n");
                return false;
        }

        printf("Load Onnx model from path : %s\n", modelpath.c_str());

        policySessionPtr = std::make_unique<Ort::Session>(
            *onnxEnvPrt_, modelpath.c_str(), sessionOptions);
        if (policySessionPtr == NULL) {
                printf("load run model failed\n");
                return false;
        }

        // get input and output info
        policyInputNames_.clear();
        policyOutputNames_.clear();
        policyInputShapes_.clear();
        policyOutputShapes_.clear();
        modelname = "user";
        command_.resize(3);
        isfirstCompAct_ = true;
        isfirstRecObs_ = NULL;
        count = 0;
        onnxdatainit();
        ret = getmodelparam();
        initfinish = 1;

        printf("Load Onnx run model successfully !!!\n");
        return true;
}

int main() {
        char buf[256];
        bool ret = true;
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        std::string ddsxml = "file://" + path + "/config/dds.xml";
        setenv("CYCLONEDDS_URI", ddsxml.c_str(), 1);
        printf("cur path is %s\n", path.c_str());
        ctrl = legged::LowController::Instance();
        ctrl->init();
        loadModel(path + "/policy/policy_user.onnx");
        init();
        while (1) {
                auto start_time = std::chrono::steady_clock::now();
                process();
                auto end_time = std::chrono::steady_clock::now();
                auto elapsed =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time);
                std::this_thread::sleep_for(std::chrono::microseconds(2000) -
                                            elapsed);
        }
        return 0;
}
