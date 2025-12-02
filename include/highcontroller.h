#ifndef HighController_H
#define HighController_H


#include "common.h"
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <atomic>
#include "DDSWrapper.h"

#include <iostream>
#include <queue>
#include <thread>
#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>


#ifdef RK3588
#include "rknn_api.h"
#endif
using namespace org::eclipse::cyclonedds;

namespace legged
{

enum class WorkMode : uint8_t
{
  STAND,
  LIE,
  USERWALK,
  DEFAULT
};

  struct RobotCfg
  {
    struct ControlCfg
    {
      std::map<std::string, float> stiffness;
      std::map<std::string, float> damping;
      float actionScale;
      int decimation;
      float user_torque_limit;
      float user_power_limit;
      float cycle_time;
    };

    struct InitState
    {
      // default joint angles
      scalar_t arm_l1_joint;
      scalar_t arm_l2_joint;
      scalar_t arm_l3_joint;
      scalar_t arm_l4_joint;
      scalar_t leg_l1_joint;
      scalar_t leg_l2_joint;
      scalar_t leg_l3_joint;
      scalar_t leg_l4_joint;
      scalar_t leg_l5_joint;
      scalar_t arm_r1_joint;
      scalar_t arm_r2_joint;
      scalar_t arm_r3_joint;
      scalar_t arm_r4_joint;
      scalar_t leg_r1_joint;
      scalar_t leg_r2_joint;
      scalar_t leg_r3_joint;
      scalar_t leg_r4_joint;
      scalar_t leg_r5_joint;
    };

    struct ObsScales
    {
      scalar_t linVel;
      scalar_t angVel;
      scalar_t dofPos;
      scalar_t dofVel;
      scalar_t quat;
      scalar_t heightMeasurements;
    };

    bool encoder_nomalize;

    scalar_t clipActions;
    scalar_t clipObs;

    InitState initState;
    ObsScales obsScales;
    ControlCfg controlCfg;

  
    int loophz;
    double cycletimeerrorThreshold;
    int ThreadPriority;

  };
template <typename T>
class DataBuffer {
  public:
   void SetData(const T &newData){
    std::unique_lock<std::shared_mutex> lock(mutex);
    data= std::make_shared<T>(newData);
    if(data == nullptr)
      printf("set data failed\n");
   }
   std::shared_ptr<const T> GetData(){
    std::shared_lock<std::shared_mutex> lock(mutex);
    return data ? data : nullptr;
   }
   void Clear(){
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = nullptr;
   }
   private:
    std::shared_ptr<T> data;
    std::shared_mutex mutex;
};
DataBuffer<std::array<MotorState,18>> motor_state_buffer_;
DataBuffer<joydata> joy_buffer_;
DataBuffer<NingImuData> imu_buffer_;
class HighController 
{
   
public:
   
 

    ~HighController()  = default;
    static HighController* Instance()
    {
       static HighController  highcontrol;
       return &highcontrol;
    }
    static HighController* instance; // 静态指针，用于调用非静态方法


    bool init(ControlMode mode );


    
    void process();

  protected:
   
    void onnxdatainit();
    bool getmodelparam();
    bool updateStateEstimation(); 
    const std::array<MotorState,18> get_joint_state();
    void set_axes(double  ver,double hor,int action);
    const joydata get_jsdata();
    const NingImuData get_imu_data();
    void send_thread_func();
    void process_thread_func();
    void set_imudata(NingImuData imu_data);
    void set_joydata(joydata joy_data);
    void set_robotstatusdata(std::array<MotorState,18> motorstate_data,NingImuData imudata,joydata joy_data);

  private:


    int64_t count;

    DDSWrapper ddswrapper;
    vector3_t baseLinVel_;
    vector3_t basePosition_;
    vector_t lastActions_;
    vector_t defaultJointAngles_;
    vector_t walkdefaultJointAngles_;
    int actuatedDofNum_;
    bool* isfirstRecObs_;
    int actionsSize_;
    int motionSize;
    int observationSize_;
    int stackSize_;
    float scalez;
    float scalex;
    float scaley;
    //RemoteDriver remotedriver;
    

    vector_t command_;

    int64_t loopcount_;
    NingImuData imu_data_;
    joydata remote_data_;
   
   // NingImuData imu_data;
    std::array<MotorState,18> motorstate_;
    std::queue<std::array<MotorState,18>> statequeue;
    std::queue<RobotMotorCmd::MotorCmdArray> cmdqueue;
                        
    WorkMode  mode_;        
    bool isChangeMode_=false;                                 
    bool startcontrol = false; 
    int initfinish = 0;
    int model_type;
    long long statetimestamp;
    std::mutex state_mutex;
    std::array<MotorCmd,18>  usermotorcmd;
    
    DataBuffer<RobotMotorCmd::MotorCmdArray> motor_cmd_buffer_;
    // DataBuffer<std::array<MotorState,18>> motor_state_buffer_;
    // DataBuffer<joydata> joy_buffer_;
    // DataBuffer<NingImuData> imu_buffer_;


    std::thread send_thread_;
    std::thread process_thread_;

   
};
}
#endif