#include "highcontroller.h"
//#include "controllerbase.h"
#include "RotationTools.h"
#include <algorithm>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <stdio.h>
using namespace org::eclipse::cyclonedds;

namespace legged
{

  HighController* HighController::instance = nullptr;


  bool HighController::init(ControlMode mode)
  {
    char buf[256];
    getcwd(buf,sizeof(buf)); 
    std::string path=std::string(buf);
    printf("cur path is %s\n",path.c_str());
 
    mode_ = WorkMode::DEFAULT;

    instance = this;
   
    RobotSetMode::SetMode cmode;
    if(mode == ControlMode::LOWMODE )
      cmode.mode(2);
    else if(mode == ControlMode::HIGHMODE )
      cmode.mode(1);

    ddswrapper.publishModeData(cmode);
    
    ddswrapper.subscribeRobotStatus([] (const  RobotStatus::StatusData& ddsdata){
      std::array<MotorState,18>   data;
      joydata remote_data;
      NingImuData imudata;
      int i=0;
       for(const auto& state : ddsdata.motorstatearray().motorstates())
        {
          data[i].pos = state.pos();
          data[i].vel = state.vel();
          data[i].tau = state.tau();
          data[i].motor_id = state.motor_id();
          data[i].error = state.error();
          data[i].temperature = state.temperature();

          i++;
        }
        for(int i=0;i<4;i++)
        {
          imudata.ori[i] = ddsdata.imudata().ori()[i];
        }
        for(int i=0;i<3;i++)
        {
          imudata.angular_vel[i] = ddsdata.imudata().angular_vel()[i];
          imudata.linear_acc[i] = ddsdata.imudata().linear_acc()[i];
        }
        for(int i=0;i<9;i++)
        {
          imudata.ori_cov[i] = ddsdata.imudata().ori_cov()[i];
          imudata.angular_vel_cov[i] = ddsdata.imudata().angular_vel_cov()[i];
          imudata.linear_acc_cov[i] = ddsdata.imudata().linear_acc_cov()[i];
        }
          memcpy(remote_data.button,&ddsdata.joydata().button(),sizeof(remote_data.button));
          memcpy(remote_data.axes,&ddsdata.joydata().axes(),sizeof(remote_data.axes));
           
              
      HighController::Instance()->set_robotstatusdata(data,imudata,remote_data);
    });   


    send_thread_ = std::thread(&HighController::send_thread_func,this);
    process_thread_ = std::thread(&HighController::process_thread_func,this);
    sched_param ddssched{ .sched_priority = 98};
    if(pthread_setschedparam(process_thread_.native_handle(),SCHED_FIFO,&ddssched) != 0 )
    {
      printf(" failed to set threads priority\n");
    }
    if(pthread_setschedparam(send_thread_.native_handle(),SCHED_FIFO,&ddssched) != 0 )
    {
      printf(" failed to set threads priority\n");
    }
    return true;
  }
  void HighController::set_imudata(NingImuData data)
  {
    for(int i=0;i<4;i++)
    {
      imu_data_.ori[i] = data.ori[i];
    }
    for(int i=0;i<3;i++)
    {
      imu_data_.angular_vel[i] = data.angular_vel[i];
      imu_data_.linear_acc[i] = data.linear_acc[i];
    }
    for(int i=0;i<9;i++)
    {
      imu_data_.ori_cov[i] = data.ori_cov[i];
      imu_data_.angular_vel_cov[i] = data.angular_vel_cov[i];
      imu_data_.linear_acc_cov[i] = data.linear_acc_cov[i];
    }
  }
  void HighController::set_joydata(joydata data)
  {
    memcpy(remote_data_.button,&data.button,sizeof(data.button));
    memcpy(remote_data_.axes,&data.axes,sizeof(data.axes));
  }
  void HighController::set_robotstatusdata( std::array<MotorState,18>   data,NingImuData imudata,joydata joy_data)
  {
    motor_state_buffer_.SetData(data);
    imu_buffer_.SetData(imudata);
    joy_buffer_.SetData(joy_data);
  
  }

  void HighController::send_thread_func()
  {

      while(1)
      {
          const std::shared_ptr<const RobotMotorCmd::MotorCmdArray> mc = motor_cmd_buffer_.GetData();
          if(mc)
          {
             RobotMotorCmd::MotorCmdArray cmdarray;
             cmdarray.motorcmds().resize(18);
            for(int i = 0;i<18;i++)
            {
                auto& cmd  = cmdarray.motorcmds()[i];
          
                cmd.pos()= mc->motorcmds().at(i).pos();
                cmd.vel() =mc->motorcmds().at(i).vel();
                cmd.tau() = mc->motorcmds().at(i).tau();
                cmd.kp() = mc->motorcmds().at(i).kp();
                cmd.kd() = mc->motorcmds().at(i).kd();
                cmd.motor_id() = mc->motorcmds().at(i).motor_id();
            }
            cmdarray.controlcmd().axes()[0] = mc->controlcmd().axes().at(0);
            cmdarray.controlcmd().axes()[1] = mc->controlcmd().axes().at(1);
            cmdarray.controlcmd().action() = mc->controlcmd().action();
            auto now = Clock::now();
            long long timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
            cmdarray.timestamp() = timestamp;

            ddswrapper.publishMotorCmdData(cmdarray);
          }
          std::this_thread::sleep_for(std::chrono::microseconds(2000));
      }
  }
  void HighController::process_thread_func()
  {
     while(1)
     {
        auto start_time =  std::chrono::steady_clock::now();
        process();

        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        std::this_thread::sleep_for(std::chrono::microseconds(2000)-elapsed);
     }
  }
  const NingImuData HighController::get_imu_data()
  {
     return imu_data_;
  }
  const joydata HighController::get_jsdata()
  {
     return remote_data_;
  }
  void HighController::set_axes(double  ver,double hor,int action)
  {
   
     RobotMotorCmd::MotorCmdArray cmdarray;
     cmdarray.motorcmds().resize(18);
     for(int i = 0;i<18;i++)
     {
        auto& cmd  = cmdarray.motorcmds()[i];
  
        cmd.pos()= 0;
        cmd.vel() =0;
        cmd.tau() = 0;
        cmd.kp() = 0;
        cmd.kd() = 0;
        cmd.motor_id() = 0;
     }
     
     auto now = Clock::now();
     long long timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
     int diff = timestamp-statetimestamp;
     cmdarray.timestamp() = timestamp;
     cmdarray.controlcmd().axes()[0] = hor;
     cmdarray.controlcmd().axes()[1] = ver;
     cmdarray.controlcmd().action() = action;
     motor_cmd_buffer_.SetData(cmdarray);

  }
   const std::array<MotorState,18>  HighController::get_joint_state()
  {
    std::array<MotorState,18>  motorstate;
     const std::shared_ptr<const std::array<MotorState,18>> ms =motor_state_buffer_.GetData();
     if(ms)
     {
        for(int i=0;i<18;i++)
        {
          motorstate[i].pos  = ms->at(i).pos;
          motorstate[i].vel  = ms->at(i).vel;
          motorstate[i].tau  = ms->at(i).tau;
          motorstate[i].motor_id  = ms->at(i).motor_id;
          motorstate[i].error  = ms->at(i).error;
          motorstate[i].temperature  = ms->at(i).temperature;
        }
   
     }
    return motorstate;
  }

  
  
  void HighController::process()
  {
    
    static int keyflag[14];
    joydata remote_data;
 
    Command  cmd;
    auto now = Clock::now();
    long starttimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    const std::shared_ptr<const std::array<MotorState,18>> ms =motor_state_buffer_.GetData();

 
    static int action =0;
    cmd.x = 0.2;
    cmd.y=0;
    cmd.yaw =0.3;
    if(remote_data.button[5] == 1)
      action = 0;
    else if(remote_data.button[6] == 1)
      action = 1;
    else if(remote_data.button[7] == 1)
      action = 2;
    else if(remote_data.button[8] == 1)
      action = 3;  
    set_axes(cmd.x ,cmd.yaw,action);


    

  }
  
  
  
  

 }// namespace legged

int main()
{
  // setenv("CYCLONEDDS_URI","file:///home/oem/test/dds-test/config/dds.xml",1);
  char buf[256];
  bool ret = true;
  getcwd(buf,sizeof(buf)); 
  std::string path=std::string(buf);
  std::string ddsxml = "file://"+path+"/config/dds.xml";
  setenv("CYCLONEDDS_URI",ddsxml.c_str(),1);
  printf("cur path is %s\n",path.c_str());
  legged::HighController highcontroller;
  highcontroller.init(legged::ControlMode::HIGHMODE);

  while(1)
  {
    usleep(10);
  }
  return 0;
    
}
