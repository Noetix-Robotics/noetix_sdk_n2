#include "highcontroller.h"
//#include "controllerbase.h"
#include "RotationTools.h"
#include <algorithm>
#include "common.h"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <stdio.h>
#include "aolion_driver.h"
using namespace org::eclipse::cyclonedds;

namespace legged
{
  DataBuffer<RobotMotorCmd::MotorCmdArray> motor_cmd_buffer_;
  DataBuffer<std::array<MotorState,18>> motor_state_buffer_;
  DataBuffer<joydata> joy_buffer_;
  DataBuffer<NingImuData> imu_buffer_;
  HighController* HighController::instance = nullptr;
  AoLionDriver remotedriver;
  int longd = 0;
  int shortd = 0;

  bool HighController::init(ControlMode mode)
  {
    char buf[256];
    getcwd(buf,sizeof(buf)); 
    std::string path=std::string(buf);
    printf("cur path is %s\n",path.c_str());
 
    mode_ = WorkMode::DEFAULT;

    instance = this;
    remotedriver.init("/dev/input/js0",115200);
    RobotSetMode::SetMode cmode;
    if (mode == ControlMode::HIGHMODE) {
      cmode.mode(1);
    } else if (mode == ControlMode::LOWMODE) {
      cmode.mode(2);
    }
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
    
          //std::cout<< "motorstate  id: "<< motorstate_[i].motor_id<<";pos " << motorstate_[i].pos << ";vel " << motorstate_[i].vel
          //<< ";tau "<<motorstate_[i].tau<<";error " <<motorstate_[i].error<<";temperature "<<motorstate_[i].temperature<<std::endl;
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
          // for(int i = 0;i<14;i++)
          // {
           
          //   if(remote_data.button[i] == 1)
          //      printf("button %d  press\n",i);
          // }     
              
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
    // for(int i = 0;i<14;i++)
    // {
     
    //   if(joy_data.button[i] == 1)
    //      printf("button %d  press\n",i);
    // }   
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
            cmdarray.controlcmd().data() = mc->controlcmd().data();
            auto now = Clock::now();
            long long timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
            cmdarray.timestamp() = timestamp;
	    //printf("send dds cmd\n");
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
  void HighController::set_axes(double  ver,double hor,int action,uint16_t index)
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
     //cmdarray.controlcmd().data() = data;
     if(action == 9 || action == 11)
     {
      cmdarray.controlcmd().data() = index;
  
     }
     if(action == 5 )
     {
        if(longd == 1)
          cmdarray.controlcmd().data()  = 0;//long dance
        else if(shortd == 1)
          cmdarray.controlcmd().data()  = 1; //short dance
     }
    // printf(" cmdarray.controlcmd().data() %d\n",cmdarray.controlcmd().data()); 
    //  else 
    //    cmdarray.controlcmd().data() = 0; 
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

  uint16_t fileindex=0;
  
  void HighController::process()
  {
    
    static int keyflag[14];
    joydata remote_data;
    static char key_updown[14],key_inuse[14];
    Command  cmd;
    
    auto now = Clock::now();
    long starttimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    //printf("process timestamp %d \n",starttimestamp);
    remote_data =  remotedriver.getremotedata();
    for(int i=0;i<14;i++)
    {
      if(remote_data.button[i] == 0)
      {
        key_updown[i] = 0;
        key_inuse[i] = 0;
      }else if(remote_data.button[i] == 1)
      {
        key_updown[i] = 1;

      }
    }
    // const std::shared_ptr<const joydata> jdata =joy_buffer_.GetData();
    // if(jdata)
    // {
    //   memcpy(remote_data.button,&(*jdata).button[0],sizeof(remote_data.button));
    //   memcpy(remote_data.axes,&(*jdata).axes[0],sizeof(remote_data.axes));
    //   // for(int i = 0;i<14;i++)
    //   // {
       
    //   //   if(remote_data.button[i] == 1)
    //   //      printf("button %d  press\n",i);
    //   // }   
    // }else 
    //   printf("joydata null\n");
    //   for(int i=0;i<14;i++)
    //   {
    //     if(remote_data.button[i] == 0)
    //     {
    //       key_updown[i] = 0;
    //       key_inuse[i] = 0;
    //     }else if(remote_data.button[i] == 1)
    //     {
    //       key_updown[i] = 1;
  
    //     }
    //   }  
    ControlCmd action = ControlCmd::DEFAULT;
    cmd.x = 0.0;
    cmd.y=0;
    cmd.yaw =0.0;
    if(key_updown[Key2]== 0  && key_updown[Key5] ==1 &&(key_inuse[Key5] == 0))
    {
      action = ControlCmd::STARTTEACH;
      key_inuse[Key5] = 1;
      printf("STARTTEACH \n");
    }
    else if((key_updown[Key6] == 1) && key_updown[Key2]== 0 && (key_inuse[Key6]==0))
    {
      action = ControlCmd::SWING;
      key_inuse[Key6] =1;
    }
    else if((key_updown[Key7] == 1) && key_updown[Key2]== 0 && (key_inuse[Key7]==0))
    {
      action = ControlCmd::SAVETEACH;
      key_inuse[Key6] = 1;
      fileindex++;
      printf("SAVETEACH \n");
    }
    else if((key_updown[Key8] == 1)  && key_updown[Key2]== 0  && (key_inuse[Key8]==0))
    {
      action = ControlCmd::CHEER;
      key_inuse[Key8] =1; 
    }
    else if((key_updown[Key9] == 1) && (key_inuse[Key9]==0))
    {
      key_inuse[Key9] = 1;
      action = ControlCmd::START;
    }
    else if(key_updown[Key10]== 1 && (key_inuse[Key10]==0) )
    {
      action = ControlCmd::SWITCH;
      key_inuse[Key10] =1;
      printf("SWITCH \n");
    }
    else if(key_updown[Key2]== 1  && key_updown[Key5] ==1 &&(key_inuse[Key5] == 0))
    {
      action = ControlCmd::WALK;
      key_inuse[Key5] = 1;
      printf("WALK \n");
    }
    else if(key_updown[Key1]== 1  && key_updown[Key6] ==1 &&(key_inuse[Key6] == 0))
    {
      action = ControlCmd::SAVETEACH;
      key_inuse[Key6] = 1;
      fileindex++;
      printf("SAVETEACH \n");
    }
    else if(key_updown[Key1]== 1  && key_updown[Key7] ==1 &&(key_inuse[Key7] == 0))
    {
      action = ControlCmd::ENDTEACH;
      key_inuse[Key7] = 1;
      printf("ENDTEACH \n");
    }
    else if(key_updown[Key1]== 1  && key_updown[Key8] ==1 &&(key_inuse[Key8] == 0))
    {
      action = ControlCmd::PLAYTEACH;
      key_inuse[Key8] = 1;
      fileindex =1;
      printf("PLAYTEACH \n");
    }
    else if(key_updown[Key2]== 1  && key_updown[Key8] ==1 &&(key_inuse[Key8] == 0))
    {
      action = ControlCmd::PLAYTEACH;
      key_inuse[Key8] = 1;
      fileindex =1;
      printf("PLAYTEACH \n");
    }
    
    set_axes(cmd.x ,cmd.yaw,(int)action,fileindex);
    //printf("remote button[9]   %d\n",remote_data.button[9]);

    

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
