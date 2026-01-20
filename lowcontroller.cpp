#include "lowcontroller.h"
//#include "controllerbase.h"
#include "RotationTools.h"
#include <algorithm>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <stdio.h>


namespace legged
{
    DataBuffer<std::array<MotorCmd,18>> motor_cmd_buffer_;
    DataBuffer<std::array<MotorState,18>> motor_state_buffer_;
    DataBuffer<joydata> joy_buffer_;
    DataBuffer<NingImuData> imu_buffer_;
  //Controllerbase controllerbase;
   LowController* LowController::instance = nullptr;

  #ifdef RK3588
  unsigned char *LowController::load_model(const char* filename,int* model_size)
  {
     FILE* fp=fopen(filename,"rb");
    if(fp == nullptr){
      printf("fopen %s fail \n",filename);
      return NULL;
    }
    fseek(fp,0,SEEK_END);
    int model_len=ftell(fp);
    unsigned char* model = (unsigned char*)malloc(model_len);
    fseek(fp,0,SEEK_SET);
    if(model_len != fread(model,1,model_len,fp)){
      printf("fread %s fail \n",filename);
      free(model);
      return NULL;
    }
    *model_size = model_len;
    if(fp)
    {
      fclose(fp);
    }
    return model;
  }
  void LowController::dump_tensor_attr(rknn_tensor_attr* attr)
  {
     std::string shape_str = attr->n_dims < 1 ? "" : std::to_string(attr->dims[0]);
     for(int i =1;i<attr->n_dims;++i)
     {
       shape_str +=", "+std::to_string(attr->dims[i]);
     }
     printf(" index = %d ,name= %s,n_dims=%d,dims=[%s],n_elems=%d,size=%d,w_stride=%d,size_with_stride=%d,fmt=%s,"
           "type=%s, qnt_type=%s,zp=%d,scale=%f\n",
           attr->index,attr->name,attr->n_dims,shape_str.c_str(),attr->n_elems,attr->size,attr->w_stride,attr->size_with_stride,get_format_string(attr->fmt),get_type_string(attr->type),
           get_qnt_type_string(attr->qnt_type),attr->zp,attr->scale);
  }
  unsigned int LowController::as_uint(const float x)
  {
    return *(unsigned int*)&x;
  }
  float LowController::as_float(const unsigned int x)
  {
    return *(float*)&x;
  }
  float  LowController::half_to_float(const unsigned short x)
  {
    const unsigned int e=(x&0x7c00)>>10;
    const unsigned int m=(x&0x03ff)<<13;
    const unsigned int v=as_uint((float)m)>>23;
    return as_float((x&0x8000)<<16 | (e!=0)*((e+112)<<23|m) | ((e==0)&(m!=0))*((v-37)<<23 |((m<<(150-v))&0x007fe000)));
  }
  unsigned short LowController::float_to_half(const float x)
  {
    const unsigned int b = as_uint(x)+0x00001000;
    const unsigned int e = (b&0x7f800000)>>23;
    const unsigned int m = b&0x007fffff;
    return (b&0x80000000)>>16 | (e>112)*((((e-112)<<10)&0x7c00)|m>>13) | ((e<113)&(e>101))*((((0x007ff000+m)>>(125-e))+1)>>1)|(e>143)*0x7fff;
  }
  #endif
  bool LowController::loadrknnmodel(std::string modle_path)
  { 
    #ifdef RK3588
    std::string policyFilePath;  
    int model_data_size =0;
    int ret = 0;
    policyFilePath = modle_path+"/policy_user.rknn";
    unsigned char* model_data=load_model(policyFilePath.c_str(),&model_data_size);
    ret = rknn_init(&ctx,model_data,model_data_size,0,NULL);
    if(ret <0)
    {
      printf("rknn init error %d \n",ret);
      return 0;
    }
    rknn_core_mask core_mask = RKNN_NPU_CORE_0_1_2;
    ret = rknn_set_core_mask(ctx,core_mask);
    if(ret <0)
    {
      printf("rknn rknn_set_core_maskerror %d \n",ret);
      return 0;
    } 

    rknn_sdk_version version;
    ret = rknn_query(ctx,RKNN_QUERY_SDK_VERSION,&version,sizeof(rknn_sdk_version));
    if(ret <0)
    {
      printf("rknn RKNN_QUERY_SDK_VERSION error %d \n",ret);
      return 0;
    }
    printf("sdk version : %s driver version : %s\n",version.api_version,version.drv_version);
    
    ret = rknn_query(ctx,RKNN_QUERY_IN_OUT_NUM,&io_num,sizeof(io_num));
    if(ret <0)
    {
      printf("rknn RKNN_QUERY_IN_OUT_NUM error %d \n",ret);
      return 0;
    }
    printf("model input num: %d,output num: %d \n",io_num.n_input,io_num.n_output);
    input_attrs.resize(io_num.n_input);
    for(int i=0;i<io_num.n_input;i++){
      input_attrs[i].index =i;
      ret = rknn_query(ctx,RKNN_QUERY_INPUT_ATTR,&(input_attrs[i]),sizeof(rknn_tensor_attr));
      if(ret <0)
      {
        printf("rknn RKNN_QUERY_INPUT_ATTR error %d \n",ret);
        return 0;
      }
      printf(" input info: ");
      dump_tensor_attr(&(input_attrs[i]));
    }

    output_attrs.resize(io_num.n_output);
    for(int i=0;i<io_num.n_output;i++){
      output_attrs[i].index =i;
      ret = rknn_query(ctx,RKNN_QUERY_OUTPUT_ATTR,&(output_attrs[i]),sizeof(rknn_tensor_attr));
      if(ret <0)
      {
        printf("rknn RKNN_QUERY_OUTPUT_ATTR error %d \n",ret);
        return -1;
      }
      printf(" output info: ");
      dump_tensor_attr(&(output_attrs[i]));
    }
    
    modelname="user";
    command_.resize(3);
    isfirstCompAct_ = true;
    isfirstRecObs_= NULL;
    
    model_type =1;
    ret = getmodelparam();
    inputfp16.resize(observationSize_ * stackSize_);
    std::fill(inputfp16.begin(), inputfp16.end(), 0);
    initfinish = 1;
    #endif
    return true;
    
  }
  bool LowController::init(ControlMode mode)
  {
    char buf[256];
    getcwd(buf,sizeof(buf)); 
    std::string path=std::string(buf);
    printf("cur path is %s\n",path.c_str());
    YAML::Node  acconfig = YAML::LoadFile(path+"/config/ning_user.yaml");
    mode_ = WorkMode::DEFAULT;
    standDuration = 1000;
    standPercent = 0;
    lieJointAngles_.resize(18);
    standJointAngles_.resize(18);
    auto &LieState = lieJointState;
    auto &StandState = standJointStatie_;
    lieJointAngles_ <<  LieState.arm_l1_joint, LieState.arm_l2_joint, LieState.arm_l3_joint,LieState.arm_l4_joint, 
                  LieState.leg_l1_joint, LieState.leg_l2_joint, LieState.leg_l3_joint,LieState.leg_l4_joint, LieState.leg_l5_joint,
                  LieState.arm_r1_joint, LieState.arm_r2_joint, LieState.arm_r3_joint,LieState.arm_r4_joint, 
                  LieState.leg_r1_joint, LieState.leg_r2_joint, LieState.leg_r3_joint,LieState.leg_r4_joint, LieState.leg_r5_joint;
    standJointAngles_ <<  StandState.arm_l1_joint, StandState.arm_l2_joint, StandState.arm_l3_joint,StandState.arm_l4_joint, 
                            StandState.leg_l1_joint, StandState.leg_l2_joint, StandState.leg_l3_joint,StandState.leg_l4_joint, StandState.leg_l5_joint,
                            StandState.arm_r1_joint, StandState.arm_r2_joint, StandState.arm_r3_joint,StandState.arm_r4_joint, 
                            StandState.leg_r1_joint, StandState.leg_r2_joint, StandState.leg_r3_joint,StandState.leg_r4_joint, StandState.leg_r5_joint;
    instance = this;
    
    RobotSetMode::SetMode cmode;
    if (mode == ControlMode::HIGHMODE) {
      cmode.mode(1);
    } else if (mode == ControlMode::LOWMODE) {
      cmode.mode(2);
    }
    //cmode.mode(2);
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
           
              
      LowController::Instance()->set_robotstatusdata(data,imudata,remote_data);
    });   

   
    
   
    process_thread_ = std::thread(&LowController::process_thread_func,this);
    send_thread_ = std::thread(&LowController::send_thread_func,this);
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
  void LowController::set_robotstatusdata( std::array<MotorState,18>   data,NingImuData imudata,joydata joy_data)
  {
    motor_state_buffer_.SetData(data);
    imu_buffer_.SetData(imudata);
    joy_buffer_.SetData(joy_data);
  
  }
  void LowController::send_thread_func()
  {
    while(1)
    {
      const std::shared_ptr<const std::array<MotorCmd,18>> mc = motor_cmd_buffer_.GetData();
      if(mc)
      {
        RobotMotorCmd::MotorCmdArray cmdarray;
        cmdarray.motorcmds().resize(18);
        for(int i = 0;i<18;i++)
        {
            auto& cmd  = cmdarray.motorcmds()[i];
      
            cmd.pos()= mc->at(i).pos;
            cmd.vel() =mc->at(i).vel;
            cmd.tau() = mc->at(i).tau;
            cmd.kp() = mc->at(i).kp;
            cmd.kd() = mc->at(i).kd;
            cmd.motor_id() = mc->at(i).motor_id;
        }
        
        auto now = Clock::now();
        long long timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
        cmdarray.timestamp() = timestamp;
        ddswrapper.publishMotorCmdData(cmdarray);
        
      }
      std::this_thread::sleep_for(std::chrono::microseconds(2000));
    }
  }

  void LowController::process_thread_func()
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
  
  void LowController::set_joint(std::array<MotorCmd,18>  motorcmd)
  {
       
    motor_cmd_buffer_.SetData(motorcmd);
    
  }
  const std::array<MotorState,18>  LowController::get_joint_state()
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
  void LowController::setparameter(Command &cmd,bool* isfirst)
  {
      isfirstRecObs_ = isfirst;
      isfirstCompAct_ = *isfirstRecObs_;
      command_[0] = cmd.x;
      command_[1] = cmd.y;
      command_[2] = cmd.yaw;
  }
  
  bool LowController::updateStateEstimation()
  {
    vector_t jointPosnoarm(10),jointVelnoarm(10),jointTornoarm(10);

    quaternion_t quat;
    vector3_t angularVel, linearAccel;
    static int num=0;

    std::array<MotorState,18> joint_state ;
    const std::shared_ptr<const std::array<MotorState,18>> ms =motor_state_buffer_.GetData();
    if(ms)
    {
       for(int i=0;i<18;i++)
       {
           joint_state[i].pos  = ms->at(i).pos;
           joint_state[i].vel  = ms->at(i).vel;
           joint_state[i].tau  = ms->at(i).tau;
       }
    }
    NingImuData imudata;
    std::chrono::microseconds now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
   
    int j=0;
    
    for (size_t i = 0; i < actuatedDofNum_; ++i)
    {
      if((i>=4) &&(i<=8))
      {
        jointPosnoarm(j) = joint_state[i].pos ;
        jointVelnoarm(j) = joint_state[i].vel ;
        jointTornoarm(j) = joint_state[i].tau ;
        if(abs(jointPosnoarm(j)) == 12.5)
        {
          //LogError(" {} disconnect",jointNames[i]);
          //return false;
        }
        j++;
      }
      if((i>=13) &&(i<=17))
      {
        jointPosnoarm(j) = joint_state[i].pos ;
        jointVelnoarm(j) = joint_state[i].vel  ;
        jointTornoarm(j) = joint_state[i].tau  ;
        if(abs(jointPosnoarm(j)) == 12.5)
        {
          
          //LogError(" {} disconnect",jointNames[i]);
          //return false;
          
        }
        j++;
      }
    }
 

    const std::shared_ptr<const NingImuData> idata =imu_buffer_.GetData();
    if(idata)
    {
      for(int i=0;i<4;i++)
      {
        imudata.ori[i] = (*idata).ori[i];
      }
      for(int i=0;i<3;i++)
      {
        imudata.angular_vel[i] = (*idata).angular_vel[i];
        imudata.linear_acc[i] = (*idata).linear_acc[i];
      }
      for(int i=0;i<9;i++)
      {
        imudata.ori_cov[i] = (*idata).ori_cov[i];
        imudata.angular_vel_cov[i] = (*idata).angular_vel_cov[i];
        imudata.linear_acc_cov[i] = (*idata).linear_acc_cov[i];
      }
    
    //imudata = get_imu_data();
      for (size_t i = 0; i < 4; ++i)
      {
        quat.coeffs()(i) = imudata.ori[i];
      }
      for (size_t i = 0; i < 3; ++i)
      {
        angularVel(i) = imudata.angular_vel[i];
        linearAccel(i) = imudata.linear_acc[i];
        //printf("imudata.angular_vel %f  imudata.linear_acc  %f  i  %d\n",imudata.angular_vel[i],imudata.linear_acc[i],i);
      }


      propri_.jointPos = jointPosnoarm;
      propri_.jointVel = jointVelnoarm;
      propri_.baseAngVel = angularVel;

      vector3_t gravityVector(0, 0, -1);
      vector3_t zyx = quatToZyx(quat);
      matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();
      propri_.projectedGravity = inverseRot * gravityVector;
      propri_.baseEulerXyz = quatToXyz(quat);
    }
      double seconds = now.count();
      phase_ = seconds/1000000.0;
   
    
    return true;
  }
  void LowController::handleDefautMode()
  {
    //MotorCmd  motorcmd;
    std::array<MotorCmd,18> motorcmd;
    for (int j = 0; j < 18; j++)
    {
      motorcmd[j].kd=0.1;
      motorcmd[j].pos=0;
      motorcmd[j].kp=0;
      motorcmd[j].motor_id = j;
      motorcmd[j].vel =0;
      motorcmd[j].tau = 0;

      //controllerbase.set_joint(motorcmd);
      
    }
 
    set_joint(motorcmd);
 
  }
  void LowController::handleStandMode()
  {
    //MotorCmd motorcmd;
    std::array<MotorCmd,18> motorcmd;
    if (standPercent <= 1)
    {
      for (int j = 0; j < 18; j++)
      {
          scalar_t pos_des = lieJointAngles_[j] * (1 - standPercent) + standJointAngles_[j] * standPercent;
          if(j <4 ){
        
          motorcmd[j].pos=pos_des;
          motorcmd[j].kp=20;
          motorcmd[j].kd=0.1;
          motorcmd[j].motor_id = j;
          motorcmd[j].vel =0;
          motorcmd[j].tau = 0;
          
          //controllerbase.set_joint(motorcmd);
          //set_joint(motorcmd);
        }
        else if((j > 4) && (j<9)){
          
          motorcmd[j].pos=pos_des;
          motorcmd[j].kp=50;
          motorcmd[j].kd=1;
          motorcmd[j].motor_id =j;
          motorcmd[j].vel =0;
          motorcmd[j].tau = 0;
          
          //controllerbase.set_joint(motorcmd);
          //set_joint(motorcmd);
        }
        else if((j > 9) && (j<13)){

          motorcmd[j].pos=pos_des;
          motorcmd[j].kp=20;
          motorcmd[j].kd=0.1;
          motorcmd[j].motor_id = j;
          motorcmd[j].vel =0;
          motorcmd[j].tau = 0;
          
          //controllerbase.set_joint(motorcmd);
          //set_joint(motorcmd);
          
        }
        else{
          
          motorcmd[j].pos=pos_des;
          motorcmd[j].kp=50;
          motorcmd[j].kd=1;
          motorcmd[j].motor_id = j;
          motorcmd[j].vel =0;
          motorcmd[j].tau = 0;
          
          //controllerbase.set_joint(motorcmd);
          //set_joint(motorcmd);
        }
      }
      set_joint(motorcmd);
      standPercent += 1 / standDuration;
      standPercent = std::min(standPercent, scalar_t(1));
    }
    
  }
  void LowController::handleLieMode()
  {
    //MotorCmd motorcmd;
    std::array<MotorCmd,18> motorcmd;

    const std::shared_ptr<const std::array<MotorState,18>> ms =motor_state_buffer_.GetData();
    if(ms)
    {
      for(int i=0;i<18;i++)
      {
        if(abs(ms->at(i).pos) == 12.5)
        {
          //printf("joint %d disconnect\n",i);
          //return;
        }
      }
      if (standPercent <= 1)
      {
        for (int j = 0; j < 18; j++)
        {
          scalar_t pos_des = ms->at(j).pos * (1 - standPercent) + lieJointAngles_[j] * standPercent;
          if(j <4 ){
            motorcmd[j].pos=pos_des;
            motorcmd[j].kp=20;
            motorcmd[j].kd=0.1;
            motorcmd[j].motor_id = j;
            motorcmd[j].vel =0;
            motorcmd[j].tau = 0;
            
          }
          else if((j > 4) && (j<9)){
            motorcmd[j].pos=pos_des;
            motorcmd[j].kp=50;
            motorcmd[j].kd=1;
            motorcmd[j].motor_id = j;
            motorcmd[j].vel =0;
            motorcmd[j].tau = 0;
           
          }
          else if((j > 9) && (j<13)){

            motorcmd[j].pos=pos_des;
            motorcmd[j].kp=20;
            motorcmd[j].kd=0.1;
            motorcmd[j].motor_id = j;
            motorcmd[j].vel =0;
            motorcmd[j].tau = 0;
          
          }
          else {

            motorcmd[j].pos=pos_des;
            motorcmd[j].kp=50;
            motorcmd[j].kd=1;
            motorcmd[j].motor_id = j;
            motorcmd[j].vel =0;
            motorcmd[j].tau = 0;
            
          }
        }
        set_joint(motorcmd);
        standPercent += 1 / standDuration;
        standPercent = std::min(standPercent, double(1));
      }
      }
    
  }
 
  void LowController::process()
  {
    static int keyflag[14];
    if(initfinish == 0)
      return;    
    Command  cmd;
    auto now = Clock::now();
    long starttimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    const std::shared_ptr<const std::array<MotorState,18>> ms =motor_state_buffer_.GetData();
    if(ms)
    {
      const std::shared_ptr<const joydata> jdata =joy_buffer_.GetData();
      if(jdata)
      {
        memcpy(remote_data.button,&(*jdata).button[0],sizeof(remote_data.button));
        memcpy(remote_data.axes,&(*jdata).axes[0],sizeof(remote_data.axes));
        cmd.x =remote_data.axes[1];
        cmd.y=0;
        cmd.yaw =remote_data.axes[0];
     
        if((remote_data.button[9] == 1)&&(keyflag[9] == 0))
        {  
          
          if(!startcontrol)
          {
            startcontrol = true;
            standPercent = 0;
            mode_ = WorkMode::LIE;
            keyflag[9] =1;
            printf("TO LIE MODE\n");
            
          }else
          {
              startcontrol = false;
              mode_ = WorkMode::DEFAULT;
              keyflag[9] =1;
              printf("stop control\n");
          }
            
        }else if(remote_data.button[9] == 0)
          keyflag[9] =0;
        if((remote_data.button[10] == 1)&&(remote_data.button[2] == 1)&&(keyflag[10] == 0))
        {
          if (startcontrol == true)
          {
            if (mode_ != WorkMode::STAND)
            {
              standPercent = 0;
              mode_ = WorkMode::STAND;
              printf("STAND2LIE\n");
            }
            else if (mode_ == WorkMode::LIE)
            {
              standPercent = 0;
              mode_ = WorkMode::STAND;
              printf("LIE2STAND\n");
            }
            
          }  
        }else if(remote_data.button[10] == 0)
          keyflag[10] =0;
        if((remote_data.button[5] == 1)&&(remote_data.button[2] == 1)&&(keyflag[5] == 0))
        {
          
          if(mode_ == WorkMode::STAND )
          {
            standPercent = 0;
            isChangeMode_ = true;
            mode_ = WorkMode::USERWALK;
            keyflag[5] =1;
            printf("TO USERWALK MODE\n");
          }
          
        }else if(remote_data.button[5] == 0)
          keyflag[5] =0;
        if((remote_data.button[11] == 1)&&(keyflag[11] == 0)) 
        {
          if(mode_ == WorkMode::USERWALK)
          {
            isChangeMode_ = true;
            mode_ = WorkMode::STAND;
            printf("WALK2STAND\n");

          }else if (mode_ == WorkMode::DEFAULT)
          {
            standPercent = 0;
            printf("deftolie\n");
            isChangeMode_ = true;
            mode_ = WorkMode::LIE;
          }
        }
      }
    
      switch (mode_)
      {
        case WorkMode::STAND:
          handleStandMode();
          break;
        case WorkMode::LIE:
          handleLieMode();
          break;  
        case WorkMode::USERWALK:
          setparameter(cmd,&isChangeMode_);
          handleWalkMode();
          break;
        case WorkMode::DEFAULT:
          handleDefautMode();
          break;  
        default:
          printf("Unexpected mode encountered: %d\n",static_cast<int>(mode_));
          break;
      }
    
    }
  }
  bool LowController::handleWalkMode()
  {
    if(updateStateEstimation() == false)
       return false;
    // compute observation & actions
    if (count % robotconfig.controlCfg.decimation == 0)
    {
      count = 0;
      computeObservation();
      if(model_type == 0)
        computeActions();
      if(model_type == 1)
        computeActionsrknn();
      // limit action range
      scalar_t actionMin = -robotconfig.clipActions;
      scalar_t actionMax = robotconfig.clipActions;
      std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                    [actionMin, actionMax](scalar_t x)
                    { return std::max(actionMin, std::min(actionMax, x)); });
    }
    // set action
    int j =0;
   // MotorCmd motorcmd;
    std::array<MotorCmd,18> motorcmd;
    for (int i = 0; i < actionsSize_; i++)
    {
      if(i<5)
        j = i+4;
      if(i>4)
        j = i+8;
      std::string partName = jointNames[j];
      scalar_t pos_des = actions_[i] * robotconfig.controlCfg.actionScale + defaultJointAngles_(j);
      double stiffness = robotconfig.controlCfg.stiffness[partName]; // 根据关节名称获取刚度
      double damping = robotconfig.controlCfg.damping[partName]; // 根据关节名称获取阻尼
      // std::cout << "joint_name:" << partName << "kp:" << stiffness << " kd:" << damping << std::endl;
      motorcmd[j].pos= pos_des;
      motorcmd[j].kp= stiffness;
      motorcmd[j].kd= damping;
      motorcmd[j].motor_id = j;
      motorcmd[j].vel =0;
      motorcmd[j].tau = 0;
  
      lastActions_(i, 0) = actions_[i];
       
    }

    for (int i = 0; i < 8; i++)
    {
      if(i<4)
        j = i;
      if(i>=4)
        j = i+5;
      std::string partName = jointNames[j];
      scalar_t pos_des;    
      // std::array<MotorState,18> joint_state = controllerbase.get_joint_state();
      const std::shared_ptr<const std::array<MotorState,18>> ms =motor_state_buffer_.GetData();
      if(ms)
      {
         double cur_pos =  ms->at(j).pos - defaultJointAngles_(j); 
        pos_des = 0.75 * cur_pos + 0.25 *  defaultJointAngles_(j);
        
        double stiffness = robotconfig.controlCfg.stiffness[partName]; // 根据关节名称获取刚度
        double damping = robotconfig.controlCfg.damping[partName]; // 根据关节名称获取阻尼
        motorcmd[j].pos= pos_des;
        motorcmd[j].kp= stiffness;
        motorcmd[j].kd= damping;
        motorcmd[j].motor_id = j;
        motorcmd[j].vel =0;
        motorcmd[j].tau = 0;

      }
    
    }
    set_joint(motorcmd);
    count++;
    return true;
  
  }
   
  void LowController::onnxdatainit()
  {
    Ort::AllocatorWithDefaultOptions allocator;
   // ROS_INFO_STREAM("count: " << poliycfg->policySessionPtr->GetOutputCount());
    for (int i = 0; i < policySessionPtr->GetInputCount(); i++)
    {
      auto policyInputnamePtr = policySessionPtr->GetInputNameAllocated(i, allocator);
      policyInputNodeNameAllocatedStrings.push_back(std::move(policyInputnamePtr));
      policyInputNames_.push_back(policyInputNodeNameAllocatedStrings.back().get());
      // inputNames_.push_back(sessionPtr_->GetInputNameAllocated(i, allocator).get());
      policyInputShapes_.push_back(policySessionPtr->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
      std::vector<int64_t> policyShape = policySessionPtr->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
      std::cerr << "Policy Shape: [";
      for (size_t j = 0; j < policyShape.size(); ++j)
      {
          std::cout << policyShape[j];
          if (j != policyShape.size() - 1)
          {
              std::cerr << ", ";
          }
      }
      std::cout << "]" << std::endl;
    }
    
    for (int i = 0; i < policySessionPtr->GetOutputCount(); i++)
    {
      auto policyOutputnamePtr = policySessionPtr->GetOutputNameAllocated(i, allocator);
      policyOutputNodeNameAllocatedStrings.push_back(std::move(policyOutputnamePtr));
      policyOutputNames_.push_back(policyOutputNodeNameAllocatedStrings.back().get());
      // outputNames_.push_back(sessionPtr_->GetOutputNameAllocated(i, allocator).get());
      std::cout << policySessionPtr->GetOutputNameAllocated(i, allocator).get() << std::endl;
      policyOutputShapes_.push_back(policySessionPtr->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
      std::vector<int64_t> policyShape = policySessionPtr->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
      std::cerr << "Policy Shape: [";
      for (size_t j = 0; j < policyShape.size(); ++j)
      {
          std::cout << policyShape[j];
          if (j != policyShape.size() - 1)
          {
              std::cerr << ", ";
          }
      }
      std::cout << "]" << std::endl;
    }
    

  }
  bool LowController::loadModel(std::string modelpath)
  {
    std::string policyFilePath;
    std::string estFilePath;
      // create session
    Ort::SessionOptions sessionOptions;
    bool ret;
    onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
    sessionOptions.SetInterOpNumThreads(1);
    if(onnxEnvPrt_ == NULL)
    {
        printf("onnxEnvPrt_  is null\n");
        return false;
    }
        
    policyFilePath= modelpath+"/policy_user.onnx";
    printf("Load Onnx model from path : %s\n",policyFilePath.c_str());

    
    policySessionPtr = std::make_unique<Ort::Session>(*onnxEnvPrt_, policyFilePath.c_str(), sessionOptions);
    if(policySessionPtr == NULL)
    {
        printf("load run model failed\n");
        return false;
    }



    // get input and output info
    policyInputNames_.clear();
    policyOutputNames_.clear();
    policyInputShapes_.clear();
    policyOutputShapes_.clear();
    estSessionPtr = NULL;
    modelname="user";
    command_.resize(3);
    isfirstCompAct_ = true;
    isfirstRecObs_= NULL;
    count = 0;
    model_type = 0;
    onnxdatainit();
    ret = getmodelparam();
    initfinish = 1;
  
    printf("Load Onnx run model successfully !!!\n");
    return true;
  }
  

  bool LowController::getmodelparam()
  {
    char buf[256];
    getcwd(buf,sizeof(buf)); 
    std::string conpath=std::string(buf);
    std::string path=modelname;
    RobotCfg::InitState &initState = robotconfig.initState;
    RobotCfg::ControlCfg &controlCfg = robotconfig.controlCfg;
    RobotCfg::ObsScales &obsScales = robotconfig.obsScales;
    
    
    YAML::Node  acconfig = YAML::LoadFile(conpath+"/config/ning_user.yaml");

    int error = 0;
    
    initState.arm_l1_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["arm_l1_joint"].as<double>();
    initState.arm_l2_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["arm_l2_joint"].as<double>();
    initState.arm_l3_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["arm_l3_joint"].as<double>();
    initState.arm_l4_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["arm_l4_joint"].as<double>();
    initState.arm_r1_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["arm_r1_joint"].as<double>();
    initState.arm_r2_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["arm_r2_joint"].as<double>();
    initState.arm_r3_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["arm_r3_joint"].as<double>();
    initState.arm_r4_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["arm_r4_joint"].as<double>();
   
    initState.leg_l1_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["leg_l1_joint"].as<double>();
    initState.leg_l2_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["leg_l2_joint"].as<double>();
    initState.leg_l3_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["leg_l3_joint"].as<double>();
    initState.leg_l4_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["leg_l4_joint"].as<double>();
    initState.leg_l5_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["leg_l5_joint"].as<double>();

    initState.leg_r1_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["leg_r1_joint"].as<double>();
    initState.leg_r2_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["leg_r2_joint"].as<double>();
    initState.leg_r3_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["leg_r3_joint"].as<double>();
    initState.leg_r4_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["leg_r4_joint"].as<double>();
    initState.leg_r5_joint = acconfig[modelname]["init_state"]["default_joint_angle"]["leg_r5_joint"].as<double>();

   
    for(const auto& pair :acconfig[modelname]["control"]["stiffness"])
    {
      controlCfg.stiffness[pair.first.as<std::string>()] = pair.second.as<float>();
      
    }   
    
    for(const auto& pair :acconfig[modelname]["control"]["damping"])
    {
      controlCfg.damping[pair.first.as<std::string>()] =  pair.second.as<float>();  
        
    }     

    controlCfg.actionScale = acconfig[modelname]["control"]["action_scale"].as<float>();
    controlCfg.decimation = acconfig[modelname]["control"]["decimation"].as<int>();
    controlCfg.cycle_time = acconfig[modelname]["control"]["cycle_time"].as<float>();
 
    robotconfig.clipObs = acconfig[modelname]["normalization"]["clip_scales"]["clip_observations"].as<double>();
    robotconfig.clipActions = acconfig[modelname]["normalization"]["clip_scales"]["clip_actions"].as<double>();

    obsScales.linVel = acconfig[modelname]["normalization"]["obs_scales"]["lin_vel"].as<double>();
    obsScales.angVel = acconfig[modelname]["normalization"]["obs_scales"]["ang_vel"].as<double>();
    obsScales.dofPos = acconfig[modelname]["normalization"]["obs_scales"]["dof_pos"].as<double>();
    obsScales.dofVel = acconfig[modelname]["normalization"]["obs_scales"]["dof_vel"].as<double>();
    obsScales.heightMeasurements = acconfig[modelname]["normalization"]["obs_scales"]["height_measurements"].as<double>();
    obsScales.quat = acconfig[modelname]["normalization"]["obs_scales"]["quat"].as<double>();

    actionsSize_ = acconfig[modelname]["size"]["actions_size"].as<int>();
    observationSize_ = acconfig[modelname]["size"]["observations_size"].as<int>();
    stackSize_ = acconfig[modelname]["size"]["stack_size"].as<int>();

    scalez = acconfig[modelname]["axis_mappings"]["scalez"].as<float>();
    scaley = acconfig[modelname]["axis_mappings"]["scaley"].as<float>();
    scalex = acconfig[modelname]["axis_mappings"]["scalex"].as<float>();
 
    actions_.resize(actionsSize_);
 
    std::vector<scalar_t> defaultJointAngles{
        robotconfig.initState.arm_l1_joint, robotconfig.initState.arm_l2_joint, robotconfig.initState.arm_l3_joint, robotconfig.initState.arm_l4_joint, 
        robotconfig.initState.leg_l1_joint, robotconfig.initState.leg_l2_joint, robotconfig.initState.leg_l3_joint,
        robotconfig.initState.leg_l4_joint, robotconfig.initState.leg_l5_joint,
        robotconfig.initState.arm_r1_joint, robotconfig.initState.arm_r2_joint, robotconfig.initState.arm_r3_joint, robotconfig.initState.arm_r4_joint, 
        robotconfig.initState.leg_r1_joint,robotconfig.initState.leg_r2_joint, robotconfig.initState.leg_r3_joint,
        robotconfig.initState.leg_r4_joint,robotconfig.initState.leg_r5_joint};
 
    std::vector<scalar_t> walkdefaultJointAngles{
        robotconfig.initState.leg_l1_joint, robotconfig.initState.leg_l2_joint, robotconfig.initState.leg_l3_joint,
        robotconfig.initState.leg_l4_joint, robotconfig.initState.leg_l5_joint,
        robotconfig.initState.leg_r1_joint, robotconfig.initState.leg_r2_joint, robotconfig.initState.leg_r3_joint,
        robotconfig.initState.leg_r4_joint, robotconfig.initState.leg_r5_joint};     
    actuatedDofNum_ =  18;
     
     
    policyObservations_.resize(observationSize_ * stackSize_);
    
    std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);
    lastActions_.resize(actionsSize_);
    lastActions_.setZero();
    const int inputSize = stackSize_ * observationSize_;
    proprioHistoryBuffer_.resize(inputSize);
    defaultJointAngles_.resize(actuatedDofNum_);
    walkdefaultJointAngles_.resize(actionsSize_);
    for (int i = 0; i < actuatedDofNum_; i++)
    {
      defaultJointAngles_(i) = defaultJointAngles[i];
      //printf("defaultJointAngles[%d]  %f\n",i,modelcfg->defaultJointAngles_(i));
    }
    for (int i = 0; i < actionsSize_; i++)
    {
      walkdefaultJointAngles_(i) = walkdefaultJointAngles[i];
      //printf("defaultJointAngles[%d]  %f\n",i,modelcfg->defaultJointAngles_(i));
    }
    return true;
  }

  void LowController::computeActionsrknn()
  { 
    #ifdef RK3588
    rknn_input inputs[1];
    int ret;
    float* outdata=NULL;
    float* inputdata=NULL;
        
    inputdata =(float*)policyObservations_.data();
    //std::cout<<"policyObservations_.size() "<<policyObservations_.size()<< std::endl;
    for(int i=0;i<policyObservations_.size();i++)
    {
       inputfp16[i]=float_to_half(inputdata[i]);
    }
    memset(inputs,0,sizeof(inputs));
    inputs[0].index= input_attrs[0].index;
    inputs[0].type= input_attrs[0].type;
    inputs[0].size= input_attrs[0].size;
    inputs[0].fmt= input_attrs[0].fmt;
    inputs[0].pass_through= 0;
    inputs[0].buf = inputfp16.data();

    ret=rknn_inputs_set(ctx,1,inputs);

    ret=rknn_run(ctx,NULL);

    if(ret != 0)
    {
      printf("rknn run failed\n");
    }
    rknn_output outputs[io_num.n_output];
    memset(outputs,0,sizeof(outputs));
    for(int i=0;i<io_num.n_output;i++)
    {
      outputs[i].index = i;
      outputs[i].is_prealloc =0;
      outputs[i].want_float =1;
    }
    ret=rknn_outputs_get(ctx,io_num.n_output,outputs,NULL);
    //printf("output buf size is %d,n_elems %d\n",outputs[0].size,output_attrs[0].n_elems);
    outdata = (float*)outputs[0].buf;
     if (isfirstCompAct_){
      printf("rknn@@@@@@@@@@@@@@@@\n");
      for (int i = 0; i < policyObservations_.size(); ++i) {
        std::cout << policyObservations_[i] << " ";
        if ((i + 1) % observationSize_ == 0) {
            std::cout << std::endl;
        }
      }
      isfirstCompAct_ = false;
    }
   
    for(int i=0;i<output_attrs[0].n_elems;i++)
    {
       actions_[i] = outdata[i];
       //actions_[i] = half_to_float(outdata[i]);
        //std::cout<<"rknn actions "<<actions_[i]<< std::endl;
    }
    //printf("###############################################\n");*/
    ret = rknn_outputs_release(ctx,io_num.n_output,outputs);
    #endif
  }
  void LowController::computeActions()
  {
    std::vector<Ort::Value> policyInputValues;
    policyInputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, policyObservations_.data(),policyObservations_.size(),
                                                                          policyInputShapes_[0].data(), policyInputShapes_[0].size()));
     // run inference
    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues = policySessionPtr->Run(runOptions, policyInputNames_.data(), policyInputValues.data(), 1, policyOutputNames_.data(), 1);
    if (isfirstCompAct_)
    {
      for (int i = 0; i < policyObservations_.size(); ++i) {
        std::cout << policyObservations_[i] << " ";
        if ((i + 1) % observationSize_ == 0) {
            std::cout << std::endl;
        }
      }
      isfirstCompAct_ = false;
    }

    for (int i = 0; i < actionsSize_; i++)
    {
      actions_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
    }
    
    
  }

  void LowController::computeObservation()
  {
    
    std::atomic<scalar_t> comm_x;
    std::atomic<scalar_t> comm_y;
    
    
    RobotCfg::ObsScales &obsScales = robotconfig.obsScales;
    
    // command
    vector_t proprioObs(observationSize_);
    if (*isfirstRecObs_)
    {
      
      for (int i = 0; i < 40; i++)
      {
        proprioObs(i,0) = 0.0;
      }

      for (size_t i = 0; i < stackSize_; i++)
      {
        proprioHistoryBuffer_.segment(i * observationSize_, observationSize_) = proprioObs.cast<tensor_element_t>();
      }
      *isfirstRecObs_ = false;
    }
    
    vector_t command(5);
    comm_x = command_[0] * scalex;
    comm_y = command_[1] * scaley;
 
    double phase = phase_/0.64;
   

    command[0] = sin(2*M_PI*phase);
    command[1] = cos(2*M_PI*phase);
    command[2] = comm_x * obsScales.linVel;
    command[3] = comm_y * obsScales.linVel;
    command[4] = command_[2] * obsScales.angVel;

    
    
    vector_t actions(lastActions_);

    matrix_t commandScaler = Eigen::DiagonalMatrix<scalar_t, 3>(obsScales.linVel, obsScales.linVel, obsScales.angVel);

    
    proprioObs << command, // 5
        propri_.baseAngVel * obsScales.angVel,  // 3
        propri_.baseEulerXyz(0) * obsScales.quat,  // 1
        propri_.baseEulerXyz(1) * obsScales.quat,  // 1
        (propri_.jointPos - walkdefaultJointAngles_) * obsScales.dofPos,  // 10
        propri_.jointVel * obsScales.dofVel,  // 10
        actions;  // 10

    proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) =
        proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() - observationSize_);
    proprioHistoryBuffer_.tail(observationSize_) = proprioObs.cast<tensor_element_t>();
  

    //clang-format on

    for (size_t i = 0; i < (observationSize_ * stackSize_); i++){
      policyObservations_[i] = static_cast<tensor_element_t>(proprioHistoryBuffer_[i]);
      // if(i < observationSize_)
      // std::cout << i << "obs:::" << estObservations_[i] << std::endl;
    }
    // Limit observation range
    scalar_t obsMin = -robotconfig.clipObs;
    scalar_t obsMax = robotconfig.clipObs;
    std::transform(policyObservations_.begin(), policyObservations_.end(), policyObservations_.begin(),
                  [obsMin, obsMax](scalar_t x)
                  { return std::max(obsMin, std::min(obsMax, x)); });

   
    
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
  legged::LowController lowcontroller;
  lowcontroller.init(legged::ControlMode::LOWMODE);
  #ifdef RK3588
  lowcontroller.loadrknnmodel(path+"/policy");      
  #else
  lowcontroller.loadModel(path+"/policy");  
  #endif
  while(1)
  {
    usleep(10);
  }
  return 0;
    
}
