#ifndef DDSWrapper_H
#define DDSWrapper_H

#include <features.h>
#include <dds/dds.hpp>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>

#include "Robot_motorcmd.hpp"
#include "Robot_status.hpp"
#include "Robot_setmode.hpp"
namespace legged
{
  class DDSWrapper
  {
    public:
        using RobotStatusCallback = std::function<void(const RobotStatus::StatusData&)>;
        DDSWrapper(int domain_id = 0);
        ~DDSWrapper();
         void publishMotorCmdData(const RobotMotorCmd::MotorCmdArray&  state);
        void publishModeData(const RobotSetMode::SetMode& data);
        
        void subscribeRobotStatus(RobotStatusCallback callback);

    private:
        dds::domain::DomainParticipant participant_;
        dds::pub::Publisher publisher_;
        dds::sub::Subscriber subscriber_;
        

        // motor state
        dds::topic::Topic<RobotStatus::StatusData> robotstatus_topic_;
        dds::sub::DataReader<RobotStatus::StatusData> robotstatus_reader_;
        RobotStatusCallback robotstatus_callback_;
        std::thread robot_status_listener_thread_;
        std::atomic<bool> robot_status_listener_running_;
        std::mutex RobotStatus_callback_mutex_;

       

        //motorcmd
        dds::topic::Topic<RobotMotorCmd::MotorCmdArray> motorcmd_topic_;
        dds::pub::DataWriter<RobotMotorCmd::MotorCmdArray> motorcmd_writer_;

        
        //setmode
        dds::topic::Topic<RobotSetMode::SetMode> mode_topic_;
        dds::pub::DataWriter<RobotSetMode::SetMode> mode_writer_;


        //qos配置
        dds::sub::qos::DataReaderQos getRobotStatusQos();

        dds::pub::qos::DataWriterQos getMotorCmdQos();
        dds::pub::qos::DataWriterQos getModeQos();
        void init();
        void robotstatusListener();

        void wait_for_reader(dds::pub::DataWriter<RobotSetMode::SetMode>& writer);
     
  };




}
#endif