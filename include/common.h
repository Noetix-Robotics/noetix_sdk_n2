#ifndef COMMON_H
#define COMMON_H

// #include <Eigen/Core>
//#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <atomic>
#include <eigen3/Eigen/Dense>
namespace legged
{
using scalar_t = double;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;
using Vecjoint = Eigen::VectorXd;
using Clock = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double>;
using tensor_element_t = float;

template <typename T>
using feet_array_t = std::array<T, 4>;
using contact_flag_t = feet_array_t<bool>;
    
    enum class ControlMode : uint8_t
    {
      LOWMODE,
      HIGHMODE,
      USERMODE,
      DEFAULT
    };
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
  struct JointState
  {
    double arm_l1_joint;
    double arm_l2_joint;
    double arm_l3_joint;
    double arm_l4_joint;
    double leg_l1_joint;
    double leg_l2_joint;
    double leg_l3_joint;
    double leg_l4_joint;
    double leg_l5_joint;
    double arm_r1_joint;
    double arm_r2_joint;
    double arm_r3_joint;
    double arm_r4_joint;
    double leg_r1_joint;
    double leg_r2_joint;
    double leg_r3_joint;
    double leg_r4_joint;
    double leg_r5_joint;
  };

  


  struct Proprioception
  {
    vector_t jointPos;
    vector_t jointVel;
    vector3_t baseAngVel;
    vector3_t baseEulerXyz;
    vector3_t projectedGravity;
  };

  struct Command
  {
    std::atomic<double> x;
    std::atomic<double> y;
    std::atomic<double> yaw;
  };
  struct MotorState
  {
    double pos;
    double vel;
    double tau;
    uint16_t motor_id;
    uint8_t error;
    int temperature;

  };
  struct MotorCmd
  {
    double pos;
    double vel;
    double tau;
    double kp;
    double kd;
    uint16_t motor_id;
  };
  struct NingImuData
  {
    double ori[4];
    double ori_cov[9];
    double angular_vel[3];
    double angular_vel_cov[9];
    double linear_acc[3];
    double linear_acc_cov[9];
  };
  struct joydata{
    double axes[2];//horz,vert;
    int button[14];
};


}
#define sleep_ms(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))
#define sleep_us(x) std::this_thread::sleep_for(std::chrono::microseconds(x))
#define get_time_us()                               \
    std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
#endif