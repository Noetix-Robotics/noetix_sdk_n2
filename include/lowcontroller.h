#ifndef LowController_H
#define LowController_H
#include "common.h"

namespace noetix {

class DDSWrapper;

class LowController {
      public:
        ~LowController();
        static LowController *Instance();
        bool init();

        void set_joint(std::array<MotorCmd, 18> motorcmd);

        void
        subscribe_robot_hardware_status(RobotHardwareStatusCallback callback);

      protected:
        void send_thread_func();

      private:
        std::unique_ptr<DDSWrapper> ddswrapper;
};
} // namespace noetix
#endif
