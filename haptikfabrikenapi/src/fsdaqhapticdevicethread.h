#ifndef FSDAQHAPTICDEVICETHREAD_H
#define FSDAQHAPTICDEVICETHREAD_H

#include "fshapticdevicethread.h"

namespace haptikfabriken {



class FsDAQHapticDeviceThread : public FsHapticDeviceThread
{
public:
    FsDAQHapticDeviceThread(bool wait_for_next_message=false,
                         Kinematics::configuration c=Kinematics::configuration::woodenhaptics_v2015()):
        FsHapticDeviceThread::FsHapticDeviceThread(wait_for_next_message,c){}

    void thread();
    void close();



private:

};
}
#endif // FSDAQHAPTICDEVICETHREAD_H
