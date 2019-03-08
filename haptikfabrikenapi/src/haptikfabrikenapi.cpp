#include "haptikfabrikenapi.h"
#include "fshapticdevicethread.h"
#include "fsusbhapticdevicethread.h"
#include "fsdaqhapticdevicethread.h"

#include <sstream>
#include <iomanip>

haptikfabriken::HaptikfabrikenInterface::HaptikfabrikenInterface(bool wait_for_next_message, haptikfabriken::Kinematics::configuration c)
{
    fsthread = new FsDAQHapticDeviceThread(wait_for_next_message,c);
}

int haptikfabriken::HaptikfabrikenInterface::open()
{
    return fsthread->open();
}

void haptikfabriken::HaptikfabrikenInterface::close()
{
    fsthread->close();
}

std::string haptikfabriken::HaptikfabrikenInterface::getErrorCode()
{
    return fsthread->getErrorCode();
}

void haptikfabriken::HaptikfabrikenInterface::getEnc(int a[])
{
    fsthread->getEnc(a);
}

haptikfabriken::fsVec3d haptikfabriken::HaptikfabrikenInterface::getBodyAngles()
{
    return fsthread->getBodyAngles();
}

void haptikfabriken::HaptikfabrikenInterface::getLatestCommandedMilliamps(int ma[])
{
    fsthread->getLatestCommandedMilliamps(ma);
}

int haptikfabriken::HaptikfabrikenInterface::getNumSentMessages()
{
    return fsthread->getNumSentMessages();
}

int haptikfabriken::HaptikfabrikenInterface::getNumReceivedMessages()
{
    return fsthread->getNumReceivedMessages();
}

haptikfabriken::fsVec3d haptikfabriken::HaptikfabrikenInterface::getPos()
{
    return fsthread->getPos();
}

haptikfabriken::fsRot haptikfabriken::HaptikfabrikenInterface::getRot()
{
    return fsthread->getRot();
}

void haptikfabriken::HaptikfabrikenInterface::setForce(haptikfabriken::fsVec3d f)
{
    fsthread->setForce(f);
}

void haptikfabriken::HaptikfabrikenInterface::setCurrent(haptikfabriken::fsVec3d amps)
{
    fsthread->setCurrent(amps);
}

std::string haptikfabriken::toString(const haptikfabriken::fsVec3d &r)
{

   std::stringstream ss;
   ss.precision(3);
   ss.setf(std::ios::fixed);
   ss << std::setw(6) << r.m_x << ", " << std::setw(6) << r.m_y << ", " << std::setw(6) << r.m_z;
   return ss.str();

}

std::string haptikfabriken::toString(const haptikfabriken::fsRot &r)
{

   std::stringstream ss;
   ss.precision(3);
   ss.setf(std::ios::fixed);
   ss << std::setw(6) << r.m[0][0] << ", " << std::setw(6) << r.m[0][1] << ", " << std::setw(6) << r.m[0][2] << ",\n";
   ss << std::setw(6) << r.m[1][0] << ", " << std::setw(6) << r.m[1][1] << ", " << std::setw(6) << r.m[1][2] << ",\n";
   ss << std::setw(6) << r.m[2][0] << ", " << std::setw(6) << r.m[2][1] << ", " << std::setw(6) << r.m[2][2] << "\n";
   return ss.str();

}
