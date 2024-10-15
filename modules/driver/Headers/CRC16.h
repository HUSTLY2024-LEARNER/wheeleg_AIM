// CICECOO 24.8.5

#include <cstdint>
#include "driver.hpp"

namespace DRIVER {
    void addCRC16(unsigned char *msg);
    bool verifyCRC16(SerialReadData::RobotStatus *data);
    bool verifyCRC16(SerialReadData::IMU_Flag *data);
}