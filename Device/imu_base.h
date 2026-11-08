#ifndef IMU_BASE_H
#define IMU_BASE_H

#include "pyro_core_def.h"

struct IMU_Data_t
{
    float Accel[3]{};
    float Gyro[3]{};

    float TempWhenCali = 0.0f;
    float Temperature = 0.0f;

    float AccelScale = 0.0f;
    float GyroOffset[3]{};

    float gNorm = 0.0f;
};

struct imu_config_t
{
    uint8_t calibrate = 0;
    float gx_offset   = 0.0f;
    float gy_offset   = 0.0f;
    float gz_offset   = 0.0f;
    float g_norm      = 9.80665f;
};

class imu_base_t
{
public:
    virtual ~imu_base_t() = default;

    virtual pyro::status_t imu_init(imu_config_t config) = 0;
    virtual void imu_update()                         = 0;
    virtual void get_imu_data(IMU_Data_t &imu_data)   = 0;
};

#endif
