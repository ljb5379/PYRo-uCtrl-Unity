#ifndef IMU_BASE_H
#define IMU_BASE_H
#include "pyro_core_def.h"

using namespace pyro;

typedef struct
{
    float Accel[3];

    float Gyro[3];

    float TempWhenCali;
    float Temperature;

    float AccelScale;
    float GyroOffset[3];

    float gNorm;
} IMU_Data_t;

typedef struct
{
    uint8_t calibrate;
    float gx_offset;
    float gy_offset;
    float gz_offset;
    float g_norm;
} imu_config_t;



class imu_base_t
{
public:
    // imu_base_t() {}
    // virtual ~imu_base_t() {}

    virtual status_t imu_init(imu_config_t _config) = 0;
    virtual void imu_update() = 0;

    virtual void get_imu_data(IMU_Data_t &imu_data) = 0;
    private:
        
};

#endif
