#ifndef PYRO_INS_H
#define PYRO_INS_H

#include "pyro_core_config.h"
#include "pyro_core_def.h"
#include "BMI088_driver.h"
#include "FreeRTOS.h"
#include "task.h"
namespace pyro{

class QuaternionEKF_t;

enum class imu_source_t :uint8_t{
    BMI088      = 0,
    DM_IMU_L1   = 1
};

struct ins_key{
    uint8_t imu_id;
    imu_source_t imu_source;
    bool operator<(const ins_key &rhs) const
    {
        if (imu_source != rhs.imu_source)
            return imu_source < rhs.imu_source;
        return imu_id < rhs.imu_id;
    }
};
struct ins_config_t
{
    enum class imu_direct_t : uint8_t
    {
        DIRECT_1 = 1,
        DIRECT_2 = 2,
        DIRECT_3 = 3,
        DIRECT_4 = 4,
        DIRECT_5 = 5,
        DIRECT_6 = 6,
        DIRECT_7 = 7,
        DIRECT_8 = 8
    };


    imu_direct_t direct = imu_direct_t::DIRECT_1;

    bool calibrate      = false;
    float gx_offset     = 0.0f;
    float gy_offset     = 0.0f;
    float gz_offset     = 0.0f;
    float g_norm        = 9.80665f;
};


// template <imu_source_t source>
class ins_drv_t {
    


private:
    IMU_Data_t imu_data;
    //quaternion=q0+i*q1+j*q2+k*q3
    float _q[4];
    float _gyro_b[3];
    float _gyro_n[3];
    float _acc_b[3];
    float _acc_n[3];
    float _angle_b[3];
    float _angle_n[3];

    float _dt;
    float _t;
    uint32_t _dwt_cnt;

    // 动态轴向映射
    uint8_t _imu_x_idx;
    uint8_t _imu_y_idx;
    uint8_t _imu_z_idx;
    bool _x_neg;
    bool _y_neg;
    bool _z_neg;

    ins_config_t _config;
    ins_key _key;
    imu_base_t* _imu_drv;

    

    TaskHandle_t _ins_task_handle;

    explicit ins_drv_t(ins_key key) : _quaternion_ekf(nullptr), _ins_task_handle(nullptr),
                                      _imu_drv(nullptr), _key(key) {

    }

    void __ins_task();
    static void __static_ins_task(void *argument);
    
public:
    static ins_drv_t* get_instance(ins_key key);
    status_t init(const ins_config_t &config, imu_base_t* imu_drv);
    status_t get_angles_b(float *yaw, float *pitch, float *roll);
    status_t get_angles_n(float *yaw, float *pitch, float *roll);
    status_t get_rads_b(float* rad_yaw, float* rad_pitch, float* rad_roll);
    status_t get_rads_n(float* rad_yaw, float* rad_pitch, float* rad_roll);
    status_t get_gyro_b(float* g_yaw, float* g_pitch, float* g_roll);
    status_t get_gyro_n(float* g_yaw, float* g_pitch, float* g_roll);
    status_t get_accel_b(float* accel_x, float* accel_y, float* accel_z);
    status_t get_accel_n(float* accel_x, float* accel_y, float* accel_z);
    status_t get_quaternion(float* q0, float* q1, float* q2, float* q3);
    QuaternionEKF_t* _quaternion_ekf;
};

}
#endif
