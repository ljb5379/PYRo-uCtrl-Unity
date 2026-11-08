#include "pyro_ins.h"
#include "pyro_dwt_drv.h"
#include "QuaternionEKF.h"
#include <map>
#include "DM_IMU_L1.h"
using namespace pyro;

#define X            0
#define Y            1
#define Z            2

namespace pyro{
std::map<ins_key, ins_drv_t *> ins_map;

IMU_Data_t *g_imu_data;//用于观测，可删

ins_drv_t *ins_drv_t::get_instance(ins_key key)
{
    if (ins_map.find(key) == ins_map.end())
    {
        ins_map[key] = new ins_drv_t(key);
    }
    return ins_map[key];
        
}



status_t ins_drv_t::init(const ins_config_t &config, imu_base_t* imu_drv)
{
    if (imu_drv == nullptr)
    {
        return PYRO_PARAM_ERROR;
    }
    if (_ins_task_handle != nullptr)
    {
        return PYRO_ALREADY_INIT;
    }

    _config = config;
    _imu_drv = imu_drv;

    // 解析轴向方向映射
    switch (_config.direct)
    {
        case ins_config_t::imu_direct_t::DIRECT_1:
            _imu_x_idx = 0; _imu_y_idx = 1; _imu_z_idx = 2;
            _x_neg = false; _y_neg = false; _z_neg = false;
            break;
        case ins_config_t::imu_direct_t::DIRECT_2:
            _imu_x_idx = 1; _imu_y_idx = 0; _imu_z_idx = 2;
            _x_neg = false; _y_neg = true;  _z_neg = false;
            break;
        case ins_config_t::imu_direct_t::DIRECT_3:
            _imu_x_idx = 0; _imu_y_idx = 1; _imu_z_idx = 2;
            _x_neg = true;  _y_neg = true;  _z_neg = false;
            break;
        case ins_config_t::imu_direct_t::DIRECT_4:
            _imu_x_idx = 1; _imu_y_idx = 0; _imu_z_idx = 2;
            _x_neg = true;  _y_neg = false; _z_neg = false;
            break;
        case ins_config_t::imu_direct_t::DIRECT_5:
            _imu_x_idx = 0; _imu_y_idx = 1; _imu_z_idx = 2;
            _x_neg = false; _y_neg = true;  _z_neg = true;
            break;
        case ins_config_t::imu_direct_t::DIRECT_6:
            _imu_x_idx = 1; _imu_y_idx = 0; _imu_z_idx = 2;
            _x_neg = true;  _y_neg = true;  _z_neg = true;
            break;
        case ins_config_t::imu_direct_t::DIRECT_7:
            _imu_x_idx = 0; _imu_y_idx = 1; _imu_z_idx = 2;
            _x_neg = true;  _y_neg = false; _z_neg = true;
            break;
        case ins_config_t::imu_direct_t::DIRECT_8:
            _imu_x_idx = 1; _imu_y_idx = 0; _imu_z_idx = 2;
            _x_neg = false; _y_neg = false; _z_neg = true;
            break;
        default:
            return PYRO_PARAM_ERROR;
    }
    // 初始化EKF
    if (_quaternion_ekf == nullptr)
    {
        _quaternion_ekf = new QuaternionEKF_t();
        if (_quaternion_ekf == nullptr)
        {
            return PYRO_NO_MEMORY;
        }
    }

    // 观测数据
    g_imu_data = &imu_data;

    // 初始化IMU
    imu_config_t imu_config{};
    imu_config.calibrate = _config.calibrate ? 1u : 0u;
    imu_config.gx_offset = _config.gx_offset;
    imu_config.gy_offset = _config.gy_offset;
    imu_config.gz_offset = _config.gz_offset;
    imu_config.g_norm    = _config.g_norm;

    const status_t imu_status = _imu_drv->imu_init(imu_config);
    if (imu_status != PYRO_OK && imu_status != PYRO_WARNING)
    {
        return imu_status;
    }

    // 初始化任务
    if (xTaskCreate(__static_ins_task, "ins_task", 512, this, 2,
                    &_ins_task_handle) != pdPASS)
    {
        _ins_task_handle = nullptr;
        return PYRO_ERROR;
    }

    return imu_status;
}

void ins_drv_t::__static_ins_task(void *argument)
{   
    auto* instance = static_cast<ins_drv_t*>(argument);
    instance->__ins_task();
}

void ins_drv_t::__ins_task()
{   
    _dwt_cnt = 0;
    _quaternion_ekf->QuaternionEKF_Init(10, 0.001, 10000000, 0.9996, 0.004f);
    //IMU_QuaternionEKF_Init(10, 0.001, 10000000, 0.9996, 0.004f);

    while (1)
    {
        _dt = dwt_drv_t::get_delta_t(&_dwt_cnt);
        _t += _dt;
//调用驱动指针获取数
        _imu_drv->imu_update();
        _imu_drv->get_imu_data(imu_data);
        
        if (_x_neg)
        {
            _acc_b[X]  = -imu_data.Accel[_imu_x_idx];
            _gyro_b[X] = -imu_data.Gyro[_imu_x_idx];
        }
        else
        {
            _acc_b[X]  = imu_data.Accel[_imu_x_idx];
            _gyro_b[X] = imu_data.Gyro[_imu_x_idx];
        }
        if (_y_neg)
        {
            _acc_b[Y]  = -imu_data.Accel[_imu_y_idx];
            _gyro_b[Y] = -imu_data.Gyro[_imu_y_idx];
        }
        else
        {
            _acc_b[Y]  = imu_data.Accel[_imu_y_idx];
            _gyro_b[Y] = imu_data.Gyro[_imu_y_idx];
        }
        if (_z_neg)
        {
            _acc_b[Z]  = -imu_data.Accel[_imu_z_idx];
            _gyro_b[Z] = -imu_data.Gyro[_imu_z_idx];
        }
        else
        {
            _acc_b[Z]  = imu_data.Accel[_imu_z_idx];
            _gyro_b[Z] = imu_data.Gyro[_imu_z_idx];
        }



        _quaternion_ekf->QuaternionEKF_Update(_gyro_b[X], _gyro_b[Y], _gyro_b[Z], _acc_b[X],
                                 _acc_b[Y], _acc_b[Z], _dt);
       memcpy(_q, _quaternion_ekf->q, sizeof(_quaternion_ekf->q));


        _angle_n[X] = _quaternion_ekf->Roll;
        _angle_n[Y] = _quaternion_ekf->Pitch;
        _angle_n[Z] = _quaternion_ekf->Yaw;

        vTaskDelay(1);
    }
}

status_t ins_drv_t::get_angles_b(float *yaw, float *pitch, float *roll)
{
    if (roll == nullptr || pitch == nullptr || yaw == nullptr)
    {
        return PYRO_ERROR;
    }
    *roll  = _angle_n[X];
    *pitch = _angle_n[Y];
    *yaw   = _angle_n[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_angles_n(float *yaw, float *pitch, float *roll)
{
    if (roll == nullptr || pitch == nullptr || yaw == nullptr)
    {
        return PYRO_ERROR;
    }
    *roll  = _angle_n[X];
    *pitch = _angle_n[Y];
    *yaw   = _angle_n[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_rads_b(float *rad_yaw, float *rad_pitch,
                               float *rad_roll)
{
    if (rad_roll == nullptr || rad_pitch == nullptr || rad_yaw == nullptr)
    {
        return PYRO_ERROR;
    }
    *rad_roll  = _angle_n[X] * PI / 180.0f;
    *rad_pitch = _angle_n[Y] * PI / 180.0f;
    *rad_yaw   = _angle_n[Z] * PI / 180.0f;
    return PYRO_OK;
}

status_t ins_drv_t::get_rads_n(float *rad_yaw, float *rad_pitch,
                               float *rad_roll)
{
    if (rad_roll == nullptr || rad_pitch == nullptr || rad_yaw == nullptr)
    {
        return PYRO_ERROR;
    }
    *rad_roll  = _angle_n[X] * PI / 180.0f;
    *rad_pitch = _angle_n[Y] * PI / 180.0f;
    *rad_yaw   = _angle_n[Z] * PI / 180.0f;
    return PYRO_OK;
}

status_t ins_drv_t::get_gyro_b(float *g_yaw, float *g_pitch, float *g_roll)
{
    if (g_yaw == nullptr || g_pitch == nullptr || g_roll == nullptr)
    {
        return PYRO_ERROR;
    }
    *g_roll  = _gyro_b[X];
    *g_pitch = _gyro_b[Y];
    *g_yaw   = _gyro_b[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_gyro_n(float *g_yaw, float *g_pitch, float *g_roll)
{
    if (g_yaw == nullptr || g_pitch == nullptr || g_roll == nullptr)
    {
        return PYRO_ERROR;
    }
    *g_roll  = _gyro_b[X];
    *g_pitch = _gyro_b[Y];
    *g_yaw   = _gyro_b[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_accel_b(float *accel_x, float *accel_y, float *accel_z)
{
    if (accel_x == nullptr || accel_y == nullptr || accel_z == nullptr)
    {
        return PYRO_ERROR;
    }
    *accel_x = _acc_b[X];
    *accel_y = _acc_b[Y];
    *accel_z = _acc_b[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_accel_n(float *accel_x, float *accel_y, float *accel_z)
{
    if (accel_x == nullptr || accel_y == nullptr || accel_z == nullptr)
    {
        return PYRO_ERROR;
    }
    *accel_x = _acc_n[X];
    *accel_y = _acc_n[Y];
    *accel_z = _acc_n[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_quaternion(float *q0, float *q1, float *q2, float *q3)
{
    if (q0 == nullptr || q1 == nullptr || q2 == nullptr || q3 == nullptr)
    {
        return PYRO_ERROR;
    }
    *q0 = _q[0];
    *q1 = _q[1];
    *q2 = _q[2];
    *q3 = _q[3];
    return PYRO_OK;
}

}
