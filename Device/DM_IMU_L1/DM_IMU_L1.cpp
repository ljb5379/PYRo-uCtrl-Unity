#include "DM_IMU_L1.h"

namespace pyro
{

static float gyroDiff[3], gNormDiff;//校准用
uint8_t caliOffset = 1;
int16_t caliCount = 0;

static IMU_Data_t*  dm_imu_cali_imu_data;

constexpr uint8_t FRAME_ACCEL      = 0x01;
constexpr uint8_t FRAME_GYRO       = 0x02;
constexpr uint8_t FRAME_EULER      = 0x03;
constexpr uint8_t FRAME_QUATERNION = 0x04;

// 说明书附录 C 表 B-2 的解码范围。
constexpr float ACCEL_MIN = -235.2f;
constexpr float ACCEL_MAX = 235.2f;
constexpr float GYRO_MIN  = -34.88f;
constexpr float GYRO_MAX  = 34.88f;
constexpr float PITCH_MIN = -90.0f;
constexpr float PITCH_MAX = 90.0f;
constexpr float ANGLE_MIN = -180.0f;
constexpr float ANGLE_MAX = 180.0f;

uint16_t read_u16_le(const uint8_t *data)
{
    return static_cast<uint16_t>(static_cast<uint16_t>(data[0]) |
                                 (static_cast<uint16_t>(data[1]) << 8));
}

float raw_to_float(const uint16_t raw, const float min, const float max)
{
    return static_cast<float>(raw) * (max - min) / 65535.0f + min;
}

float raw_to_quaternion(const uint16_t raw)
{
    return static_cast<float>(raw) * 2.0f / 16383.0f - 1.0f;
}

dm_imu_l1_drv_t::dm_imu_l1_drv_t(uint8_t slave_id, uint8_t master_id,
                                 bsp_can::which_can which)
    : _slave_id(slave_id), _master_id(master_id), _can_drv(nullptr),
      _msg_buffer(nullptr), _data{}, _last_rx_tick(0), _is_online(false),
      _last_frame_type(0)
{
    _can_drv = bsp_can::get_can(which);
}

dm_imu_l1_drv_t::~dm_imu_l1_drv_t() = default;

status_t dm_imu_l1_drv_t::init()
{
    if (nullptr != _msg_buffer)
        return PYRO_ALREADY_INIT;
    if (nullptr == _can_drv)
        return PYRO_ERROR;

    _msg_buffer = new can_msg_buffer_t(_master_id);
    if (nullptr == _msg_buffer)
        return PYRO_NO_MEMORY;

    const status_t ret = _can_drv->register_rx_msg(_msg_buffer);
    if (PYRO_OK != ret)
    {
        delete _msg_buffer;
        _msg_buffer = nullptr;
        return ret;
    }

    return PYRO_OK;
}

status_t dm_imu_l1_drv_t::update_feedback()
{
    if(dwt_drv_t::get_timeline_s() - _last_rx_tick > 1.0f)
    {
        _is_online = false;
    }

    if (nullptr == _msg_buffer)
        return PYRO_ERROR;

    std::array<uint8_t, 8> frame{};
    if (!_msg_buffer->get_data(frame)){
        return PYRO_BUSY;
    }

    if(_msg_buffer->is_fresh()){
        _msg_buffer->mark_read();
        _is_online = true;
        _last_rx_tick = dwt_drv_t::get_timeline_s();
    }
    return _decode(frame);
}


status_t dm_imu_l1_drv_t::get_data(dm_imu_l1_data_t &data) const
{
    data = _data;
    return PYRO_OK;
}

bool dm_imu_l1_drv_t::is_online() const
{
    return _is_online;
}

// void dm_imu_l1_drv_t::read_data(IMU_Data_t *imu_data){

//     imu_data->Accel[0] = _data.accel[0];
//     imu_data->Accel[1] = _data.accel[1];
//     imu_data->Accel[2] = _data.accel[2];
//     imu_data->Gyro[0] = _data.gyro[0];
//     imu_data->Gyro[1] = _data.gyro[1];
//     imu_data->Gyro[2] = _data.gyro[2];
//     imu_data->Temperature = _data.temperature;

// }
status_t dm_imu_l1_drv_t::_decode(const std::array<uint8_t, 8> &frame)
{
    _last_frame_type = frame[0];

    switch (frame[0])
    {
        case FRAME_ACCEL:
        {
            const float x = raw_to_float(read_u16_le(&frame[2]), ACCEL_MIN,
                                         ACCEL_MAX);
            const float y = raw_to_float(read_u16_le(&frame[4]), ACCEL_MIN,
                                         ACCEL_MAX);
            const float z = raw_to_float(read_u16_le(&frame[6]), ACCEL_MIN,
                                         ACCEL_MAX);

            _data.Accel[0] = x * _data.AccelScale;
            _data.Accel[1] = y * _data.AccelScale;
            _data.Accel[2] = z * _data.AccelScale;
            _data.Temperature = static_cast<float>(frame[1]);
            break;
        }
        case FRAME_GYRO:
        {
         
            
            const float x = raw_to_float(read_u16_le(&frame[2]), GYRO_MIN,
                                         GYRO_MAX);
            const float y = raw_to_float(read_u16_le(&frame[4]), GYRO_MIN,
                                         GYRO_MAX);
            const float z = raw_to_float(read_u16_le(&frame[6]), GYRO_MIN,
                                         GYRO_MAX);
            if (caliOffset)
            { 
            _data.Gyro[0] = x - _data.GyroOffset[0];
            _data.Gyro[1] = y - _data.GyroOffset[1];
            _data.Gyro[2] = z - _data.GyroOffset[2];
            }
            else
            {
            _data.Gyro[0] = x;
            _data.Gyro[1] = y;
            _data.Gyro[2] = z;   
            }
            break;
        }
        case FRAME_EULER:
        {
            // CAN 帧顺序为 pitch / yaw / roll。
            const float pitch = raw_to_float(read_u16_le(&frame[2]),
                                             PITCH_MIN, PITCH_MAX);
            const float yaw   = raw_to_float(read_u16_le(&frame[4]),
                                             ANGLE_MIN, ANGLE_MAX);
            const float roll  = raw_to_float(read_u16_le(&frame[6]),
                                             ANGLE_MIN, ANGLE_MAX);
            
            _data.roll  = roll;
            _data.pitch = pitch;
            _data.yaw   = yaw;
            break;
        }
        case FRAME_QUATERNION:
        {
            // W/X/Y/Z 各 14 bit，从 DATA[1] 起按高位在前连续排列。
            const uint16_t w = static_cast<uint16_t>(
                (static_cast<uint16_t>(frame[1]) << 6) | (frame[2] >> 2));
            const uint16_t x = static_cast<uint16_t>(
                ((frame[2] & 0x03U) << 12) |
                (static_cast<uint16_t>(frame[3]) << 4) | (frame[4] >> 4));
            const uint16_t y = static_cast<uint16_t>(
                ((frame[4] & 0x0FU) << 10) |
                (static_cast<uint16_t>(frame[5]) << 2) | (frame[6] >> 6));
            const uint16_t z = static_cast<uint16_t>(
                ((frame[6] & 0x3FU) << 8) | frame[7]);

            const float qw = raw_to_quaternion(w);
            const float qx = raw_to_quaternion(x);
            const float qy = raw_to_quaternion(y);
            const float qz = raw_to_quaternion(z);

            _data.quaternion[0] = qw;
            _data.quaternion[1] = qx;
            _data.quaternion[2] = qy;
            _data.quaternion[3] = qz;
            break;
        }
        default:
            return PYRO_WARNING;
    }
    _last_rx_tick = dwt_drv_t::get_timeline_s();

    return PYRO_OK;
} 
status_t dm_imu_l1_drv_t::imu_init(imu_config_t _config) {
    status_t ret = dm_imu_l1_drv_t::init();
    if(ret != PYRO_OK){
        return ret;//初始化成功
    }
    
    dm_imu_cali_imu_data = &_data;//全局观测数据

    if (_config.calibrate){
        DM_Calibrate_MPU_Offset(&_config, &_data);
        return PYRO_WARNING;
    }
    else
    {
        _data.GyroOffset[0] = _config.gx_offset;
        _data.GyroOffset[1] = _config.gy_offset;
        _data.GyroOffset[2] = _config.gz_offset;
        _data.gNorm = _config.g_norm;
        _data.AccelScale = 9.81f / _data.gNorm;
        _data.TempWhenCali = 40;
    }
    float start_init_time = dwt_drv_t::get_timeline_s();
    while(_is_online == false){
        update_feedback();
        float cur_time = dwt_drv_t::get_timeline_s();
        if(cur_time - start_init_time > 5.0f){
            return PYRO_ERROR;
        }
    }
    
   return PYRO_OK;
}
void dm_imu_l1_drv_t::imu_update(){
    this->update_feedback();
    
}
void dm_imu_l1_drv_t::get_imu_data(IMU_Data_t &imu_data){
    imu_data.Accel[0] = _data.Accel[0];
    imu_data.Accel[1] = _data.Accel[1];
    imu_data.Accel[2] = _data.Accel[2];
    imu_data.Gyro[0] = _data.Gyro[0];
    imu_data.Gyro[1] = _data.Gyro[1];
    imu_data.Gyro[2] = _data.Gyro[2];
    imu_data.GyroOffset[0] = _data.GyroOffset[0];
    imu_data.GyroOffset[1] = _data.GyroOffset[1];
    imu_data.GyroOffset[2] = _data.GyroOffset[2];
    imu_data.gNorm = _data.gNorm;
    imu_data.AccelScale = _data.AccelScale;
    imu_data.TempWhenCali = _data.TempWhenCali;
    imu_data.Temperature = _data.Temperature;
}
//dm_imu_l1_drv_t *dm_imu;




void dm_imu_l1_drv_t::DM_Calibrate_MPU_Offset(const dm_imu_l1_config_t *config, dm_imu_l1_data_t *dm_imu_data)
{
    constexpr uint16_t CaliTimes = 6000;
    float gyroMax[3] = {0.0f, 0.0f, 0.0f};
    float gyroMin[3] = {0.0f, 0.0f, 0.0f};
    float gNormTemp = 0.0f;
    float gNormMax = 0.0f;
    float gNormMin = 0.0f;
    uint32_t gyroSamples = 0;
    uint32_t accelSamples = 0;

    // 校准期间必须使用原始角速度：不扣零偏、不做量程缩放，否则会把正在估计的零偏扣掉
    const uint8_t savedCaliOffset = caliOffset;
    caliOffset = 0;
    dm_imu_data->AccelScale = 1.0f;

    const float startTime = pyro::dwt_drv_t::get_timeline_s();
    do
    {
        if (pyro::dwt_drv_t::get_timeline_s() - startTime > 10.0f)
        {
            // 超时：沿用配置值，避免写入未收敛的结果
            dm_imu_data->GyroOffset[0] = config->gx_offset;
            dm_imu_data->GyroOffset[1] = config->gy_offset;
            dm_imu_data->GyroOffset[2] = config->gz_offset;
            dm_imu_data->gNorm = config->g_norm;
            dm_imu_data->TempWhenCali = 40.0f;
            break;
        }

        pyro::dwt_drv_t::delay_s(0.005f);
        dm_imu_data->gNorm = 0.0f;
        dm_imu_data->GyroOffset[0] = 0.0f;
        dm_imu_data->GyroOffset[1] = 0.0f;
        dm_imu_data->GyroOffset[2] = 0.0f;

        gyroSamples = 0;
        accelSamples = 0;
        bool firstGyro = true;
        bool firstAccel = true;

        for (uint16_t i = 0; i < CaliTimes; i++)
        {
            // 只有解出新的已知帧才计数，避免同一帧被重复累加
            if (update_feedback() == PYRO_OK)
            {
                if (_last_frame_type == FRAME_GYRO)
                {
                    for (uint8_t j = 0; j < 3; j++)
                    {
                        const float gyro = dm_imu_data->Gyro[j];
                        dm_imu_data->GyroOffset[j] += gyro;
                        if (firstGyro)
                        {
                            gyroMax[j] = gyro;
                            gyroMin[j] = gyro;
                        }
                        else
                        {
                            if (gyro > gyroMax[j])
                                gyroMax[j] = gyro;
                            if (gyro < gyroMin[j])
                                gyroMin[j] = gyro;
                        }
                    }
                    firstGyro = false;
                    gyroSamples++;
                }
                else if (_last_frame_type == FRAME_ACCEL)
                {
                    gNormTemp = sqrtf(dm_imu_data->Accel[0] * dm_imu_data->Accel[0] +
                                      dm_imu_data->Accel[1] * dm_imu_data->Accel[1] +
                                      dm_imu_data->Accel[2] * dm_imu_data->Accel[2]);
                    dm_imu_data->gNorm += gNormTemp;
                    if (firstAccel)
                    {
                        gNormMax = gNormTemp;
                        gNormMin = gNormTemp;
                    }
                    else
                    {
                        if (gNormTemp > gNormMax)
                            gNormMax = gNormTemp;
                        if (gNormTemp < gNormMin)
                            gNormMin = gNormTemp;
                    }
                    firstAccel = false;
                    accelSamples++;
                }

                if (gyroSamples > 1 && accelSamples > 1)
                {
                    gNormDiff = gNormMax - gNormMin;
                    for (uint8_t j = 0; j < 3; j++)
                        gyroDiff[j] = gyroMax[j] - gyroMin[j];
                    if (gNormDiff > 0.5f ||
                        gyroDiff[0] > 0.15f ||
                        gyroDiff[1] > 0.15f ||
                        gyroDiff[2] > 0.15f)
                        break;
                }
            }
            pyro::dwt_drv_t::delay_s(0.0005f);
        }

        if (accelSamples > 0)
            dm_imu_data->gNorm /= static_cast<float>(accelSamples);
        if (gyroSamples > 0)
            for (uint8_t j = 0; j < 3; j++)
                dm_imu_data->GyroOffset[j] /= static_cast<float>(gyroSamples);

        gNormDiff = 0.0f;
        for (uint8_t j = 0; j < 3; j++)
            gyroDiff[j] = 0.0f;
        if (accelSamples > 1)
            gNormDiff = gNormMax - gNormMin;
        if (gyroSamples > 1)
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];

        dm_imu_data->TempWhenCali = dm_imu_data->Temperature;

        caliCount++;
    } while (gyroSamples == 0 || accelSamples == 0 ||
             gNormDiff > 0.5f ||
             fabsf(dm_imu_data->gNorm - 9.8f) > 0.5f ||
             gyroDiff[0] > 0.15f ||
             gyroDiff[1] > 0.15f ||
             gyroDiff[2] > 0.15f);

    caliOffset = savedCaliOffset;

    if (dm_imu_data->gNorm > 1.0f)
        dm_imu_data->AccelScale = 9.81f / dm_imu_data->gNorm;
    else
        dm_imu_data->AccelScale = 9.81f / config->g_norm;
}




status_t dm_imu_l1_drv_t::get_angles(float *yaw, float *pitch, float *roll)
{
    if (roll == nullptr || pitch == nullptr || yaw == nullptr)
    {
        return PYRO_ERROR;
    }
    *roll  = _data.roll;
    *pitch = _data.pitch;
    *yaw   = _data.yaw;
    return PYRO_OK;
}

status_t dm_imu_l1_drv_t::get_rads(float *rad_yaw, float *rad_pitch,
                               float *rad_roll)
{
    if (rad_roll == nullptr || rad_pitch == nullptr || rad_yaw == nullptr)
    {
        return PYRO_ERROR;
    }
    *rad_roll  = _data.roll * PI / 180.0f;
    *rad_pitch = _data.pitch * PI / 180.0f;
    *rad_yaw   = _data.yaw * PI / 180.0f;
    return PYRO_OK;
}


status_t dm_imu_l1_drv_t::get_gyro(float *g_yaw, float *g_pitch, float *g_roll)
{
    if (g_yaw == nullptr || g_pitch == nullptr || g_roll == nullptr)
    {
        return PYRO_ERROR;
    }
    *g_roll  = _data.Gyro[0];
    *g_pitch = _data.Gyro[1];
    *g_yaw   = _data.Gyro[2];
    return PYRO_OK;
}


status_t dm_imu_l1_drv_t::get_accel(float *accel_x, float *accel_y, float *accel_z)
{
    if (accel_x == nullptr || accel_y == nullptr || accel_z == nullptr)
    {
        return PYRO_ERROR;
    }
    *accel_x = _data.Accel[0];
    *accel_y = _data.Accel[1];
    *accel_z = _data.Accel[2];
    return PYRO_OK;
}


status_t dm_imu_l1_drv_t::get_quaternion(float *q0, float *q1, float *q2, float *q3)
{
    if (q0 == nullptr || q1 == nullptr || q2 == nullptr || q3 == nullptr)
    {
        return PYRO_ERROR;
    }
    *q0 = _data.quaternion[0];
    *q1 = _data.quaternion[1];
    *q2 = _data.quaternion[2];
    *q3 = _data.quaternion[3];
    return PYRO_OK;
}



}// namespace
