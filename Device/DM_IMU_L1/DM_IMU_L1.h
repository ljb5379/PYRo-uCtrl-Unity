/**
 * @file DM_IMU_L1.h
 * @brief 达妙 DM-IMU-L1 六轴 IMU 驱动（CAN 通信）（只收）
 *
 * 协议要点（DM-IMU-L1 说明书 V1.3 附录 C）：
 *  - 经典 CAN 标准帧，DLC 固定为 8 字节。
 *  - 数据帧 DATA[0] 为数据类型：1 加速度+温度，2 角速度，
 *
 * @note 底层 can_msg_buffer_t 每个 ID 只保存最新一帧。主动模式下设备
 *       会用同一个主机 ID 依次发送多种数据帧，update_feedback() 的调用
 */
#ifndef DM_IMU_L1_H
#define DM_IMU_L1_H

#include "pyro_bsp_can.h"
#include "pyro_core_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include <array>
#include <cstdint>
#include "imu_base.h"
#include "pyro_dwt_drv.h"
namespace pyro
{

/**
 * @brief DM-IMU-L1 解算后的数据。
 */
struct dm_imu_l1_data_t: public IMU_Data_t
{
    // float accel[3];      ///< X/Y/Z 加速度，m/s^2
    // float gyro[3];       ///< X/Y/Z 角速度，rad/s
    float roll  = 0.0f;  ///< 横滚角，deg，[-180, 180]
    float pitch = 0.0f;  ///< 俯仰角，deg，[-90, 90]
    float yaw   = 0.0f;  ///< 偏航角，deg，[-180, 180]
    float quaternion[4]{}; ///< 四元数 w/x/y/z，归一化
    // float temperature;   ///< 传感器温度，degC
    //新增————初始化数据
    // float g_norm;
    // float gx_offset[3];
    // bool calibrate;
};

using dm_imu_l1_config_t = imu_config_t;

/**
 * @brief 达妙 DM-IMU-L1 驱动类。
 */
class dm_imu_l1_drv_t:public imu_base_t
{
  public:
    

    /**
     * @brief 主动输出数据类型位掩码。
     */
    enum output_mask_t : uint8_t
    {
        output_quaternion = 0x01,
        output_euler      = 0x02,
        output_gyro       = 0x04,
        output_accel      = 0x08,
        output_all        = 0x0F,
    };

    /**
     * @brief 构造函数。
     *
     * @param slave_id  设备从机 ID（命令帧目标），出厂默认 0x01。
     * @param master_id 设备主机 ID（数据帧来源），出厂默认 0x11。
     * @param which     使用的 CAN 总线。
     */
    dm_imu_l1_drv_t(uint8_t slave_id, uint8_t master_id,
                    bsp_can::which_can which);
    ~dm_imu_l1_drv_t();

    /**
     * @brief 注册 CAN 接收缓冲区；建议在 CAN 启动前或启动后立即调用。
     */
    status_t init();

    /**
     * @brief 解析一帧接收数据，需在任务上下文中周期调用。
     *
     * @retval PYRO_OK      成功解析一帧
     * @retval PYRO_BUSY    暂无新数据
     * @retval PYRO_WARNING 收到未知数据类型
     * @retval PYRO_ERROR   驱动未初始化
     */
    status_t update_feedback();
    
  

    /**
     * @brief 拷贝全部最新解算数据。
     *
     * @retval PYRO_OK   已收到过数据
     * @retval PYRO_BUSY 尚未收到任何数据
     */
    status_t get_data(dm_imu_l1_data_t &data) const;

    /**
     * @brief 判断设备是否在线。
     */
    bool is_online() const;

    void read_data(IMU_Data_t *imu_data);
//ins 使用的接口函数
    status_t imu_init(imu_config_t _config) override;
    void imu_update() override;
    void get_imu_data(IMU_Data_t &imu_data) override;
//其他使用的接口函数
    status_t get_angles(float *yaw, float *pitch, float *roll);
    status_t get_rads(float* rad_yaw, float* rad_pitch, float* rad_roll);
    status_t get_gyro(float* g_yaw, float* g_pitch, float* g_roll);
    status_t get_accel(float* accel_x, float* accel_y, float* accel_z);
    status_t get_quaternion(float* q0, float* q1, float* q2, float* q3);
  private:
//解码
    status_t _decode(const std::array<uint8_t, 8> &frame);

    uint8_t _slave_id;
    uint8_t _master_id;
    can_drv_t *_can_drv;
    can_msg_buffer_t *_msg_buffer;

    dm_imu_l1_data_t _data;
    float _last_rx_tick;

    bool _is_online;
    uint8_t _last_frame_type; // 最近一次解码的帧类型，校准用
    void DM_Calibrate_MPU_Offset(const dm_imu_l1_config_t *config, dm_imu_l1_data_t *dm_imu_data);
};
//extern dm_imu_l1_drv_t *dm_imu;
} // namespace pyro

#endif // DM_IMU_L1_H
