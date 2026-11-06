#include "QuaternionEKF.h"

namespace pyro
{ 
const float Init_IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};
const float Init_IMU_QuaternionEKF_P[36] = {100000, 0.1,   0.1,    0.1, 0.1, 0.1,
                                     0.1,   100000, 0.1,    0.1, 0.1, 0.1,
                                     0.1,   0.1,    100000, 0.1, 0.1, 0.1,
                                     0.1,   0.1,    0.1, 100000, 0.1, 0.1,
                                     0.1,   0.1,    0.1, 0.1, 100, 0.1,
                                     0.1,   0.1,    0.1, 0.1, 0.1, 100};




static float invSqrt(float x);

QuaternionEKF_t::QuaternionEKF_t()
{
    Initialized = 0;
    ConvergeFlag = 0;
    StableFlag = 0;
    ErrorCount   = 0;
    UpdateCount  = 0;

    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;

    GyroBias[0] = 0.0f;
    GyroBias[1] = 0.0f;
    GyroBias[2] = 0.0f;

    dt         = 0.0f;
    lambda     = 1.0f;
    accLPFcoef = 0.0f;

    IMU_QuaternionEKF.UserData = this;
}

//自定义初始化函数
// static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
// static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
// static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
//static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);



/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */

void QuaternionEKF_t::QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf)
{
    Initialized = 1;
    Q1 = process_noise1;
    Q2 = process_noise2;
    R = measure_noise;
    ChiSquareTestThreshold = 1e-8;
    ConvergeFlag = 0;
    ErrorCount = 0;
    UpdateCount = 0;
    
    if (lambda > 1)
    {
        lambda = 1;
    }
    this->lambda = lambda;
    accLPFcoef = lpf;

    // 初始化矩阵维度信息
    Kalman_Filter_Init(&IMU_QuaternionEKF, 6, 0, 3);
    Matrix_Init(&ChiSquare, 1, 1, (float *)ChiSquare_Data);

    // 姿态初始化
    IMU_QuaternionEKF.xhat_data[0] = 1;
    IMU_QuaternionEKF.xhat_data[1] = 0;
    IMU_QuaternionEKF.xhat_data[2] = 0;
    IMU_QuaternionEKF.xhat_data[3] = 0;

    // 自定义函数初始化,用于扩展或增加kf的基础功能
    IMU_QuaternionEKF.UserData = this; // 将类对象指针传入kf结构体中,用于自定义函数中调用类成员变量

    IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    IMU_QuaternionEKF.SkipEq3 = TRUE;
    IMU_QuaternionEKF.SkipEq4 = TRUE;

    memcpy(IMU_QuaternionEKF.F_data, Init_IMU_QuaternionEKF_F, sizeof(Init_IMU_QuaternionEKF_F));
    memcpy(IMU_QuaternionEKF.P_data, Init_IMU_QuaternionEKF_P, sizeof(Init_IMU_QuaternionEKF_P));
}


/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s²
 * @param[in]       update period in s
 */
void QuaternionEKF_t::QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    float halfgxdt, halfgydt, halfgzdt;
    float accelInvNorm;
    if (!Initialized)
    {
        QuaternionEKF_Init(10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);
    }

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    this->dt = dt;

    Gyro[0] = gx - GyroBias[0];
    Gyro[1] = gy - GyroBias[1];
    Gyro[2] = gz - GyroBias[2];

    // set F
    halfgxdt = 0.5f * Gyro[0] * dt;
    halfgydt = 0.5f * Gyro[1] * dt;
    halfgzdt = 0.5f * Gyro[2] * dt;

    // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    memcpy(IMU_QuaternionEKF.F_data, Init_IMU_QuaternionEKF_F, sizeof(Init_IMU_QuaternionEKF_F));

    IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    IMU_QuaternionEKF.F_data[2] = -halfgydt;
    IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    IMU_QuaternionEKF.F_data[6] = halfgxdt;
    IMU_QuaternionEKF.F_data[8] = halfgzdt;
    IMU_QuaternionEKF.F_data[9] = -halfgydt;

    IMU_QuaternionEKF.F_data[12] = halfgydt;
    IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    IMU_QuaternionEKF.F_data[15] = halfgxdt;

    IMU_QuaternionEKF.F_data[18] = halfgzdt;
    IMU_QuaternionEKF.F_data[19] = halfgydt;
    IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
    if (UpdateCount == 0) // 如果是第一次进入,需要初始化低通滤波
    {
        Accel[0] = ax;
        Accel[1] = ay;
        Accel[2] = az;
    }
    Accel[0] = Accel[0] * accLPFcoef / (dt + accLPFcoef) + ax * dt / (dt + accLPFcoef);
    Accel[1] = Accel[1] * accLPFcoef / (dt + accLPFcoef) + ay * dt / (dt + accLPFcoef);
    Accel[2] = Accel[2] * accLPFcoef / (dt + accLPFcoef) + az * dt / (dt + accLPFcoef);

    // set z,单位化重力加速度向量
    accelInvNorm = invSqrt(Accel[0] * Accel[0] + Accel[1] * Accel[1] + Accel[2] * Accel[2]);
    for (uint8_t i = 0; i < 3; i++)
    {
        IMU_QuaternionEKF.MeasuredVector[i] = Accel[i] * accelInvNorm; // 用加速度向量更新量测值
    }

    // get body state
    gyro_norm = 1.0f / invSqrt(Gyro[0] * Gyro[0] +
                                        Gyro[1] * Gyro[1] +
                                        Gyro[2] * Gyro[2]);
    accl_norm = 1.0f / accelInvNorm;

    // 静止判断用原始陀螺。如果零偏状态跑飞，Gyro（已减零偏）会很大，
    // 用 Gyro 会让 StableFlag 永远为 0，发散恢复逻辑就永远触发不了。
    const float raw_gyro_norm = sqrtf(gx * gx + gy * gy + gz * gz);

    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    // 稍后在最后的姿态更新部分会利用StableFlag来确定
    if (raw_gyro_norm < 0.3f && accl_norm > 9.8f - 0.5f && accl_norm < 9.8f + 0.5f)
    {
        StableFlag = 1;
    }
    else
    {
        StableFlag = 0;
    }

    // set Q R,过程噪声和观测噪声矩阵
    IMU_QuaternionEKF.Q_data[0] = Q1 * dt;
    IMU_QuaternionEKF.Q_data[7] = Q1 * dt;
    IMU_QuaternionEKF.Q_data[14] = Q1 * dt;
    IMU_QuaternionEKF.Q_data[21] = Q1 * dt;
    IMU_QuaternionEKF.Q_data[28] = Q2 * dt;
    IMU_QuaternionEKF.Q_data[35] = Q2 * dt;
    IMU_QuaternionEKF.R_data[0] = R;
    IMU_QuaternionEKF.R_data[4] = R;
    IMU_QuaternionEKF.R_data[8] = R;

    // 调用kalman_filter.c封装好的函数,注意几个User_Funcx_f的调用
    Kalman_Filter_Update(&IMU_QuaternionEKF);

    // 获取融合后的数据,包括四元数和xy零飘值
    q[0] = IMU_QuaternionEKF.FilteredValue[0];
    q[1] = IMU_QuaternionEKF.FilteredValue[1];
    q[2] = IMU_QuaternionEKF.FilteredValue[2];
    q[3] = IMU_QuaternionEKF.FilteredValue[3];
    GyroBias[0] = IMU_QuaternionEKF.FilteredValue[4];
    GyroBias[1] = IMU_QuaternionEKF.FilteredValue[5];
    GyroBias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移

    // 利用四元数反解欧拉角
    Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    Roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    Pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.295779513f;

    // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    if (Yaw - YawAngleLast > 180.0f)
    {
        YawRoundCount--;
    }
    else if (Yaw - YawAngleLast < -180.0f)
    {
        YawRoundCount++;
    }
    YawTotalAngle = 360.0f * YawRoundCount + Yaw;
    YawAngleLast = Yaw;
    UpdateCount++; // 初始化低通滤波用,计数测试用
}



/**
 * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新;
 *        并对零漂的方差进行限制,防止过度收敛并限幅防止发散
 *
 * @param kf
 */
void QuaternionEKF_t::IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    //static float q0, q1, q2, q3;
    //static float qInvNorm;
    float q0, q1, q2, q3;
    float qInvNorm;
    auto* owner = static_cast<QuaternionEKF_t*>(kf->UserData);

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // quaternion normalize
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    
    kf->F_data[4] = q1 * owner->dt / 2;
    kf->F_data[5] = q2 * owner->dt / 2;

    kf->F_data[10] = -q0 * owner->dt / 2;
    kf->F_data[11] = q3 * owner->dt / 2;

    kf->F_data[16] = -q3 * owner->dt / 2;
    kf->F_data[17] = -q0 * owner->dt / 2;

    kf->F_data[22] = q2 * owner->dt / 2;
    kf->F_data[23] = -q1 * owner->dt / 2;

    // fading filter,防止零飘参数过度收敛
    kf->P_data[28] /= owner->lambda;
    kf->P_data[35] /= owner->lambda;

    // 限幅,防止发散
    if (kf->P_data[28] > 10000)
    {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000)
    {
        kf->P_data[35] = 10000;
    }
}


/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 *
 * @param kf
 */
void QuaternionEKF_t::IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    float doubleq0, doubleq1, doubleq2, doubleq3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 *
 * @param kf
 */
void QuaternionEKF_t::IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    //static float q0, q1, q2, q3;
    float q0, q1, q2, q3;
    auto* owner = static_cast<QuaternionEKF_t*>(kf->UserData);
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    // 计算预测得到的重力加速度方向(通过姿态获取的)
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 计算预测值和各个轴的方向余弦
    for (uint8_t i = 0; i < 3; i++)
    {
        owner->OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    // 利用加速度计数据修正
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test,卡方检验
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &owner->ChiSquare);
    // rk is small,filter converged/converging
    if (owner->ChiSquare_Data[0] < 0.5f * owner->ChiSquareTestThreshold)
    {
        owner->ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged
    if (owner->ChiSquare_Data[0] > owner->ChiSquareTestThreshold && owner->ConvergeFlag)
    {
        if (owner->StableFlag)
        {
            owner->ErrorCount++; // 载体静止时仍无法通过卡方检验
        }
        else
        {
            owner->ErrorCount = 0;
        }

        if (owner->ErrorCount > 50)
        {
            // 滤波器发散
            owner->ConvergeFlag = 0;
            kf->SkipEq5 = FALSE; // step-5 is cov mat P updating
        }
        else
        {
            //  残差未通过卡方检验 仅预测
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE; // part5 is P updating
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk越小则增益越大,否则更相信预测值
        if (owner->ChiSquare_Data[0] > 0.1f * owner->ChiSquareTestThreshold && owner->ConvergeFlag)
        {
            owner->AdaptiveGainScale = (owner->ChiSquareTestThreshold - owner->ChiSquare_Data[0]) / (0.9f * owner->ChiSquareTestThreshold);
        }
        else
        {
            owner->AdaptiveGainScale = 1;
        }
        owner->ErrorCount = 0;
        kf->SkipEq5 = FALSE;
    }

    // cal kf-gain K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // implement adaptive
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; i++)
    {
        kf->K_data[i] *= owner->AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf->K_data[i * 3 + j] *= owner->OrientationCosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))

    // 零漂修正限幅,一般不会有过大的漂移
    if (owner->ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * owner->dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * owner->dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * owner->dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * owner->dt;
            }
        }
    }

    // 不修正yaw轴数据
    kf->temp_vector.pData[3] = 0;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);

    // 陀螺零偏状态硬限幅。正常零偏不会超过 ±0.05 rad/s（约 ±2.9°/s），
    // 限幅防止滤波发散时零偏状态跑飞，进而把姿态持续转下去。
    for (uint8_t i = 4; i < 6; i++)
    {
        if (kf->xhat_data[i] > 0.05f)
            kf->xhat_data[i] = 0.05f;
        else if (kf->xhat_data[i] < -0.05f)
            kf->xhat_data[i] = -0.05f;
    }
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 *
 * @param kf kf类型定义
 */
void QuaternionEKF_t::IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{   auto* owner = static_cast<QuaternionEKF_t*>(kf->UserData);
    memcpy(owner->IMU_QuaternionEKF_P, kf->P_data, sizeof(owner->IMU_QuaternionEKF_P));
    memcpy(owner->IMU_QuaternionEKF_K, kf->K_data, sizeof(owner->IMU_QuaternionEKF_K));
    memcpy(owner->IMU_QuaternionEKF_H, kf->H_data, sizeof(owner->IMU_QuaternionEKF_H));
}

/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

} // namespace pyro