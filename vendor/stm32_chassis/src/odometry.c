#include "sys.h"

//里程计模块 */

extern void Odom_xyz(float vel_a, float vel_b, float vel_c, float vel_d,
                      float ang_a, float ang_b, float ang_c, float ang_d);

void Odom_xyz(float vel_a,float vel_b,float vel_c,float vel_d,float ang_a,float ang_b,float ang_c,float ang_d)

{

	//轮子半径 WHEEL_DIAMETER_MM/2 mm（自动解算自chassis_config.h）

	//to_rad  0.017453f  //角度转弧度

	//轴距和轮距由 chassis_config.h 中的 WHEELBASE_MM / TRACK_WIDTH_MM 定义

  //设定车子正负方向 ，车子前进方向为X正，后退为X负，左移为Y正，右移为Y负

	 //轮子从上往下看，逆时针转为正角度，顺时针转为负角度

    const float dt = 1.0f / CONTROL_FREQ_HZ; // 中断周期=1/控制频率
    const float L1 = WHEELBASE_MM / 1000.0f;  // 前后轮轴距（m）
    const float W  = TRACK_WIDTH_MM / 1000.0f; // 左右轮中心距（m）



	

    float cos_a = cos(ang_a * to_rad);

    float sin_a = sin(ang_a * to_rad);

    float cos_b = cos(ang_b * to_rad);

    float sin_b = sin(ang_b * to_rad);

    float cos_c = cos(ang_c * to_rad);

    float sin_c = sin(ang_c * to_rad);

    float cos_d = cos(ang_d * to_rad);

    float sin_d = sin(ang_d * to_rad);



    //四舵轮速度正解（修正vth符号，确保逆时针自转时vth为正）

    // 底盘坐标系：vx（前进+）、vy（左移+）、vth（逆时针+）

    vx = (vel_a * cos_a + vel_b * cos_b + vel_c * cos_c + vel_d * cos_d) / 4.0f;

    vy = (vel_a * sin_a + vel_b * sin_b + vel_c * sin_c + vel_d * sin_d) / 4.0f;

    

    // vth计算：分母为L2/4 + W2/4（旋转半径的平方和）

    float denom = (L1*L1 + W*W) / 4.0f;  // 0.172 + 0.122 = 0.0433，与原代码一致

    vth = (vel_a * (sin_a * (L1/2) - cos_a * (W/2)) +  // A轮：(+L/2, -W/2)

           vel_b * (sin_b * (L1/2) + cos_b * (W/2)) +  // B轮：(+L/2, +W/2)

           vel_c * (-sin_c * (L1/2) + cos_c * (W/2)) + // C轮：(-L/2, +W/2)

           vel_d * (-sin_d * (L1/2) - cos_d * (W/2)))/ denom;  // D轮：(-L/2, -W/2)

          

					

//    vth = -vth;  // 确保逆时针自转时vth为正（根据实际测试调整，若方向反则注释此行）



//世界坐标系位移积分（核心修正：用当前累计角度Delta_th做旋转，而非增量）

    // 旋转矩阵：底盘坐标系 -> 世界坐标系（逆时针旋转Delta_th）

    float cos_th = cos(Delta_th);

    float sin_th = sin(Delta_th);

    

    // 位移增量 = 底盘速度 * 旋转矩阵 * 时间

    float dx = (vx * cos_th - vy * sin_th) * dt;

    float dy = (vx * sin_th + vy * cos_th) * dt;

    float dth = vth * dt;



    // 积分更新

    Delta_x += dx;

    Delta_y += dy;

    Delta_th += dth;



 //（可选）角度归一化到[-π, π]，避免数值溢出

    if (Delta_th > M_PI) Delta_th -= 2*M_PI;

    else if (Delta_th < -M_PI) Delta_th += 2*M_PI;

		

    if (Flag_init == 0) {//Flag_init标志位置0时会清除已累计的数据

        Flag_init = 1;

        Delta_x = 0;

        Delta_y = 0;

        Delta_th = 0;

    }				

}

/**************************************************************************

函数功能：位置PD控制 

**************************************************************************/

