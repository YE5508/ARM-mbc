#ifndef SEIZE_SKY_H
#define SEIZE_SKY_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "UnitreeMotor.h"
#include "DJmotor.h"
#include "ZDrive.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "sky_config.h"
#include "kinematics.h"
#include "fdcan.h"

/* 状态位姿（四台电机 + 运行时间）统一放在 Arm_Pose_Table，见 seize_sky.c，
   新机构数值待实测标定 */

extern volatile uint8_t Is_ok;

typedef enum
{
    ARM_STATE_SILENT = 0,       // 静默状态(上电初态)
    ARM_STATE_SKY_PRE_H,        // 横预取天空块
    ARM_STATE_SKY_PRE_V,        // 竖预取天空块
    ARM_STATE_SKY_PICK_H,       // 横取天空块
    ARM_STATE_SKY_PICK_V,       // 竖取天空块
    ARM_STATE_SKY_KEEP_CW90,    // 持天空块:GIM4310 顺时针90°
    ARM_STATE_SKY_KEEP_CW180,   // 持天空块:GIM4310 顺时针180°
    ARM_STATE_SKY_KEEP_CCW90,   // 持天空块:GIM4310 逆时针90°
    ARM_STATE_SKY_KEEP_CCW180,  // 持天空块:GIM4310 逆时针180°
    ARM_STATE_SKY_PLACE,        // 放置天空块(沿用上一个持块状态的GIM角度)
    ARM_STATE_BALL_PRE,         // 预取球
    ARM_STATE_BALL_PICK,        // 取球
    ARM_STATE_BALL_PLACE,       // 放置球
#if USE_FK
    ARM_DEBUG,              // 正运动学调试
#endif
#if USE_IK
    ARM_Pos_DEBUG,          // 逆运动学关节坐标系调试
#endif
#if (USE_FK && USE_IK)
    ARM_CartPos_DEBUG,      // 逆运动学笛卡尔坐标调试
#endif
    ARM_STATE_COUNT
} ArmState_t;

typedef enum
{
    ARM_MODE_DEFAULT = 0,   // 默认：关节空间直接给电机转角
    ARM_MODE_CARTESIAN      // 笛卡尔：末端坐标轨迹 + 逐周期逆解（需 USE_FK && USE_IK）
} ArmMode_t;

// 配置表条目（下标 = ArmState_t 枚举值）
typedef struct
{
    float u1;             // 宇树电机U1转角 (rad)
    float rotary_z1;      // GIM4310(Zmotor[0])转角 (deg)
    float prismatic_z1;   // U8Lite(Zmotor[1])位置 (deg)
    float dj;             // 大疆M3508转角 (deg)
    float move_time;      // 运行时间 (s)
} Arm_Pose_t;

// 轨迹控制结构体
typedef struct
{
    float u1_target, rotary_z1_target, prismatic_z1_target, dj_target;  // 四台电机目标值
    float u1_start,  rotary_z1_start,  prismatic_z1_start,  dj_start;   // 四台电机起点（切换时读当前实际位置）
    float time;        // 运行时间
    float total_time;  // 总运行时间
    float dt;          // 运动更新周期
#if (USE_FK && USE_IK)
    float start_x, start_y, target_x, target_y; // 笛卡尔目标（仅 CARTESIAN）
#endif
} ArmTrajCtrl_t;

// 轨迹规划结构体
typedef struct
{
    ArmTrajCtrl_t ctrl;   // 控制结构体
    ArmMode_t      mode;  // 运行模式（笛卡尔 / 默认）
} ArmTraj_t;

//机械臂四个电机的引用(指向真实电机对象,由Arm_Control_Init赋值)
typedef struct
{
    UnitreeMotor *RotaryU1;    // 宇树U1(大臂)
    DJMotor      *RotaryDJ;    // M3508(第2台)
    Zdrive *RotaryZ1;          // GIM4310(Zmotor[0])
    Zdrive *PrismaticZ1;       // U8Lite(Zmotor[1])
}Arm_Motor;



// 机械臂运动控制结构体（重构）
typedef struct
{
    volatile bool        enable;     // 使能
    volatile bool        reset;      // 复位（系统复位请求）
    volatile ArmState_t  state;      // 当前状态
    volatile ArmState_t  req_state;  // 请求状态（变化沿检测源）

    Arm_Pose_t pose;   // 四台电机状态配置（来自配置表）
    ArmTraj_t  traj;   // 轨迹规划结构体
    Arm_Motor arm_motor;

    volatile bool running;   // 插补运行中
    volatile bool finish;    // 插补完成
} ArmControl_t;

typedef struct
{
    float u1_target;
    float rotary_z1_target;
    float prismatic_z1_target;
    float dj_target;
    float move_time;

} Arm_Interpolation_t;

#if USE_IK
typedef struct
{
    float x;                 //末端x值
    float y;                 //末端y值
    float dj_target;
    float move_time;

} Arm_Interpolation_Pos_t;
#endif

#if USE_FK
typedef struct
{
    float Angle1;
    float Angle2;
    float Angle3;
}Arm_Angle;
#endif



#if (USE_FK || USE_IK)
typedef struct
{
#if USE_FK
    Arm_Interpolation_t motor_target;
    Arm_Angle arm_angle;
    Unitree_Theta_t forward_angle;
    Vec2 end_coordinate;
#endif
#if USE_IK
    Unitree_Theta_t inverse_angle;
#endif
    /* data */
}Arm_Kinetics_Data_t;
#endif

#if (USE_FK || USE_IK)
extern Arm_Kinetics_Data_t Arm_Kinetics_Data;
#endif
#if USE_FK
extern Arm_Interpolation_t Arm_Debug;
#endif
#if USE_IK
extern Arm_Interpolation_Pos_t Arm_Pos_Debug;
#endif
extern ArmControl_t ArmControl;
extern Arm_Pose_t Arm_Pose_Table[ARM_STATE_COUNT];

void Relay_ON(void);
void Relay_OFF(void);
void Arm_Control_Init(void);
void Arm_Func(void);
void Arm_Motor_Enable(void);
void Arm_Motor_Disable(void);
void Arm_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data);

void Arm_Transmit(void);

#endif
