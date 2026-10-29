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

#define ARM_MOTORS_USE (RotaryU1_USE || RotaryDJ_USE || RotaryZ1_USE || PrismaticZ1_USE)
/* Business kinematics requires both rotary joints. */
#define ARM_USE_FK (USE_FK && RotaryU1_USE && RotaryZ1_USE)
#define ARM_USE_IK (USE_IK && RotaryU1_USE && RotaryZ1_USE)

/* 位置配置（三台位置电机 + 运行时间）放在 Arm_Pose_Table，末端旋转独立管理，
   新机构数值待实测标定 */

extern volatile uint8_t Is_ok;

typedef enum
{
    ARM_STATE_SILENT = 0,       // 静默状态(上电初态)
    ARM_STATE_SKY_PRE_H=1,        // 横预取天空块
    ARM_STATE_SKY_PRE_V=2,        // 竖预取天空块
    ARM_STATE_SKY_PICK_H=3,       // 横取天空块
    ARM_STATE_SKY_PICK_V=4,       // 竖取天空块
    ARM_STATE_END_CW90 = 5,    // 末端相对机械零位顺时针90°
    ARM_STATE_END_CW180 = 6,   // 末端相对机械零位顺时针180°
    ARM_STATE_END_CCW90 = 7,   // 末端相对机械零位逆时针90°
    ARM_STATE_SKY_KEEP=8,   // 持天空块位置，保持理想末端角度
    // 8: 原 CCW180 状态编号保留，不再使用
    ARM_STATE_SKY_PLACE = 9,   // 放置天空块，保持理想末端角度
    ARM_STATE_BALL_PRE = 10,   // 预取球
    ARM_STATE_BALL_PICK ,  // 取球
    ARM_STATE_BALL_PLACE, // 放置球

#if ARM_USE_FK
    ARM_DEBUG,              // 正运动学调试
#endif
#if ARM_USE_IK
    ARM_Pos_DEBUG,          // 逆运动学关节坐标系调试
#endif
#if (ARM_USE_FK && ARM_USE_IK)
    ARM_CartPos_DEBUG,      // 逆运动学笛卡尔坐标调试
#endif
    ARM_STATE_COUNT
} ArmState_t;

typedef enum
{
    ARM_MODE_DEFAULT = 0,   // 默认：关节空间直接给电机转角
    ARM_MODE_CARTESIAN      // 笛卡尔：末端坐标轨迹 + 逐周期逆解（需 ARM_USE_FK && ARM_USE_IK）
} ArmMode_t;

// 配置表条目（下标 = ArmState_t；旋转状态仅使用 move_time）
typedef struct
{
#if RotaryU1_USE
    float u1;             // rad
#endif
#if PrismaticZ1_USE
    float prismatic_z1;   // deg
#endif
#if RotaryDJ_USE
    float dj;             // deg
#endif
    float move_time;      // s
} Arm_Pose_Config_t;

// 运行时完整目标：由位置表与独立末端角度逐项组装
typedef struct
{
#if RotaryU1_USE
    float u1;             // 宇树电机U1转角 (rad)
#endif
#if RotaryZ1_USE
    float rotary_z1;      // GIM4310(Zmotor[0])转角 (deg)
#endif
#if PrismaticZ1_USE
    float prismatic_z1;   // U8Lite(Zmotor[1])位置 (deg)
#endif
#if RotaryDJ_USE
    float dj;             // 大疆M3508转角 (deg)
#endif
    float move_time;      // 运行时间 (s)
} Arm_Pose_t;

// 轨迹控制结构体
typedef struct
{
#if RotaryU1_USE
    float u1_target, u1_start;
#endif
#if RotaryZ1_USE
    float rotary_z1_target, rotary_z1_start;
#endif
#if PrismaticZ1_USE
    float prismatic_z1_target, prismatic_z1_start;
#endif
#if RotaryDJ_USE
    float dj_target, dj_start;
#endif
    float time;        // 运行时间
    float total_time;  // 总运行时间
    float dt;          // 运动更新周期
#if (ARM_USE_FK && ARM_USE_IK)
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
#if ARM_MOTORS_USE
typedef struct
{
#if RotaryU1_USE
    UnitreeMotor *RotaryU1;    // 宇树U1(大臂)
#endif
#if RotaryDJ_USE
    DJMotor      *RotaryDJ;    // M3508(第2台)
#endif
#if RotaryZ1_USE
    Zdrive *RotaryZ1;          // GIM4310(Zmotor[0])
#endif
#if PrismaticZ1_USE
    Zdrive *PrismaticZ1;       // U8Lite(Zmotor[1])
#endif
}Arm_Motor;
#endif



// 机械臂运动控制结构体（重构）
typedef struct
{
    volatile bool        enable;     // 使能
    volatile bool        reset;      // 复位（系统复位请求）
    volatile ArmState_t  state;      // 当前状态
    volatile ArmState_t  req_state;  // 请求状态（变化沿检测源）

    Arm_Pose_t pose;   // 四台电机运行目标
#if RotaryZ1_USE
    int32_t end_quarter_turns; // 理想末端角度相对机械零位的90°步数
#endif
    ArmTraj_t  traj;   // 轨迹规划结构体
#if ARM_MOTORS_USE
    Arm_Motor arm_motor;
#endif

    volatile bool running;   // 插补运行中
    volatile bool finish;    // 插补完成
} ArmControl_t;

typedef struct
{
#if RotaryU1_USE
    float u1_target;
#endif
#if RotaryZ1_USE
    float rotary_z1_target;
#endif
#if PrismaticZ1_USE
    float prismatic_z1_target;
#endif
#if RotaryDJ_USE
    float dj_target;
#endif
    float move_time;

} Arm_Interpolation_t;

#if ARM_USE_IK
typedef struct
{
    float x;                 //末端x值
    float y;                 //末端y值
#if RotaryDJ_USE
    float dj_target;
#endif
    float move_time;

} Arm_Interpolation_Pos_t;
#endif

#if ARM_USE_FK
typedef struct
{
    float Angle1;
    float Angle2;
#if RotaryDJ_USE
    float Angle3;
#endif
}Arm_Angle;
#endif



#if (ARM_USE_FK || ARM_USE_IK)
typedef struct
{
#if ARM_USE_FK
    Arm_Interpolation_t motor_target;
    Arm_Angle arm_angle;
    Unitree_Theta_t forward_angle;
    Vec2 end_coordinate;
#endif
#if ARM_USE_IK
    Unitree_Theta_t inverse_angle;
#endif
    /* data */
}Arm_Kinetics_Data_t;
#endif

#if (ARM_USE_FK || ARM_USE_IK)
extern Arm_Kinetics_Data_t Arm_Kinetics_Data;
#endif
#if ARM_USE_FK
extern Arm_Interpolation_t Arm_Debug;
#endif
#if ARM_USE_IK
extern Arm_Interpolation_Pos_t Arm_Pos_Debug;
#endif
extern ArmControl_t ArmControl;
extern Arm_Pose_Config_t Arm_Pose_Table[ARM_STATE_COUNT];

void Relay_ON(void);
void Relay_OFF(void);
void Arm_Control_Init(void);
void Arm_Func(void);
void Arm_Motor_Enable(void);
void Arm_Motor_Disable(void);
void Arm_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data);

void Arm_Transmit(void);

#endif
