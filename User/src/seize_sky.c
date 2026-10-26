#include "seize_sky.h"
#include "kinematics.h"
#include "DJmotor.h"
#include "holding_jaw.h"
#include <math.h>


#define ARM_INTERPOLATION_DT    0.001f
#define ARM_EPSILON             0.0001f
#define ARM_MOVE_TIME           10.0f
#if USE_FK
#define ARM_Debug_MOVE_TIME     6.0f
#endif
#if USE_IK
#define ARM_Pos_Debug_MOVE_TIME 30.0f
#endif

//五次多项式轨迹参数(归一化时间域 t∈[0,1],末速度/末加速度固定为0)
//起始速度 s'(0) 与起始加速度 s''(0) 可配置;取0时为平滑起步(等价smoothstep)
#define ARM_START_VELOCITY    0.2f
#define ARM_START_ACCEL       2.0f

/*
typedef struct
{
    float u1;             // rad
    float prismatic_z1;   // deg
    float dj;             // deg
    float move_time;      // s
} Arm_Pose_Config_t;
*/

Arm_Pose_Config_t Arm_Pose_Table[ARM_STATE_COUNT] =
{
    // 新机构位置待实测标定；旋转状态仅使用 move_time，前三项运行时继承
    [ARM_STATE_SILENT]          = {-1.5f, 0.0f, -69.0f, ARM_MOVE_TIME},
    [ARM_STATE_SKY_PRE_H]       = {0.0f, 0.0f, 0.0f, ARM_MOVE_TIME},
    [ARM_STATE_SKY_PRE_V]       = {0.0f, 0.0f, 0.0f, ARM_MOVE_TIME},
    [ARM_STATE_SKY_PICK_H]      = {0.0f, 0.0f, 0.0f, ARM_MOVE_TIME},
    [ARM_STATE_SKY_PICK_V]      = {0.0f, 0.0f, 0.0f, ARM_MOVE_TIME},
    [ARM_STATE_END_CW90]        = {0.0f, 0.0f, 0.0f, ARM_MOVE_TIME},
    [ARM_STATE_END_CW180]       = {0.0f, 0.0f, 0.0f, ARM_MOVE_TIME},
    [ARM_STATE_END_CCW90]       = {0.0f, 0.0f, 0.0f, ARM_MOVE_TIME},
    [ARM_STATE_SKY_KEEP]        = {0.0f, 0.0f, 0.0f, ARM_MOVE_TIME},
    [ARM_STATE_SKY_PLACE]       = {0.0f, 0.0f, 0.0f, ARM_MOVE_TIME},
    [ARM_STATE_BALL_PRE]        = {0.0f, 0.0f, 0.0f, ARM_MOVE_TIME},
    [ARM_STATE_BALL_PICK]       = {0.0f, 0.0f, 0.0f, ARM_MOVE_TIME},
    [ARM_STATE_BALL_PLACE]      = {0.0f, 0.0f, 0.0f, ARM_MOVE_TIME},
#if USE_FK
    // 调试状态：位置/时长占位，运行时直接组装完整 pose
    [ARM_DEBUG]           = {0.0f, 0.0f, 0.0f, ARM_Debug_MOVE_TIME},
#endif
#if USE_IK
    [ARM_Pos_DEBUG]       = {0.0f, 0.0f, 0.0f, ARM_Pos_Debug_MOVE_TIME},
#endif
#if (USE_FK && USE_IK)
    [ARM_CartPos_DEBUG]   = {0.0f, 0.0f, 0.0f, ARM_Pos_Debug_MOVE_TIME},
#endif
};

/* 离散角度选择独立标定的绝对目标，反馈仅用于插补起点。 */
static float Arm_End_Target(void)
{
    switch (ArmControl.end_quarter_turns)
    {
        case -1: return ARM_END_CCW90_POS_DEG;
        case 1:  return ARM_END_CW90_POS_DEG;
        case 2:  return ARM_END_CW180_POS_DEG;
        default: return ARM_END_ZERO_POS_DEG;
    }
}

static Arm_Pose_t Arm_Load_Pose(ArmState_t state)
{
    Arm_Pose_t pose;
    pose.u1 = Arm_Pose_Table[state].u1;
    pose.prismatic_z1 = Arm_Pose_Table[state].prismatic_z1;
    pose.dj = Arm_Pose_Table[state].dj;
    pose.move_time = Arm_Pose_Table[state].move_time;
    pose.rotary_z1 = Arm_End_Target();

#if USE_FK
    if (state == ARM_DEBUG)
    {
        pose.u1 = Arm_Debug.u1_target;
        pose.rotary_z1 = Arm_Debug.rotary_z1_target;
        pose.prismatic_z1 = Arm_Debug.prismatic_z1_target;
        pose.dj = Arm_Debug.dj_target;
        pose.move_time = Arm_Debug.move_time;
    }
#endif
#if USE_IK
    if (state == ARM_Pos_DEBUG)
    {
        Vec2 p = {Arm_Pos_Debug.x, Arm_Pos_Debug.y};
        Unitree_Theta_t th = Inverse(p);
        pose.u1 = th.u1_theta;
        pose.rotary_z1 = RAD2DEG(th.u2_theta); // TODO 旧2R模型映射待标定
        pose.dj = Arm_Pos_Debug.dj_target;
        pose.move_time = Arm_Pos_Debug.move_time;
    }
#endif
#if (USE_FK && USE_IK)
    if (state == ARM_CartPos_DEBUG)
    {
        pose.dj = Arm_Pos_Debug.dj_target;
        pose.move_time = Arm_Pos_Debug.move_time;
    }
#endif
    return pose;
}

volatile uint8_t Is_ok=0;

ArmControl_t ArmControl;
#if USE_FK
Arm_Interpolation_t Arm_Debug={0,0,0,0,ARM_Debug_MOVE_TIME};
#endif
#if USE_IK
Arm_Interpolation_Pos_t Arm_Pos_Debug={0,0,0,ARM_Pos_Debug_MOVE_TIME};
#endif
#if (USE_FK || USE_IK)
Arm_Kinetics_Data_t Arm_Kinetics_Data={0};
#endif



static void Sys_reset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void Relay_ON(void)
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
}

void Relay_OFF(void)
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
}



//时间归一化,计算轨迹设定角度(五次多项式)
//s(t)=a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5, 边界条件:
//  s(0)=0, s(1)=1
//  s'(0)=v0(可配置), s'(1)=0
//  s''(0)=acc0(可配置), s''(1)=0
static float Quintic_S(float time, float total_time)
{
    float t;
    float t2;
    float t3;
    float t4;
    float t5;
    float v0   = ARM_START_VELOCITY;
    float acc0 = ARM_START_ACCEL;

    if (total_time <= 0.0f)
    {
        return 1.0f;
    }

    t=time/total_time;

    if(t<=0.0f)
    {
        t=0.0f;
    }
    else if(t>=1.0f)
    {
        t=1.0;
    }

    t2=t*t;
    t3=t2*t;
    t4=t3*t;
    t5=t4*t;

    //由边界条件解出的系数:
    //a1=v0, a2=acc0/2, a3=10-6v0-1.5acc0, a4=-15+8v0+1.5acc0, a5=6-3v0-0.5acc0
    return v0*t
      +0.5f*acc0*t2
      +(10.0f-6.0f*v0-1.5f*acc0)*t3
      +(-15.0f+8.0f*v0+1.5f*acc0)*t4
      +(6.0f-3.0f*v0-0.5f*acc0)*t5;
}

static float Target_Quintic_Interpolation(float start_angle, float target,
                                          float time, float total_time)
{
    if (total_time<=0)
    {
        return target;
    }

    float s = Quintic_S(time, total_time);
    return start_angle + (target - start_angle) * s;
}

//开始轨迹计算
static void Arm_Interpolation_Start(Arm_Pose_t pose)
{
    ArmControl.pose = pose;                              // 更新状态配置（四台电机 + 运行时间）
    ArmControl.traj.ctrl.u1_start = ArmControl.arm_motor.RotaryU1->data.position;
    ArmControl.traj.ctrl.rotary_z1_start = ArmControl.arm_motor.RotaryZ1->valReal.pos_deg;
    ArmControl.traj.ctrl.prismatic_z1_start = ArmControl.arm_motor.PrismaticZ1->valReal.pos_deg;
    ArmControl.traj.ctrl.dj_start = ArmControl.arm_motor.RotaryDJ->valNow.angle_deg;

    ArmControl.traj.ctrl.u1_target = pose.u1;
    ArmControl.traj.ctrl.rotary_z1_target = pose.rotary_z1;
    ArmControl.traj.ctrl.prismatic_z1_target = pose.prismatic_z1;
    ArmControl.traj.ctrl.dj_target = pose.dj;

    ArmControl.traj.ctrl.time       = 0.0f;
    ArmControl.traj.ctrl.total_time = pose.move_time;
    ArmControl.traj.ctrl.dt         = ARM_INTERPOLATION_DT;
    ArmControl.traj.mode            = ARM_MODE_DEFAULT;

    /* 不在此置 running/finish —— 由 Arm_Func 依据目标值变化沿决定 */
}


#if (USE_FK && USE_IK)
static void Arm_Interpolation_Cart_Start(Arm_Pose_t pose, float x, float y)
{
    Unitree_Theta_t now = {ArmControl.arm_motor.RotaryU1->data.position,
                           DEG2RAD(ArmControl.arm_motor.RotaryZ1->valReal.pos_deg)}; // TODO 第二关节暂按 GIM4310
    Vec2 target_pos = {x, y};
    Unitree_Theta_t target = Inverse(target_pos);
    pose.u1 = target.u1_theta;
    pose.rotary_z1 = RAD2DEG(target.u2_theta);
    ArmControl.traj.ctrl.u1_target = pose.u1;
    ArmControl.traj.ctrl.rotary_z1_target = pose.rotary_z1;
    ArmControl.traj.ctrl.prismatic_z1_target = pose.prismatic_z1;
    Vec2 now_pos = Forward(now);                       // 起点 = 当前真实末端

    ArmControl.pose = pose;                            // dj + move_time 来自表
    ArmControl.traj.ctrl.start_x  = now_pos.x;
    ArmControl.traj.ctrl.start_y  = now_pos.y;
    ArmControl.traj.ctrl.target_x = x;
    ArmControl.traj.ctrl.target_y = y;
    ArmControl.traj.ctrl.dj_start = ArmControl.arm_motor.RotaryDJ->valNow.angle_deg;
    ArmControl.traj.ctrl.dj_target = pose.dj;

    ArmControl.traj.ctrl.time = 0.0f;
    ArmControl.traj.ctrl.total_time = pose.move_time;
    ArmControl.traj.ctrl.dt = ARM_INTERPOLATION_DT;
    ArmControl.traj.mode = ARM_MODE_CARTESIAN;

    /* 不在此置 running/finish —— 由 Arm_Func 依据目标坐标变化沿决定 */
}
#endif /* (USE_FK && USE_IK) */


//轨迹点更新
static void Arm_Interpolation_Update(void)
{
    if (ArmControl.running == false)
    {
        return ;
    }

#if (USE_FK && USE_IK)
    float s = Quintic_S(ArmControl.traj.ctrl.time, ArmControl.traj.ctrl.total_time);

    if (ArmControl.traj.mode==ARM_MODE_CARTESIAN)
    {
        Vec2 p;
        p.x = ArmControl.traj.ctrl.start_x
            + (ArmControl.traj.ctrl.target_x - ArmControl.traj.ctrl.start_x) * s;
        p.y = ArmControl.traj.ctrl.start_y
            + (ArmControl.traj.ctrl.target_y - ArmControl.traj.ctrl.start_y) * s;

        //可达域检查
        float r = sqrtf(p.x * p.x + p.y * p.y);
        if (r > (ARM_U1_LENTH + ARM_U2_LENTH) ||
            r < fabsf(ARM_U1_LENTH - ARM_U2_LENTH))
        {
            ArmControl.running = false;
            ArmControl.finish  = true;
            return;
        }

        Unitree_Theta_t th = Inverse(p);
        ArmControl.arm_motor.RotaryU1->cmd.position = th.u1_theta;
        ArmControl.arm_motor.RotaryZ1->valSetNow.pos_deg = RAD2DEG(th.u2_theta); // TODO 第二关节暂按 GIM4310,待标定
        ArmControl.arm_motor.RotaryDJ->valSet.angle_deg = ArmControl.traj.ctrl.dj_start
            + (ArmControl.traj.ctrl.dj_target - ArmControl.traj.ctrl.dj_start) * s;
    }
    else
#endif
    {
        ArmControl.arm_motor.RotaryU1->cmd.position = Target_Quintic_Interpolation(ArmControl.traj.ctrl.u1_start,ArmControl.traj.ctrl.u1_target,ArmControl.traj.ctrl.time,ArmControl.traj.ctrl.total_time);
        ArmControl.arm_motor.RotaryZ1->valSetNow.pos_deg = Target_Quintic_Interpolation(ArmControl.traj.ctrl.rotary_z1_start,ArmControl.traj.ctrl.rotary_z1_target,ArmControl.traj.ctrl.time,ArmControl.traj.ctrl.total_time);
        ArmControl.arm_motor.PrismaticZ1->valSetNow.pos_deg = Target_Quintic_Interpolation(ArmControl.traj.ctrl.prismatic_z1_start,ArmControl.traj.ctrl.prismatic_z1_target,ArmControl.traj.ctrl.time,ArmControl.traj.ctrl.total_time);
        ArmControl.arm_motor.RotaryDJ->valSet.angle_deg = Target_Quintic_Interpolation(ArmControl.traj.ctrl.dj_start,ArmControl.traj.ctrl.dj_target,ArmControl.traj.ctrl.time,ArmControl.traj.ctrl.total_time);
    }

    ArmControl.traj.ctrl.time += ArmControl.traj.ctrl.dt;

    if (ArmControl.traj.ctrl.time >= ArmControl.traj.ctrl.total_time)
    {
        ArmControl.traj.ctrl.time = ArmControl.traj.ctrl.total_time;
#if (USE_FK && USE_IK)
        if (ArmControl.traj.mode==ARM_MODE_CARTESIAN)
        {
            Vec2 p_end;
            p_end.x = ArmControl.traj.ctrl.target_x;
            p_end.y = ArmControl.traj.ctrl.target_y;
            Unitree_Theta_t last = Inverse(p_end);
            ArmControl.arm_motor.RotaryU1->cmd.position = last.u1_theta;
            ArmControl.arm_motor.RotaryZ1->valSetNow.pos_deg = RAD2DEG(last.u2_theta); // TODO 第二关节暂按 GIM4310,待标定
        }
        else
#endif
        {
            ArmControl.arm_motor.RotaryU1->cmd.position = ArmControl.traj.ctrl.u1_target;
            ArmControl.arm_motor.RotaryZ1->valSetNow.pos_deg = ArmControl.traj.ctrl.rotary_z1_target;
            ArmControl.arm_motor.PrismaticZ1->valSetNow.pos_deg = ArmControl.traj.ctrl.prismatic_z1_target;
        }
        ArmControl.arm_motor.RotaryDJ->valSet.angle_deg = ArmControl.traj.ctrl.dj_target;
        ArmControl.running = false;
        ArmControl.finish = true;
    }

}


//机械臂位置、模式初始化
void Arm_Control_Init(void)
{
    //电机配置:把结构体成员指向真实电机对象
    ArmControl.arm_motor.RotaryU1 = &Unitree_motors[0];
    ArmControl.arm_motor.RotaryZ1 = &Zmotor[0];//GIM4310
    ArmControl.arm_motor.PrismaticZ1=&Zmotor[1];//U8Lite
    ArmControl.arm_motor.RotaryDJ = &DJmotor[1];

    /*
    ArmControl.arm_motor.RotaryU1->cmd.kp=;
    ArmControl.arm_motor.RotaryU1->cmd.kd=;
   */
    ArmControl.arm_motor.RotaryDJ->posPID.KP=0.3F;
    ArmControl.arm_motor.RotaryDJ->posPID.KI=0;
    ArmControl.arm_motor.RotaryDJ->posPID.KD=0.009F;
    ArmControl.arm_motor.RotaryDJ->velPID.KP=7.0F;
    ArmControl.arm_motor.RotaryDJ->velPID.KI=0.3F;
    ArmControl.arm_motor.RotaryDJ->velPID.KD=0.1F;

    ArmControl.arm_motor.RotaryZ1->param.kpPos=0.5F;
    ArmControl.arm_motor.RotaryZ1->param.kdPos=0.1F;

    ArmControl.arm_motor.RotaryZ1->param.kpVel=0.5F;
    ArmControl.arm_motor.RotaryZ1->param.kiVel=0;

    ArmControl.arm_motor.PrismaticZ1->param.kpPos=4.8F;
    ArmControl.arm_motor.PrismaticZ1->param.kdPos=0.001F;

    ArmControl.arm_motor.PrismaticZ1->param.kpVel=1.5F;
    ArmControl.arm_motor.PrismaticZ1->param.kiVel=0;






    ArmControl.enable    = false;
    ArmControl.reset     = false;
    ArmControl.state     = ARM_STATE_SILENT;
    ArmControl.req_state = ARM_STATE_SILENT;

    ArmControl.end_quarter_turns = 0;
    ArmControl.pose = Arm_Load_Pose(ARM_STATE_SILENT);

    ArmControl.traj.mode = ARM_MODE_DEFAULT;

    ArmControl.traj.ctrl.u1_start = 0.0f;
    ArmControl.traj.ctrl.rotary_z1_start = Arm_End_Target();
    ArmControl.traj.ctrl.prismatic_z1_start = 0.0f;
    ArmControl.traj.ctrl.dj_start = 0.0f;

    //目标值与初始状态(SILENT)配置一致,保证首次非SILENT命令能触发目标变化沿
    ArmControl.traj.ctrl.u1_target = Arm_Pose_Table[ARM_STATE_SILENT].u1;
    ArmControl.traj.ctrl.rotary_z1_target = Arm_End_Target();
    ArmControl.traj.ctrl.prismatic_z1_target = Arm_Pose_Table[ARM_STATE_SILENT].prismatic_z1;
    ArmControl.traj.ctrl.dj_target = Arm_Pose_Table[ARM_STATE_SILENT].dj;

    ArmControl.traj.ctrl.time       = 0.0f;
    ArmControl.traj.ctrl.total_time = ARM_MOVE_TIME;
    ArmControl.traj.ctrl.dt         = ARM_INTERPOLATION_DT;

#if (USE_FK && USE_IK)
    ArmControl.traj.ctrl.start_x  = 0.0f;
    ArmControl.traj.ctrl.start_y  = 0.0f;
    ArmControl.traj.ctrl.target_x = 0.0f;
    ArmControl.traj.ctrl.target_y = 0.0f;
#endif

    ArmControl.running = false;
    ArmControl.finish  = true;   //防止上电后立刻乱动

    Jaw_Init();
}



/* 仅三个独立末端旋转状态会更新理想离散角度。 */
static bool Arm_Is_End_Rotation_State(ArmState_t state)
{
    return (state == ARM_STATE_END_CW90) ||
           (state == ARM_STATE_END_CW180) ||
           (state == ARM_STATE_END_CCW90);
}

//机械臂运动更新(1kHz,运行于TIM2中断)
//1.使能沿收敛  2.复位则系统复位  3.夹爪继电器  4.未使能直接return
//5.模式切换变化沿检测(更新状态+配置)  6.运动更新(默认给转角/笛卡尔先规划再逆解)
void Arm_Func(void)
{
    static uint8_t s_motor_enabled = 0;

    /* 0) 未完成初始化(电机引用为空)直接返回:TIM2在Arm_Control_Init之前就已启动 */
    if (ArmControl.arm_motor.RotaryU1 == NULL ||
        ArmControl.arm_motor.RotaryZ1 == NULL ||
        ArmControl.arm_motor.PrismaticZ1 == NULL ||
        ArmControl.arm_motor.RotaryDJ == NULL)
    {
        return;
    }

    /* 1) 使能判断:先收敛电机使能沿 */
    if (ArmControl.enable)
    {
        if (s_motor_enabled == 0)
        {
            Arm_Motor_Enable();
            s_motor_enabled = 1;
        }
    }
    else
    {
        if (s_motor_enabled == 1)
        {
            Arm_Motor_Disable();
            s_motor_enabled = 0;
        }
    }

    /* 2) 复位:直接调用系统复位函数(不返回) */
    if (ArmControl.reset)
    {
        Sys_reset();
        return;
    }

    /* 未使能:不执行状态切换与运动更新,直接return */
    if (ArmControl.enable == false)
    {
        return;
    }
    bool target_changed = false;

    /* 3) 模式切换变化沿检测:新状态目标与当前目标不同才重新规划 */
    ArmState_t requested_state = ArmControl.req_state;
    if ((requested_state != ArmControl.state) &&
        ((unsigned int)requested_state < ARM_STATE_COUNT) &&
        ((unsigned int)requested_state != 8U))
    {
        Arm_Pose_t pose;
        bool rotation = Arm_Is_End_Rotation_State(requested_state);
        bool was_running = ArmControl.running;

        ArmControl.state = requested_state;
        if (rotation)
        {
            switch (requested_state)
            {
                case ARM_STATE_END_CW90:  ArmControl.end_quarter_turns = 1; break;
                case ARM_STATE_END_CCW90: ArmControl.end_quarter_turns = -1; break;
                case ARM_STATE_END_CW180: ArmControl.end_quarter_turns = 2; break;
                default: break;
            }
        }
        pose = Arm_Load_Pose(requested_state);
        if (rotation)
        {
            // 保留上一段位置目标；只改变 GIM 的理想绝对角度。
            pose.u1 = ArmControl.traj.ctrl.u1_target;
            pose.prismatic_z1 = ArmControl.traj.ctrl.prismatic_z1_target;
            pose.dj = ArmControl.traj.ctrl.dj_target;
            if (was_running)
            {
                pose.move_time = fmaxf(ArmControl.traj.ctrl.total_time -
                                      ArmControl.traj.ctrl.time,
                                      ARM_END_MIN_REPLAN_TIME);
            }
        }

#if (USE_FK && USE_IK)
        if (ArmControl.state == ARM_CartPos_DEBUG)
        {
            //笛卡尔模式:末端目标坐标发生改变才启动
            target_changed = (ArmControl.traj.mode != ARM_MODE_CARTESIAN) ||
                             (ArmControl.traj.ctrl.target_x != Arm_Pos_Debug.x) ||
                             (ArmControl.traj.ctrl.target_y != Arm_Pos_Debug.y);
            if (target_changed)
            {
                Arm_Interpolation_Cart_Start(pose, Arm_Pos_Debug.x, Arm_Pos_Debug.y);
            }
        }
        else
#endif
        {
            //默认模式:四台电机转角目标值发生改变才启动
            target_changed = (ArmControl.traj.mode != ARM_MODE_DEFAULT) ||
                             (ArmControl.traj.ctrl.u1_target != pose.u1) ||
                             (ArmControl.traj.ctrl.rotary_z1_target != pose.rotary_z1) ||
                             (ArmControl.traj.ctrl.prismatic_z1_target != pose.prismatic_z1) ||
                             (ArmControl.traj.ctrl.dj_target != pose.dj);
            if (target_changed)
            {
                Arm_Interpolation_Start(pose);
            }
        }
    }

    if (target_changed)
    {
        //目标变化:重新计时,以当前实际位置为起点运动到新目标
        ArmControl.traj.ctrl.time = 0.0f;
        ArmControl.running = true;
        ArmControl.finish  = false;
    }

    /* 4) 运动更新 */
    if (ArmControl.running)
    {
        Arm_Interpolation_Update();
    }

#if USE_FK
    /* 运动学遥测(1kHz刷新,供VOFA/调试器观察) */
    Arm_Kinetics_Data.motor_target = Arm_Debug;
    Arm_Kinetics_Data.arm_angle.Angle1 = U1_Motor2Geom(ArmControl.arm_motor.RotaryU1->data.position);
    Arm_Kinetics_Data.arm_angle.Angle2 = U2_Motor2Geom(ArmControl.arm_motor.RotaryU1->data.position,
                                                       DEG2RAD(ArmControl.arm_motor.RotaryZ1->valReal.pos_deg)); // TODO 第二关节暂按 GIM4310
    Arm_Kinetics_Data.forward_angle.u1_theta = ArmControl.arm_motor.RotaryU1->data.position;
    Arm_Kinetics_Data.forward_angle.u2_theta = DEG2RAD(ArmControl.arm_motor.RotaryZ1->valReal.pos_deg); // TODO 第二关节暂按 GIM4310
    Arm_Kinetics_Data.end_coordinate = Forward(Arm_Kinetics_Data.forward_angle);
#endif
#if (USE_FK && USE_IK)
    Arm_Kinetics_Data.inverse_angle = Inverse(Arm_Kinetics_Data.end_coordinate);
#endif
}








void Arm_Motor_Enable(void)
{

    ArmControl.arm_motor.RotaryU1->enable = true;

    ArmControl.arm_motor.RotaryDJ->Begin = true;
    ArmControl.arm_motor.RotaryDJ->MODE_Set = DJ_Position;

    ArmControl.arm_motor.PrismaticZ1->Begin=true;
    ArmControl.arm_motor.PrismaticZ1->mode=Zdrive_Postion;

    ArmControl.arm_motor.RotaryZ1->Begin=true;
    ArmControl.arm_motor.RotaryZ1->mode=Zdrive_Postion;



}


void Arm_Motor_Disable(void)
{
    ArmControl.arm_motor.RotaryU1->enable = false;

    ArmControl.arm_motor.RotaryDJ->MODE_Set = DJ_Disable;
    ArmControl.arm_motor.RotaryDJ->Begin = false;

    ArmControl.arm_motor.RotaryZ1->mode = Zdrive_Disable;
    ArmControl.arm_motor.PrismaticZ1->mode = Zdrive_Disable;
}


//状态命令表:主控板下发 CANID -> 机械臂请求状态
//CANID 未分配(宏值为 0)的条目自动跳过
static const struct
{
    uint32_t   can_id;
    ArmState_t state;
}s_arm_state_canid_map[] =
{
    {ARM_CANID_STATE_SILENT,         ARM_STATE_SILENT},
    {ARM_CANID_STATE_SKY_PRE_H,      ARM_STATE_SKY_PRE_H},
    {ARM_CANID_STATE_SKY_PRE_V,      ARM_STATE_SKY_PRE_V},
    {ARM_CANID_STATE_SKY_PICK_H,     ARM_STATE_SKY_PICK_H},
    {ARM_CANID_STATE_SKY_PICK_V,     ARM_STATE_SKY_PICK_V},
    {ARM_CANID_STATE_END_CW90,       ARM_STATE_END_CW90},
    {ARM_CANID_STATE_END_CW180,      ARM_STATE_END_CW180},
    {ARM_CANID_STATE_END_CCW90,      ARM_STATE_END_CCW90},
    {ARM_CANID_STATE_SKY_KEEP,       ARM_STATE_SKY_KEEP},
    {ARM_CANID_STATE_SKY_PLACE,      ARM_STATE_SKY_PLACE},
    {ARM_CANID_STATE_BALL_PRE,       ARM_STATE_BALL_PRE},
    {ARM_CANID_STATE_BALL_PICK,      ARM_STATE_BALL_PICK},
    {ARM_CANID_STATE_BALL_PLACE,     ARM_STATE_BALL_PLACE},
};


//上位机命令接收(FDCAN中断):判断接收的CANID得到对应的请求状态,由Arm_Func完成变化沿处理
void Arm_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data)
{
    if (Rxheader.IdType != FDCAN_EXTENDED_ID)
    {
        return;
    }

    //使能/失能 与 系统复位 不受使能状态限制
    switch (Rxheader.Identifier)
    {
        //使能失能
        case ARM_CANID_ENABLE:

            if (Rx_data[0] == 'E')
            {
                ArmControl.enable = true;
            }
            else if (Rx_data[0] == 'D')
            {
                ArmControl.enable = false;
            }

            break;

        //系统复位
        case ARM_CANID_RESET:

            if (Rx_data[0] == 'R')
            {
                ArmControl.reset = true;
            }

            break;

        default:

            break;
    }

    if (ArmControl.enable == false)
    {
        return;
    }

    //状态命令:CANID 命中即置请求状态
    for (uint32_t i = 0U; i < (sizeof(s_arm_state_canid_map) / sizeof(s_arm_state_canid_map[0])); i++)
    {
        if ((s_arm_state_canid_map[i].can_id != 0U) &&
            (Rxheader.Identifier == s_arm_state_canid_map[i].can_id))
        {
            ArmControl.req_state = s_arm_state_canid_map[i].state;
            break;
        }
    }
}




void Arm_Transmit(void)
{
    static uint8_t tx_data[1] = {0};
    FDCAN_TxHeaderTypeDef tx_header = {0};

    tx_header.Identifier = 0x03020101U;

    tx_header.IdType = FDCAN_EXTENDED_ID;

    tx_header.TxFrameType = FDCAN_DATA_FRAME;

    tx_header.DataLength = FDCAN_DLC_BYTES_1;

    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    tx_data[0] = (uint8_t)ArmControl.state;

//    if( HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&tx_header,tx_data)!=HAL_OK)
//    {
//         Is_ok=1;
//    }
//    HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
// if (ret != HAL_OK) {
//     // 通过串口或调试器查看 ret 的值
//     Is_ok = ret;  // 把具体错误码存下来
// }

HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_header, tx_data);
if (ret != HAL_OK) {
    Is_ok = ret;
    HAL_FDCAN_StateTypeDef state = hfdcan3.State;   // 观察值：0=RESET, 1=READY, 2=LISTENING, 4=ERROR
    uint32_t err = HAL_FDCAN_GetError(&hfdcan3); // 读取硬件错误码
    // 可通过串口打印出来，或存储在全局变量中调试
}

}
