#ifndef SKY_CONFIG_H
#define SKY_CONFIG_H

/**
 * @file    sky_config.h
 * @brief   机械臂(天空块)业务配置:运动学调试开关 + 主控板命令 CANID。
 *
 * USE_FK / USE_IK 控制正/逆运动学相关代码是否参与编译:
 *   USE_FK = 1        -> kinematics 正解(Forward/几何角换算)与 ARM_DEBUG 调试状态参与编译
 *   USE_IK = 1        -> kinematics 逆解(Inverse)与 ARM_Pos_DEBUG 调试状态参与编译
 *   USE_FK && USE_IK  -> ARM_CartPos_DEBUG(笛卡尔轨迹 + 逐周期逆解)参与编译
 * 默认全 0:新机构(已非双宇树 2R 模型)标定完成前,相关代码不参与编译。
 */
#define USE_FK 0
#define USE_IK 0

/* 各离散角度独立标定的绝对 pos_deg，不再使用统一比例换算。 */
#define ARM_END_ZERO_POS_DEG       0.0f
#define ARM_END_CCW90_POS_DEG     (-929.337646f)
#define ARM_END_CW90_POS_DEG       912.406311f
#define ARM_END_CW180_POS_DEG      1802.16016f
#define ARM_END_MIN_REPLAN_TIME    1.5f /* 运动中加入旋转的最短剩余时间(s) */

/* ------------------------------------------------------------------ */
/* 主控板状态命令 CANID(FDCAN3 扩展帧)                                 */
/* 待主控板协议确定后填入,0 = 未分配(Arm_Receive 自动跳过该条目)       */
/* ------------------------------------------------------------------ */
#define ARM_CANID_STATE_SILENT          0x00000000U /* 静默状态             TODO */
#define ARM_CANID_STATE_SKY_PRE_H       0x00000000U /* 横预取天空块         TODO */
#define ARM_CANID_STATE_SKY_PRE_V       0x00000000U /* 竖预取天空块         TODO */
#define ARM_CANID_STATE_SKY_PICK_H      0x00000000U /* 横取天空块           TODO */
#define ARM_CANID_STATE_SKY_PICK_V      0x00000000U /* 竖取天空块           TODO */
#define ARM_CANID_STATE_END_CW90        0x00000000U /* 末端顺时针90°       TODO */
#define ARM_CANID_STATE_END_CW180       0x00000000U /* 末端顺时针180°      TODO */
#define ARM_CANID_STATE_END_CCW90       0x00000000U /* 末端逆时针90°       TODO */
#define ARM_CANID_STATE_SKY_KEEP        0x00000000U /* 持天空块位置         TODO */
#define ARM_CANID_STATE_SKY_PLACE       0x00000000U /* 放置天空块           TODO */
#define ARM_CANID_STATE_BALL_PRE        0x00000000U /* 预取球               TODO */
#define ARM_CANID_STATE_BALL_PICK       0x00000000U /* 取球                 TODO */
#define ARM_CANID_STATE_BALL_PLACE      0x00000000U /* 放置球               TODO */

/* 使能/失能 与 系统复位(沿用旧协议,不随状态改造变化) */
#define ARM_CANID_ENABLE           0x01020211U /* 数据 'E' 使能 / 'D' 失能 */
#define ARM_CANID_RESET            0x010202F0U /* 数据 'R' 系统复位 */

#endif /* SKY_CONFIG_H */
