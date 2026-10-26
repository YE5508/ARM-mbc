"""Run the real arm controller on a host GCC with hardware-only stubs.

Usage: python Tools/tests/test_arm_states.py [--cc gcc]
Sources and alternate configs are copied to a temporary directory; firmware
configuration and tracked build files are never modified by these tests.
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

HARDWARE = r"""
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
typedef struct { float KP, KI, KD; } PID;
typedef struct {
    struct { float position; } data;
    struct { float position; } cmd;
    bool enable;
} UnitreeMotor;
typedef struct {
    PID posPID, velPID;
    struct { float angle_deg; } valNow, valSet;
    bool Begin;
    int MODE_Set;
} DJMotor;
typedef struct {
    struct { float kpPos, kdPos, kpVel, kiVel; } param;
    struct { float pos_deg; } valReal, valSetNow;
    bool Begin;
    int mode;
} Zdrive;
extern UnitreeMotor Unitree_motors[1];
extern DJMotor DJmotor[2];
extern Zdrive Zmotor[2];
enum { DJ_Position, DJ_Disable, Zdrive_Postion, Zdrive_Disable };
typedef struct { uint32_t Identifier, IdType; } FDCAN_RxHeaderTypeDef;
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
    uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef int HAL_StatusTypeDef;
typedef int HAL_FDCAN_StateTypeDef;
typedef struct { int State; } FDCAN_HandleTypeDef;
extern FDCAN_HandleTypeDef hfdcan3;
#define HAL_OK 0
#define FDCAN_EXTENDED_ID 1
#define FDCAN_DATA_FRAME 0
#define FDCAN_DLC_BYTES_1 1
#define FDCAN_ESI_ACTIVE 0
#define FDCAN_BRS_OFF 0
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_NO_TX_EVENTS 0
#define GPIOA 0
#define GPIO_PIN_9 9
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
static inline void __disable_irq(void) {}
static inline void NVIC_SystemReset(void) {}
static inline void HAL_GPIO_WritePin(int a, int b, int c) {(void)a;(void)b;(void)c;}
static inline void Jaw_Init(void) {}
static inline int HAL_FDCAN_AddMessageToTxFifoQ(
    FDCAN_HandleTypeDef *h, FDCAN_TxHeaderTypeDef *t, uint8_t *d)
    {(void)h;(void)t;(void)d;return HAL_OK;}
static inline uint32_t HAL_FDCAN_GetError(FDCAN_HandleTypeDef *h) {(void)h;return 0;}
#define RAD2DEG(x) ((x) * 57.295779513f)
#define DEG2RAD(x) ((x) / 57.295779513f)
"""

HARNESS = r"""
#include <assert.h>
#include <stdio.h>
#include "seize_sky.c"
#include "expected.h"
UnitreeMotor Unitree_motors[1];
DJMotor DJmotor[2];
Zdrive Zmotor[2];
FDCAN_HandleTypeDef hfdcan3;
static void check_near(float a, float b, int line) {
    if (fabsf(a-b) >= 0.002f) {
        fprintf(stderr, "line %d: actual=%f expected=%f\n", line, a, b);
        assert(0);
    }
}
#define near(a,b) check_near((a),(b),__LINE__)
static float ideal(int n) {
    return expected_end[n+1];
}
static void request(ArmState_t state) { ArmControl.req_state=state; Arm_Func(); }
static void setup(void) {
    Arm_Control_Init(); ArmControl.enable=true;
    Arm_Pose_Table[ARM_STATE_SKY_KEEP]=(Arm_Pose_Config_t){1,20,30,10};
    request(ARM_STATE_SKY_KEEP);
    assert(ArmControl.running && !ArmControl.finish);
}
static void finish(void) {
    int count=0;
    while (ArmControl.running && count++ < 40000) Arm_Func();
    assert(!ArmControl.running && ArmControl.finish);
}
static void positions(void) {
    near(ArmControl.traj.ctrl.u1_target,1);
    near(ArmControl.traj.ctrl.prismatic_z1_target,20);
    near(ArmControl.traj.ctrl.dj_target,30);
}
int main(void) {
    assert(ARM_STATE_END_CW90==5 && ARM_STATE_END_CW180==6);
    assert(ARM_STATE_END_CCW90==7 && ARM_STATE_SKY_PLACE==9);
    assert(ARM_STATE_BALL_PLACE==12 && ARM_STATE_SKY_KEEP==13);
    setup();
    near(ArmControl.pose.rotary_z1,ideal(0));
    ArmControl.traj.ctrl.time=4;
    Unitree_motors[0].data.position=0.4f;
    Zmotor[1].valReal.pos_deg=8;
    DJmotor[1].valNow.angle_deg=12;
    Zmotor[0].valReal.pos_deg=ideal(0)+3; // Deliberate feedback error.
    request(ARM_STATE_END_CW90);
    positions();
    near(ArmControl.pose.move_time,6);
    near(ArmControl.traj.ctrl.total_time,6);
    near(ArmControl.traj.ctrl.time,0.001f);
    near(ArmControl.traj.ctrl.u1_start,0.4f);
    near(ArmControl.traj.ctrl.rotary_z1_start,ideal(0)+3);
    near(ArmControl.pose.rotary_z1,ideal(1));
    near(Arm_Pose_Table[ARM_STATE_END_CW90].move_time,10);
    near(Arm_Pose_Table[ARM_STATE_SKY_KEEP].move_time,10);
    ArmControl.traj.ctrl.time=5.5f;
    request(ARM_STATE_END_CCW90);
    positions(); near(ArmControl.pose.move_time,1.5f);
    near(ArmControl.pose.rotary_z1,ideal(-1));
    ArmControl.traj.ctrl.time=0.25f;
    request(ARM_STATE_END_CW180);
    positions(); near(ArmControl.pose.move_time,1.5f);
    near(ArmControl.pose.rotary_z1,ideal(2));
    request(ARM_STATE_END_CW180); // Same state must not restart.
    near(ArmControl.traj.ctrl.time,0.002f);
    finish();
    near(Unitree_motors[0].cmd.position,1);
    near(Zmotor[1].valSetNow.pos_deg,20);
    near(DJmotor[1].valSet.angle_deg,30);
    near(Zmotor[0].valSetNow.pos_deg,ideal(2));

    // Idle rotation holds the three positional commands despite feedback errors.
    request(ARM_STATE_END_CCW90);
    near(ArmControl.pose.move_time,10);
    for (int i=0;i<100;i++) {
        Arm_Func();
        near(Unitree_motors[0].cmd.position,1);
        near(Zmotor[1].valSetNow.pos_deg,20);
        near(DJmotor[1].valSet.angle_deg,30);
    }
    finish();

    // Position states retain the ideal endpoint, never the measured GIM angle.
    Arm_Pose_Table[ARM_STATE_SKY_PLACE]=(Arm_Pose_Config_t){1,20,30,10};
    Zmotor[0].valReal.pos_deg=ideal(-1)+7;
    request(ARM_STATE_SKY_PLACE);
    assert(!ArmControl.running);
    near(ArmControl.traj.ctrl.rotary_z1_target,ideal(-1));
    Arm_Pose_Table[ARM_STATE_BALL_PRE]=(Arm_Pose_Config_t){2,40,60,10};
    request(ARM_STATE_BALL_PRE);
    near(ArmControl.pose.rotary_z1,ideal(-1));
    ArmControl.traj.ctrl.time=4;
    request(ARM_STATE_END_CCW90); // Different state, same angle: keep trajectory.
    near(ArmControl.traj.ctrl.time,4.001f);
    near(ArmControl.traj.ctrl.total_time,10);
    request((ArmState_t)8); // Reserved slot must not interrupt interpolation.
    assert(ArmControl.state==ARM_STATE_END_CCW90);
    request((ArmState_t)-1);
    assert(ArmControl.state==ARM_STATE_END_CCW90);
    finish();

#if USE_FK
    Arm_Debug.rotary_z1_target=17;
    request(ARM_DEBUG);
    near(ArmControl.pose.rotary_z1,17);
    assert(ArmControl.end_quarter_turns==-1);
    request(ARM_STATE_SKY_KEEP);
    near(ArmControl.pose.rotary_z1,ideal(-1));
    assert(ArmControl.traj.mode==ARM_MODE_DEFAULT);
#endif
#if USE_IK
    Arm_Pos_Debug.x=400; Arm_Pos_Debug.y=100;
    request(ARM_Pos_DEBUG);
    assert(isfinite(ArmControl.pose.rotary_z1));
    assert(ArmControl.end_quarter_turns==-1);
    request(ARM_STATE_SKY_KEEP);
    near(ArmControl.pose.rotary_z1,ideal(-1));
#endif
#if (USE_FK && USE_IK)
    request(ARM_CartPos_DEBUG);
    assert(ArmControl.traj.mode==ARM_MODE_CARTESIAN);
    float u1_target=ArmControl.traj.ctrl.u1_target;
    float prism_target=ArmControl.traj.ctrl.prismatic_z1_target;
    request(ARM_STATE_END_CW90);
    assert(ArmControl.traj.mode==ARM_MODE_DEFAULT);
    near(ArmControl.traj.ctrl.u1_target,u1_target);
    near(ArmControl.traj.ctrl.prismatic_z1_target,prism_target);
    near(ArmControl.pose.rotary_z1,ideal(1));
#endif
    // A fresh 10s position move with 9s elapsed clamps to 1.5s.
    setup(); ArmControl.traj.ctrl.time=9;
    request(ARM_STATE_END_CW90);
    near(ArmControl.pose.move_time,1.5f);
    puts("arm controller scenarios passed");
    return 0;
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    args = parser.parse_args()
    compiler = shutil.which(args.cc)
    if compiler is None:
        parser.error(f"C compiler not found: {args.cc}")
    with tempfile.TemporaryDirectory(prefix="arm-state-tests-") as directory:
        temp = Path(directory)
        for name in ("seize_sky.h", "kinematics.h"):
            shutil.copyfile(ROOT / "User/inc" / name, temp / name)
        for name in ("seize_sky.c", "kinematics.c"):
            shutil.copyfile(ROOT / "User/src" / name, temp / name)
        (temp / "hardware.h").write_text(HARDWARE, encoding="utf-8")
        for name in ("main.h", "UnitreeMotor.h", "DJmotor.h", "ZDrive.h",
                     "cmsis_os.h", "gpio.h", "fdcan.h", "mathFunc.h", "holding_jaw.h"):
            (temp / name).write_text('#include "hardware.h"\n', encoding="utf-8")
        (temp / "test.c").write_text(HARNESS, encoding="utf-8")
        config = (ROOT / "User/inc/sky_config.h").read_text(encoding="utf-8")
        for fk, ik in ((0, 0), (1, 0), (0, 1), (1, 1)):
            # Independently specified absolute endpoints, deliberately asymmetric.
            for targets in ((-929.337646, 0, 912.406311, 1802.16016),
                            (-923, 12, 907, 1795)):
                variant = re.sub(r"(#define USE_FK) 0", rf"\g<1> {fk}", config)
                variant = re.sub(r"(#define USE_IK) 0", rf"\g<1> {ik}", variant)
                for name, value in zip(("CCW90", "ZERO", "CW90", "CW180"), targets):
                    variant = re.sub(rf"(#define ARM_END_{name}_POS_DEG\s+)[^\n]+",
                                     rf"\g<1>({value:.6f}f)", variant)
                values = ", ".join(f"{value:.6f}f" for value in targets)
                (temp / "expected.h").write_text(
                    f"static const float expected_end[] = {{{values}}};\n",
                    encoding="utf-8")
                (temp / "sky_config.h").write_text(variant, encoding="utf-8")
                executable = temp / "test.exe"
                subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                                # Existing Arm_Transmit debug locals are unused.
                                "-Wno-unused-variable", str(temp / "test.c"),
                                str(temp / "kinematics.c"), "-lm", "-o", str(executable)],
                               check=True)
                subprocess.run([str(executable)], check=True)
                print(f"PASS FK={fk} IK={ik} targets={targets}", flush=True)


if __name__ == "__main__":
    main()
