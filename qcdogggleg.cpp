#include "qcdogggleg.h"

QCDogggLeg::QCDogggLeg(MOTOR_send *ps1, MOTOR_send *ps2, MOTOR_recv *pr1,
                       MOTOR_recv *pr2, LegType type)

    // 初始化变量
    : StepDistance(30),                                   // mm
      StepDesiredDistance(StepDistance), StepHeight(-25), // mm
      StepDesiredHeight(StepHeight), StepStandingHeight(100),
      StepTotalCycle(1.f), // s
      StepCurrentTime(0)

{
  DesiredHeightError = 0;

  // 外面传进来的电机指针
  motorSendA = ps1;
  motorSendB = ps2;

  motorRecvA = pr1;
  motorRecvB = pr2;

  DesiredPosA = 0.0f;
  DesiredPosB = 0.0f;

  InitDesiredPosA = 0.0f;
  InitDesiredPosB = 0.0f;

  ErrorPosA = 0.0f;
  ErrorPosB = 0.0f;

  legType = type;

  // 外面传进来的 腿类型，设定类型
  switch (type) {
  case LeftFrontLeg:
    motorSendA->id = 0;
    motorSendB->id = 1;
    break;
  case RightFrontLeg:
    motorSendA->id = 0;
    motorSendB->id = 1;
    break;
  case LeftHindLeg:
    motorSendA->id = 2;
    motorSendB->id = 0;
    break;
  case RightHindLeg:
    motorSendA->id = 2;
    motorSendB->id = 1;
    break;
  }

  motorSendA->K_P = kp;
  motorSendA->K_W = kw;

  motorSendB->K_P = kp;
  motorSendB->K_W = kw;
}

// python 计算
// 4.02529176036953e-09 * x ** 4 + -1.02267416878680e-05 * x ** 3 +
// 0.00743558748214280 * x ** 2 + -0.245630720143646 * x + 4.53950909717144

#include <stdio.h>

// 输入一个 0-1000 的值，可以计算出 这个坐标图形， 也就是图形轨迹
void QCDogggLeg::setCurrentTimePoint(int ms) {
  // 非线性化的时间函数

  ms = (/*4.02529176036953e-09*/ -1.08814372330426e-11) * pow(ms, 5) +
       (/*-1.02267416878680e-05*/ 2.49131119441928e-08) * pow(ms, 4) +
       (/*0.00743558748214280*/ -1.90727872499507e-05) * pow(ms, 3) +
       (/*-0.245630720143646*/ 0.00580969715599520) * pow(ms, 2) +
       (/*-0.245630720143646*/ 0.228515286424892) * pow(ms, 1) +
       (/*4.53950909717144*/ 0.396124750148504);

  StepCurrentTime = ms * 0.001f;

  if (legType == LeftFrontLeg || legType == LeftHindLeg) {
    if (ms <= 500)
      LegSwing(StepCurrentTime, 1);
    else
      LegSupport(StepCurrentTime, 1);
  } else {
    if (ms <= 500)
      LegSwing(StepCurrentTime, 0);
    else
      LegSupport(StepCurrentTime, 0);
  }
}

// 支撑相 图形轨迹计算
void QCDogggLeg::LegSupport(float t, int side) {
  float temp = 1; // 零时变量
  float TB = StepTotalCycle / 2;

  // 限制周期在 ( 0 , T/2 ) 内
  if (t > TB)
    t = t - TB;

  // side = 0 右腿    side = 1 左腿
  if (side) {
    if (t <= TB / 2) {
      temp = -1;
    }
  } else {
    if (t >= TB / 2) {
      temp = -1;
      t = TB - t;
    } else {
      t = TB - t;
    }
  }

  // 计算所需中间变量
  float f = t / TB - 1 / (4 * pi) * sin(4 * pi * t / TB);
  float f2 = 2 * f - 1;
  float f3 = TB / 2 - t;
  float f4 = f2 * temp + 1;

  DesiredFootEndX =
      StepDistance * (t / TB - (1 / (2 * pi)) * sin(2 * pi * t / TB)) +
      2 * (side ? -1 : 0) * StepDistance + DesiredFootEndXError +
      OriginalOffsetX;
  DesiredFootEndZ = StepStandingHeight + DesiredFootEndZError + OriginalOffsetZ;

  if (side) {
    DesiredFootEndX += StepDistance;
  } else {
    DesiredFootEndX += 0;
  }
}

// 摆动相 图形轨迹计算
void QCDogggLeg::LegSwing(float t, int side) {
  float temp = 1; // 零时变量
  float TB = StepTotalCycle / 2;

  // 限制周期在 ( 0 , T/2 ) 内
  if (t > TB)
    t = t - TB;

  // side = 0 右腿    side = 1 左腿
  if (side) {
    if (t < TB / 2) {
      temp = -1;
      t = TB - t;
    } else {
      t = TB - t;
    }
  } else {
    if (t > TB / 2) {
      temp = -1;
    }
  }

  // 计算所需中间变量
  float f = t / TB - 1 / (4 * pi) * sin(4 * pi * t / TB);
  float f2 = 2 * f - 1;
  float f3 = TB / 2 - t;
  float f4 = f2 * temp + 1;

  DesiredFootEndX =
      StepDistance * (t / TB - (1 / (2 * pi)) * sin(2 * pi * t / TB)) +
      2 * (side ? -1 : 0) * StepDistance + DesiredFootEndXError +
      OriginalOffsetX;
  DesiredFootEndZ = (StepHeight + DesiredHeightError) * f4 +
                    StepStandingHeight + DesiredFootEndZError + OriginalOffsetZ;

  if (side) {
    DesiredFootEndX += StepDistance;
  } else {
    DesiredFootEndX += 0;
  }
}

// 应用逆解，就是把坐标转换为 电机转角
void QCDogggLeg::AppliedInverseSolution() {

  switch (legType) {
  case LeftFrontLeg:
    AppliedInverseSolutionLeft(DesiredFootEndX, DesiredFootEndZ);
    break;

  case RightFrontLeg:
    AppliedInverseSolutionRight(DesiredFootEndX, DesiredFootEndZ);
    break;

  case LeftHindLeg:
    AppliedInverseSolutionLeft(DesiredFootEndX, DesiredFootEndZ);
    break;

  case RightHindLeg:
    AppliedInverseSolutionRight(DesiredFootEndX, DesiredFootEndZ);
    break;
  }
}

void QCDogggLeg::AppliedInverseSolutionLeft(float x, float z) {
  float c2 = (x * x + z * z - hu * hu - hl * hl) / (2 * hu * hl);
  float temp2 = 1 - c2 * c2;

  if (temp2 < 0)
    temp2 = 0;

  DesiredPosB = atan2(sqrt(temp2), c2);

  float s2 = sin(DesiredPosB);
  DesiredPosA = atan2(z, x) - atan2(hl * s2, hu + hl * c2);

  DesiredPosNoErrorA = -DesiredPosA;
  DesiredPosNoErrorB = 3.1415926f - DesiredPosB;

  DesiredPosA = DesiredPosNoErrorA - ErrorPosA;
  DesiredPosB = DesiredPosNoErrorB - ErrorPosB;

  // 9.1 是减速比  0.9是 传动比

  motorSendA->Pos = DesiredPosA * 9.1f;
  motorSendB->Pos = DesiredPosB * 9.1f;

  // motorSendA->T = VirtualTorqueA;
  // motorSendB->T = -VirtualTorqueB;
}

void QCDogggLeg::AppliedInverseSolutionRight(float x, float z) {
  x = -x;

  float c2 = (x * x + z * z - hu * hu - hl * hl) / (2 * hu * hl);
  float temp2 = 1 - c2 * c2;

  if (temp2 < 0)
    temp2 = 0;

  DesiredPosB = atan2(sqrt(temp2), c2);

  float s2 = sin(DesiredPosB);
  DesiredPosA = atan2(z, x) - atan2(hl * s2, hu + hl * c2);

  DesiredPosNoErrorA = DesiredPosA;
  DesiredPosNoErrorB = -3.1415926f + DesiredPosB;

  DesiredPosA = DesiredPosNoErrorA - ErrorPosA;
  DesiredPosB = DesiredPosNoErrorB - ErrorPosB;

  // 9.1 是减速比  0.9是 传动比

  motorSendA->Pos = DesiredPosA * 9.1f;
  motorSendB->Pos = DesiredPosB * 9.1f;

  // motorSendA->T = -VirtualTorqueA;
  // motorSendB->T = VirtualTorqueB;
}

// 正解求出足端坐标
void QCDogggLeg::AppliedPostiveSolution() {
  ActualPosA = motorRecvA->Pos / 9.1f + ErrorPosA;
  ActualPosB = motorRecvB->Pos / 9.1f + ErrorPosB;

  if (legType == LeftFrontLeg || legType == LeftHindLeg) {
    ActualPosA = -ActualPosA;
    ActualPosB = -(ActualPosB - 3.1415926f);
  } else {
    ActualPosA = ActualPosA;
    ActualPosB = (ActualPosB + 3.1415926f);
  }

  ActualFootEndX = hu * cos(ActualPosA) + hl * cos(ActualPosA + ActualPosB);
  ActualFootEndZ = hu * sin(ActualPosA) + hl * sin(ActualPosA + ActualPosB);
}

// 返回总时间
float QCDogggLeg::getTotalCycle() { return StepTotalCycle; }

// 返回当前进度
float QCDogggLeg::getCurrentTime() { return StepCurrentTime; }

// 计算出虚拟力
void QCDogggLeg::AppliedVirtualForce(float dt) {
  //    //暂时用这个坐标
  //    ActualFootEndX = DesiredFootEndX;
  //    ActualFootEndZ = DesiredFootEndZ;

  // 这个可能还没做好，不太对 todo

  static float ActualFootEndXLast = ActualFootEndX;
  static float ActualFootEndZLast = ActualFootEndZ;

  float CalHeight = 0;

  // 只是为了后半段减速
  if (StepCurrentTime > 0.25f && StepCurrentTime < 0.5f) {
    CalHeight = StepStandingHeight;
  } else {
    CalHeight = DesiredFootEndZ;
  }

  float FootEndSpeedX = (ActualFootEndXLast - ActualFootEndX) * dt;
  float FootEndSpeedZ = (ActualFootEndZLast - ActualFootEndZ) * dt;

  // pd控制器
  AppliedForceX = ForceKd * (0 - FootEndSpeedX) +
                  ForceKp * (DesiredFootEndX - ActualFootEndX);
  AppliedForceZ =
      ForceKd * (0 - FootEndSpeedZ) + ForceKp * (CalHeight - ActualFootEndZ);

  // 以下方程采用理论力学的方法，这里的theta 跟 前面电机的theta不一样
  float theta1, theta2;

  theta1 = atan(fabs(ActualFootEndZ / ActualFootEndX));
  theta2 = 1.5707963 - (ActualPosB - ActualPosA);

  // 足端中心到大腿电机中心的长度，
  float absLen = fabs(
      sqrt(ActualFootEndX * ActualFootEndX + ActualFootEndZ * ActualFootEndZ));

  VirtualTorqueA = -(AppliedForceZ * cos(theta1) * absLen +
                     AppliedForceX * sin(theta1) * absLen);
  VirtualTorqueB =
      -(AppliedForceZ * cos(theta2) * hl + AppliedForceX * sin(theta2) * hl);

  ActualFootEndXLast = ActualFootEndX;
  ActualFootEndZLast = ActualFootEndZ;
}
