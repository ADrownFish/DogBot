#ifndef QCDOGGGLEG_H
#define QCDOGGGLEG_H

#include "qcdogggbotconfiguration.h"
#include <math.h>

enum LegType { LeftFrontLeg, RightFrontLeg, LeftHindLeg, RightHindLeg };

class QCDogggLeg {
public:
  QCDogggLeg(MOTOR_send *ps1, MOTOR_send *ps2, MOTOR_recv *pr1, MOTOR_recv *pr2,
             LegType type);

  // 设置当前的时间点
  void setCurrentTimePoint(int ms);
  // 应用逆解 求出 电机转角
  void AppliedInverseSolution();

  // 获取腿的总周期 秒
  float getTotalCycle();
  // 获取当前的时间点 秒
  float getCurrentTime();

  //=========================内部计算变量=========================
public:
  // 期望的足端坐标 计算而来
  float DesiredFootEndX;
  float DesiredFootEndZ;

  // 实际的足端坐标,正解而来
  float ActualFootEndX;
  float ActualFootEndZ;

  // 每只腿的位置补偿
  float DesiredFootEndXError;
  float DesiredFootEndZError;

  // 步高补偿
  float DesiredHeightError;

  // 原点偏移，是以当前姿态的增量
  float OriginalOffsetX;
  float OriginalOffsetZ;

  // 经过计算的出的 期望转角位置，但没有加上补偿值
  float DesiredPosNoErrorA;
  float DesiredPosNoErrorB;

  // 经过计算的出的 期望转角位置
  float ActualPosA;
  float ActualPosB;

  // 经过计算的出的 期望转角位置
  float DesiredPosA;
  float DesiredPosB;

  // 初始化时  站立的转角
  float InitDesiredPosA;
  float InitDesiredPosB;

  // 大腿和小腿电机 的位置误差
  float ErrorPosA;
  float ErrorPosB;

  // 以防程序bug
  // 导致烧毁电机，这里要对输出进行限幅，不在这个范围内，那就不发出指令
  // 注意：这个值是 乘传动比 之后的值
  float AOutPutUpperLimit;
  float AOutPutLowerLimit;

  float BOutPutUpperLimit;
  float BOutPutLowerLimit;

  // 虚拟扭矩,直接作用与电机上
  float VirtualTorqueA = 0;
  float VirtualTorqueB = 0;

  // 腿受到的力
  float AppliedForceX = 0;
  float AppliedForceZ = 0;

  // 腿的类型
  LegType legType;

  // 两个电机的 发送 和 接受结构体指针
  MOTOR_send *motorSendA;
  MOTOR_send *motorSendB;

  MOTOR_recv *motorRecvA;
  MOTOR_recv *motorRecvB;

  // 步距离 步高度 步站立时高度      步整个周期 步当前的时间

  float StepDistance;        // 实际步距
  float StepDesiredDistance; // 期望步距

  float StepHeight;
  float StepDesiredHeight; // 期望步高

  float StepStandingHeight;

  float StepTotalCycle;
  float StepCurrentTime;

  // 大腿的长度 小腿的长度
  float hu = 142.5f;
  float hl = 162.f;
  float kp = 0.06f;
  float kw = 0.0f;

  // 求虚拟力的参数
  float ForceKp = 0.00001;
  float ForceKd = 0.00015;

  float pi = 3.1415926f;

private:
  // 根据类的类型求出不同的轨迹
  void LegSupport(float s, int side);
  void LegSwing(float s, int side);

  // 根据腿的类型来 求不同的逆解
  void AppliedInverseSolutionLeft(float x, float z);
  void AppliedInverseSolutionRight(float x, float z);

public:
  // 计算正解
  void AppliedPostiveSolution();
  // 计算虚拟力
  void AppliedVirtualForce(float dt);
};

#endif // QCDOGGGLEG_H
