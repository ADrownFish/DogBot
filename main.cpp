#include <QCoreApplication>

#include "qcdogggbot.h"

/*
 * project : Doggg Bot
 * author : ADrownFish
 * date : 2022-10-12
 * email : a.drownfish@qq.com
 * describe : Four-legged robot  ..
 */

#include <stdlib.h>

/**
* 操作方法：
* 1.将狗腿摆到指定姿势（因为A1电机不能返回外圈绝对位置，只能通过特定的姿态来读取位置，并设定零偏），发送指令站立-行走
* 2. 建立tcp socket ，发送指令（查询r或者设定w）如: r dog.actuator.leg0.MotorMechanicalAngleA 注意结尾需要追加\n符

* 状态机-纯位控-足端轨迹规划-逆解
* A1电机支持混合控：Pos Vel Kp Kd ForwardTau
* 更换电机注意：KP KD不能通用
*/

int main(int argc, char *argv[]) {
  QCoreApplication a(argc, argv);

  // Create robot control instance
  QCDogggBot db;

  /*
  int ms = 0;
  int res = 0;

  while (ms <= 1000)
  {
      res = (      4.02529176036953e-09    ) * pow(ms,4) +
              (  -1.02267416878680e-05    ) * pow(ms,3) +
              (   0.00743558748214280     ) * pow(ms,2) +
              (  -0.245630720143646       ) * pow(ms,1) +
              (   4.53950909717144        );

      printf("x: %d\t\ty: %d\n",ms,res);
      QThread::msleep(20);
      ms++;
  }
  */

  // 进入消息循环
  return a.exec();
}
