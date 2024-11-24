#ifndef QCDOGGGBOTCONFIGURATION_H
#define QCDOGGGBOTCONFIGURATION_H

#include <QObject>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <QList>
#include <QString>
#include <QStringList>
#include <QVariant>

#include "include/motor_ctrl.h" //声明发送数据、接收数据的结构体，以及函数声明

class QCDogggBotConfiguration : public QObject {
  Q_OBJECT

public:
  // 电机配置结构体
  typedef struct {
    MOTOR_send MotorSend[8];
    MOTOR_recv MotorRecv[8];
  } DogggMotor_t;

  struct HostInfo {
    bool lock;    // 锁定当前动作
    bool release; // 释放电机
    int CPU;      // CPU利用率
    int ram;      // RAM利用率
  };

  struct Controller {
    int stepRate;      // 运动速率
    int stepHigh;      // 步高
    int stepDistance;  // 步距
    int stepStandTall; // 站高

    int stepDistanceDifferential; // 速度差分，负值：往左边走，正值：往右边走

    float stiffness; // 刚度
    int FuselageGesture; // 机身姿态： 0 无定义    1 蹲姿    2 站立    3 行走

    bool UploadFootPos; // 是否上传足端坐标
    bool InitDog;       // 是否可开始初始化机体

    bool FreezeDog;     // 冻结机器人，可恢复
    bool EmergencyStop; // 紧急停止，切断输出，不可恢复
  };

  struct Pos {
    int x; // 坐标系x
    int z; // 坐标系z
  };

  struct Leg {
    float calfLength;         // 小腿长
    float thighLength;        // 大腿长
    float trajectoryProgress; // 当前轨迹进度

    Pos pos; // 足端坐标

    float MotorMechanicalAngleA; // 电机机械角度，大腿
    float MotorMechanicalAngleB; // 电机机械角度，小腿
  };

  struct Actuator {
    Leg leg0; // 执行器，四个腿
    Leg leg1;
    Leg leg2;
    Leg leg3;
  };

  struct Dog {
    HostInfo host; //
    Controller controller;
    Actuator actuator;
  };

public:
  explicit QCDogggBotConfiguration(QObject *p);
  QString parsingControlCommands(QString cmd); // 命令解析
  bool GetUpdates();
  void setExpired();

  // 电机配置
  DogggMotor_t DogggMotor;
  Dog dog;

private:
  bool m_GetUpdates = false;

public:
signals:
  void UnFreezeDog();
};

#endif // QCDOGGGBOTCONFIGURATION_H
