#ifndef QCDOGGGBRAINSCHEDULER_H
#define QCDOGGGBRAINSCHEDULER_H

#include <QDateTime>
#include <QMutex>
#include <QObject>
#include <QThread>
#include <QWaitCondition>

#include <QEventLoop>

#include "math.h"

#include "qcdogggbotconfiguration.h"
#include "qcdogggleg.h"
#include "qcdogggnetwork.h"

#include "makeconfig.h"

class QCDogggBrainScheduler : public QThread {
  Q_OBJECT

  enum BodyState {
    DogNull,     // 狗子没有初始化
    DogHalt,     // 狗子立定
    DogCritical, // 临界动作
    DogSquart,   // 蹲下
    DogStand,    // 站立
    DogWalk,     // 行走
    DogRotation, // 旋转
    DogWalkTriangularGait,
    DogWalkStairs,
    DogDrop,
  };

public:
  explicit QCDogggBrainScheduler(QObject *parent = nullptr);
  ~QCDogggBrainScheduler();

  // 获取当前系统运行频率
  int getSystemFrequency();

  void setInit(QCDogggBotConfiguration *, QCDogggNetwork *, QCDogggLeg **);

private:
  // 腿部计算: 返回值：是否 到达了半个周期
  bool LegsMotionControl(int pd);
  // 电机指令发送
  void LegsSendMotorCommands();

  // 绝对延时
  void AbsoluteTimeDelay(int ms);

  // 是否在误差范围内:角度
  bool InsideTheErrorAngle(float value1, float value2);

  // 是否在误差范围内:高度
  bool InsideTheErrorHeight(float value1, float value2);

  // 读取并处理初始化数据
  void readAndProcessInitData();
  // 执行开机 位置 初始化，将位置回到蹲下行动
  void InitDogLegState();

  // 执行 下蹲、站立 行动
  void ExecuteTheAction(BodyState);

  // 发送指令
  void MotorSendRecvData(int device, MOTOR_send *send, MOTOR_recv *recv);

  // 动作事件循环
  void ActionEventLoop();

public:
  // 外部信号：更改动作
  void ProcessingControlCommand();

  // 任务暂停和继续
  void pause();
  void resume();

  // 线程锁
  QMutex mutex;
  QWaitCondition waitCondition;

  // 暂停标志位
  std::atomic_bool pauseFlag;

  // 临界标志位：这个是在执行动作的时候防止中断当前的动作，必须当前动作完成之后才能进行下一个动作
  std::atomic_bool CriticalFlag;

  QEventLoop *FreezeLoop;

private:
  QCDogggBotConfiguration *Configuration;
  QCDogggNetwork *Network;
  QCDogggLeg *DogLeg[4];

protected:
  void run() override;

private:
  int systemSchedulingInterval;

  BodyState currentBodyState;
  BodyState nextBodyState;
  BodyState targetBodyState;

  // 当前机器狗步态的 两个相位 值
  int currentProgressA = 0;
  int currentProgressB = 0;

  bool isUpdates;

  // 步频系数
  int StepFrequencyCoefficient = 10;

  // 系统运行频率
  int SystemFerq;

signals:
  void uploadLogs(QString, int);
  void uploadLegPos();
};

#endif // QCDOGGGBRAINSCHEDULER_H
