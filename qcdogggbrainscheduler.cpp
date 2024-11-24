#include "qcdogggbrainscheduler.h"

QCDogggBrainScheduler::QCDogggBrainScheduler(QObject *parent)
    : QThread(parent), pauseFlag(false), CriticalFlag(false) {
  systemSchedulingInterval = 0;

  // 初始化动作：无动作
  currentBodyState = DogNull;

  // 默认没有新的更新
  isUpdates = false;
}

QCDogggBrainScheduler::~QCDogggBrainScheduler() {}

// 获取系统频率
int QCDogggBrainScheduler::getSystemFrequency() {
  int res = 1000 / systemSchedulingInterval;
  systemSchedulingInterval = 0;
  return res;
}
// 初始化类
void QCDogggBrainScheduler::setInit(QCDogggBotConfiguration *p1,
                                    QCDogggNetwork *p2, QCDogggLeg **p3) {
  Configuration = p1;
  Network = p2;
  DogLeg[0] = p3[0];
  DogLeg[1] = p3[1];
  DogLeg[2] = p3[2];
  DogLeg[3] = p3[3];
}

void QCDogggBrainScheduler::run() {
  emit uploadLogs(QStringLiteral("线程已启动.\n"), 0);

  Configuration->dog.controller.InitDog = true;
  FreezeLoop = new QEventLoop(this);
  connect(Configuration, &QCDogggBotConfiguration::UnFreezeDog, FreezeLoop,
          &QEventLoop::quit);

  SystemFerq = 4;           // 系统运行频率 1000/SystemFerq = 333 hz
  readAndProcessInitData(); // 读取并处理初始化数据
  InitDogLegState();        // 将机体状态初始化

  currentBodyState = DogSquart; // 蹲下状态

  // 进入事件循环
  while (1) {
    // 挂起 继续 线程
    if (pauseFlag) {
      emit uploadLogs(QStringLiteral("运动控制器线程 已暂停\n"), 0);
      mutex.lock();
      waitCondition.wait(&mutex);
      mutex.unlock();
      emit uploadLogs(QStringLiteral("运动控制器线程 已恢复\n"), 0);
    }

    ActionEventLoop();               // 运动事件循环器
    ExecuteTheAction(nextBodyState); // 执行目标动作
    AbsoluteTimeDelay(4);            // 绝对延时 4ms

    // 如果当前状态是目标状态了，就休眠
    if (currentBodyState == targetBodyState) {
      if (currentBodyState == DogWalk) {

      } else if (currentBodyState == DogWalkTriangularGait) {

      } else if (currentBodyState == DogRotation) {

      } else if (currentBodyState == DogWalkStairs) {

      } else {
        pause();
      }
    }

    // 紧急切断输出
    if (Configuration->dog.controller.EmergencyStop)
      break;

    // emit uploadLogs(QStringLiteral("循环完成一次\n"),0);
  }

  emit uploadLogs(QStringLiteral("警告！子调度线程 已结束，请重新启动.\n"), 0);
  FreezeLoop->deleteLater();
}

void QCDogggBrainScheduler::ActionEventLoop() {
  // 事件循环
  // emit
  // uploadLogs(QStringLiteral("执行器调整姿态为：%1\n").arg(targetBodyState),0);
  switch (targetBodyState) {
  case DogSquart:
    if (currentBodyState == DogSquart)
      nextBodyState = DogSquart;
    else if (currentBodyState == DogStand)
      nextBodyState = DogSquart;
    else if (currentBodyState == DogWalk)
      nextBodyState = DogHalt;
    else if (currentBodyState == DogRotation)
      nextBodyState = DogHalt;
    else if (currentBodyState == DogWalkTriangularGait)
      nextBodyState = DogSquart;
    else if (currentBodyState == DogWalkStairs)
      nextBodyState = DogStand;
    else if (currentBodyState == DogDrop)
      nextBodyState = DogStand;

    break;

  case DogStand:
    if (currentBodyState == DogSquart)
      nextBodyState = DogStand;
    else if (currentBodyState == DogStand)
      nextBodyState = DogStand;
    else if (currentBodyState == DogWalk)
      nextBodyState = DogHalt;
    else if (currentBodyState == DogRotation)
      nextBodyState = DogHalt;
    else if (currentBodyState == DogWalkTriangularGait)
      nextBodyState = DogSquart;
    else if (currentBodyState == DogWalkStairs)
      nextBodyState = DogStand;
    else if (currentBodyState == DogDrop)
      nextBodyState = DogStand;

    break;

  case DogWalk:
    if (currentBodyState == DogSquart)
      nextBodyState = DogStand;
    else if (currentBodyState == DogStand)
      nextBodyState = DogWalk;
    else if (currentBodyState == DogWalk)
      nextBodyState = DogWalk;
    else if (currentBodyState == DogRotation)
      nextBodyState = DogHalt;
    else if (currentBodyState == DogWalkTriangularGait)
      nextBodyState = DogSquart;
    else if (currentBodyState == DogWalkStairs)
      nextBodyState = DogStand;
    else if (currentBodyState == DogDrop)
      nextBodyState = DogStand;

    break;

  case DogRotation:
    if (currentBodyState == DogSquart)
      nextBodyState = DogStand;
    else if (currentBodyState == DogStand)
      nextBodyState = DogRotation;
    else if (currentBodyState == DogWalk)
      nextBodyState = DogHalt;
    else if (currentBodyState == DogRotation)
      nextBodyState = DogRotation;
    else if (currentBodyState == DogWalkTriangularGait)
      nextBodyState = DogSquart;
    else if (currentBodyState == DogWalkStairs)
      nextBodyState = DogStand;
    else if (currentBodyState == DogDrop)
      nextBodyState = DogStand;

    break;

  case DogWalkTriangularGait:
    if (currentBodyState == DogSquart)
      nextBodyState = DogWalkTriangularGait;
    else if (currentBodyState == DogStand)
      nextBodyState = DogSquart;
    else if (currentBodyState == DogWalk)
      nextBodyState = DogHalt;
    else if (currentBodyState == DogRotation)
      nextBodyState = DogHalt;
    else if (currentBodyState == DogWalkTriangularGait)
      nextBodyState = DogWalkTriangularGait;
    else if (currentBodyState == DogWalkStairs)
      nextBodyState = DogStand;
    else if (currentBodyState == DogDrop)
      nextBodyState = DogStand;

    break;

  case DogWalkStairs:
    if (currentBodyState == DogSquart)
      nextBodyState = DogStand;
    else if (currentBodyState == DogStand)
      nextBodyState = DogWalkStairs;
    else if (currentBodyState == DogWalk)
      nextBodyState = DogHalt;
    else if (currentBodyState == DogRotation)
      nextBodyState = DogHalt;
    else if (currentBodyState == DogWalkTriangularGait)
      nextBodyState = DogSquart;
    else if (currentBodyState == DogWalkStairs)
      nextBodyState = DogWalkStairs;
    else if (currentBodyState == DogDrop)
      nextBodyState = DogStand;

    break;

  case DogDrop:
    if (currentBodyState == DogSquart)
      nextBodyState = DogStand;
    else if (currentBodyState == DogStand)
      nextBodyState = DogDrop;
    else if (currentBodyState == DogWalk)
      nextBodyState = DogHalt;
    else if (currentBodyState == DogRotation)
      nextBodyState = DogHalt;
    else if (currentBodyState == DogWalkTriangularGait)
      nextBodyState = DogSquart;
    else if (currentBodyState == DogWalkStairs)
      nextBodyState = DogStand;
    else if (currentBodyState == DogDrop)
      nextBodyState = DogDrop;

    break;

  default:
    pause();
  }
}

bool QCDogggBrainScheduler::InsideTheErrorAngle(float value1, float value2) {
  // 如果角度不在指定误差范围内
  float error = fabsf(value1 - value2);

  if (error > (0.1f / 57.3f))
    return false;
  else
    return true;
}

bool QCDogggBrainScheduler::InsideTheErrorHeight(float value1, float value2) {
  // 如果角度不在指定误差范围内
  float error = fabsf(value1 - value2);

  if (error > 0.5)
    return false;
  else
    return true;
}

bool QCDogggBrainScheduler::LegsMotionControl(int pd) {
  bool isOK = false;

  //============================== 对脚步态 ==============================

  // 处理相位问题
  int DifferentValue = abs(currentProgressA - currentProgressB);
  if (DifferentValue < pd - StepFrequencyCoefficient) {
    currentProgressA += StepFrequencyCoefficient;
  } else if (DifferentValue > pd + StepFrequencyCoefficient) {
    currentProgressB += StepFrequencyCoefficient;
  } else {
    currentProgressA += StepFrequencyCoefficient;
    currentProgressB += StepFrequencyCoefficient;
  }

  // 周期 限幅检查
  if (currentProgressA >= 1000) {
    currentProgressA -= 1000;
    isOK = true;
  }
  if (currentProgressB >= 1000) {
    currentProgressB -= 1000;
    isOK = true;
  }

  /*
      <A>     <B>
       BA === CD
          |||
          |||
          |||
       FE === GH
      <B>     <A>
   */

  // 在步态起始周期应用新的 设定参数 ，如果可用

  DogLeg[0]->setCurrentTimePoint(currentProgressA);
  DogLeg[0]->AppliedInverseSolution();

  DogLeg[1]->setCurrentTimePoint(currentProgressB);
  DogLeg[1]->AppliedInverseSolution();

  DogLeg[2]->setCurrentTimePoint(currentProgressB);
  DogLeg[2]->AppliedInverseSolution();

  DogLeg[3]->setCurrentTimePoint(currentProgressA);
  DogLeg[3]->AppliedInverseSolution();

  //============================== 对脚步态 ==============================

  // 处理一下角度问题，这里必须限幅，要不然要出事，之前翻车两次了
  for (int i = 0; i < 4; i++) {
    if (DogLeg[i]->motorSendA->Pos > DogLeg[i]->AOutPutUpperLimit ||
        DogLeg[i]->motorSendA->Pos < DogLeg[i]->AOutPutLowerLimit) {
      DogLeg[i]->motorSendA->mode = 0;
      printf("  Error: ID %d  电机  A 超界 当前 %.4f 上限 %.4f 下限 %.4f\n", i,
             DogLeg[i]->motorSendA->Pos / 9.1f * 57.3f,
             DogLeg[i]->AOutPutUpperLimit / 9.1f * 57.3f,
             DogLeg[i]->AOutPutLowerLimit / 9.1f * 57.3f);
    }

    if (DogLeg[i]->motorSendB->Pos > DogLeg[i]->BOutPutUpperLimit ||
        DogLeg[i]->motorSendB->Pos < DogLeg[i]->BOutPutLowerLimit) {
      DogLeg[i]->motorSendB->mode = 0;
      printf("  Error: ID %d  电机 B 超界 当前 %.4f 上限 %.4f 下限 %.4f\n", i,
             DogLeg[i]->motorSendA->Pos / 9.1f * 57.3f,
             DogLeg[i]->BOutPutUpperLimit / 9.1f * 57.3f,
             DogLeg[i]->BOutPutLowerLimit / 9.1f * 57.3f);
    }
  }

  return isOK;
}

void QCDogggBrainScheduler::MotorSendRecvData(int device, MOTOR_send *send,
                                              MOTOR_recv *recv) {
  // 紧急切断输出
  if (Configuration->dog.controller.EmergencyStop) {
    send->mode = 0;
  }

  modify_data(send);

  switch (device) {
  case 0:
    send_recv(Network->fd0, Network->epfd0, send, recv);
    break;
  case 1:
    send_recv(Network->fd1, Network->epfd1, send, recv);
    break;
  case 2:
    send_recv(Network->fd2, Network->epfd2, send, recv);
    break;
  }

  extract_data(recv);
}

void QCDogggBrainScheduler::LegsSendMotorCommands() {
  if (Configuration->dog.controller.UploadFootPos) {
    emit uploadLegPos();
  }

  static int currentID = 0;

  switch (currentID) {
  case 0:
    // config->DogggMotor.MotorSend[5].Pos *= 1.1f;
    MotorSendRecvData(0, &Configuration->DogggMotor.MotorSend[0],
                      &Configuration->DogggMotor.MotorRecv[0]);
    MotorSendRecvData(1, &Configuration->DogggMotor.MotorSend[2],
                      &Configuration->DogggMotor.MotorRecv[2]);
    MotorSendRecvData(2, &Configuration->DogggMotor.MotorSend[5],
                      &Configuration->DogggMotor.MotorRecv[5]);

    break;

  case 1:
    // config->DogggMotor.MotorSend[1].Pos *= 1.1f;
    MotorSendRecvData(0, &Configuration->DogggMotor.MotorSend[1],
                      &Configuration->DogggMotor.MotorRecv[1]);
    MotorSendRecvData(1, &Configuration->DogggMotor.MotorSend[3],
                      &Configuration->DogggMotor.MotorRecv[3]);
    MotorSendRecvData(2, &Configuration->DogggMotor.MotorSend[7],
                      &Configuration->DogggMotor.MotorRecv[7]);

    break;

  case 2:

    MotorSendRecvData(0, &Configuration->DogggMotor.MotorSend[4],
                      &Configuration->DogggMotor.MotorRecv[4]);
    MotorSendRecvData(1, &Configuration->DogggMotor.MotorSend[6],
                      &Configuration->DogggMotor.MotorRecv[6]);

    break;
  }

  DogLeg[0]->AppliedPostiveSolution();
  DogLeg[1]->AppliedPostiveSolution();
  DogLeg[2]->AppliedPostiveSolution();
  DogLeg[3]->AppliedPostiveSolution();

  // a 485 bus can mount 3 devices
  if (++currentID == 3)
    currentID = 0;
}

void QCDogggBrainScheduler::readAndProcessInitData() {
  // 每只腿的足端坐标补偿
  DogLeg[0]->DesiredFootEndXError = -0; // +
  DogLeg[0]->DesiredFootEndZError = -0;

  DogLeg[1]->DesiredFootEndXError = 0; // -
  DogLeg[1]->DesiredFootEndZError = 0;

  DogLeg[2]->DesiredFootEndXError = -0; // 25;  // +
  DogLeg[2]->DesiredFootEndZError = -8;

  DogLeg[3]->DesiredFootEndXError = 0; //-25; // -
  DogLeg[3]->DesiredFootEndZError = -10;

  // 禁止电机转动，获取每个电机当前位置 Pos,如果没有完全读取成功则不继续
  emit uploadLogs(QStringLiteral("读取电机初始角度. \n"), 0);

  int ReadFailed = -1;

  do {
    AbsoluteTimeDelay(200);

    if (ReadFailed != -1) {
      emit uploadLogs(
          QStringLiteral("读取电机机械角度失败,Code:%1 .\n").arg(ReadFailed),
          0);
    }

    Configuration->DogggMotor.MotorSend[0].mode = 0;
    MotorSendRecvData(0, &Configuration->DogggMotor.MotorSend[0],
                      &Configuration->DogggMotor.MotorRecv[0]);
    AbsoluteTimeDelay(1);

    Configuration->DogggMotor.MotorSend[1].mode = 0;
    MotorSendRecvData(0, &Configuration->DogggMotor.MotorSend[1],
                      &Configuration->DogggMotor.MotorRecv[1]);
    AbsoluteTimeDelay(1);

    Configuration->DogggMotor.MotorSend[4].mode = 0;
    MotorSendRecvData(0, &Configuration->DogggMotor.MotorSend[4],
                      &Configuration->DogggMotor.MotorRecv[4]);
    AbsoluteTimeDelay(1);

    Configuration->DogggMotor.MotorSend[2].mode = 0;
    MotorSendRecvData(1, &Configuration->DogggMotor.MotorSend[2],
                      &Configuration->DogggMotor.MotorRecv[2]);
    AbsoluteTimeDelay(1);

    Configuration->DogggMotor.MotorSend[3].mode = 0;
    MotorSendRecvData(1, &Configuration->DogggMotor.MotorSend[3],
                      &Configuration->DogggMotor.MotorRecv[3]);
    AbsoluteTimeDelay(1);

    Configuration->DogggMotor.MotorSend[6].mode = 0;
    MotorSendRecvData(1, &Configuration->DogggMotor.MotorSend[6],
                      &Configuration->DogggMotor.MotorRecv[6]);
    AbsoluteTimeDelay(1);

    Configuration->DogggMotor.MotorSend[5].mode = 0;
    MotorSendRecvData(2, &Configuration->DogggMotor.MotorSend[5],
                      &Configuration->DogggMotor.MotorRecv[5]);
    AbsoluteTimeDelay(1);

    Configuration->DogggMotor.MotorSend[7].mode = 0;
    MotorSendRecvData(2, &Configuration->DogggMotor.MotorSend[7],
                      &Configuration->DogggMotor.MotorRecv[7]);
    AbsoluteTimeDelay(1);
  } while ((ReadFailed = 0,
            fabs(Configuration->DogggMotor.MotorRecv[0].Pos) < 1e-6) ||
           (ReadFailed = 1,
            fabs(Configuration->DogggMotor.MotorRecv[1].Pos) < 1e-6) ||
           (ReadFailed = 2,
            fabs(Configuration->DogggMotor.MotorRecv[2].Pos) < 1e-6) ||
           (ReadFailed = 3,
            fabs(Configuration->DogggMotor.MotorRecv[3].Pos) < 1e-6) ||
           (ReadFailed = 4,
            fabs(Configuration->DogggMotor.MotorRecv[4].Pos) < 1e-6) ||
           (ReadFailed = 5,
            fabs(Configuration->DogggMotor.MotorRecv[5].Pos) < 1e-6) ||
           (ReadFailed = 6,
            fabs(Configuration->DogggMotor.MotorRecv[6].Pos) < 1e-6) ||
           (ReadFailed = 7,
            fabs(Configuration->DogggMotor.MotorRecv[7].Pos) < 1e-6));

  // 计算极限位置，防止bug 出事
  DogLeg[0]->AOutPutUpperLimit = DogLeg[0]->motorRecvA->Pos + 50 / 57.3f * 9.1f;
  DogLeg[0]->AOutPutLowerLimit = DogLeg[0]->motorRecvA->Pos - 50 / 57.3f * 9.1f;
  DogLeg[0]->BOutPutUpperLimit =
      DogLeg[0]->motorRecvB->Pos + 150 / 57.3f * 9.1f;
  DogLeg[0]->BOutPutLowerLimit = DogLeg[0]->motorRecvB->Pos - 5 / 57.3f * 9.1f;

  DogLeg[1]->AOutPutUpperLimit = DogLeg[1]->motorRecvA->Pos + 50 / 57.3f * 9.1f;
  DogLeg[1]->AOutPutLowerLimit = DogLeg[1]->motorRecvA->Pos - 50 / 57.3f * 9.1f;
  DogLeg[1]->BOutPutUpperLimit = DogLeg[1]->motorRecvB->Pos + 5 / 57.3f * 9.1f;
  DogLeg[1]->BOutPutLowerLimit =
      DogLeg[1]->motorRecvB->Pos - 150 / 57.3f * 9.1f;

  DogLeg[2]->AOutPutUpperLimit = DogLeg[2]->motorRecvA->Pos + 50 / 57.3f * 9.1f;
  DogLeg[2]->AOutPutLowerLimit = DogLeg[2]->motorRecvA->Pos - 50 / 57.3f * 9.1f;
  DogLeg[2]->BOutPutUpperLimit =
      DogLeg[2]->motorRecvB->Pos + 150 / 57.3f * 9.1f;
  DogLeg[2]->BOutPutLowerLimit = DogLeg[2]->motorRecvB->Pos - 5 / 57.3f * 9.1f;

  DogLeg[3]->AOutPutUpperLimit = DogLeg[3]->motorRecvA->Pos + 50 / 57.3f * 9.1f;
  DogLeg[3]->AOutPutLowerLimit = DogLeg[3]->motorRecvA->Pos - 50 / 57.3f * 9.1f;
  DogLeg[3]->BOutPutUpperLimit = DogLeg[3]->motorRecvB->Pos + 5 / 57.3f * 9.1f;
  DogLeg[3]->BOutPutLowerLimit =
      DogLeg[3]->motorRecvB->Pos - 150 / 57.3f * 9.1f;

  // 输出读取到的电机位置
  emit uploadLogs(QStringLiteral("已读取到 腿1 机械角度A: %1 机械角度B:%2.\n")
                      .arg(DogLeg[0]->motorRecvA->Pos * 57.3f / 9.1f)
                      .arg(DogLeg[0]->motorRecvB->Pos * 57.3f / 9.1f),
                  0);
  emit uploadLogs(QStringLiteral("已读取到 腿2 机械角度A: %1 机械角度B:%2.\n")
                      .arg(DogLeg[1]->motorRecvA->Pos * 57.3f / 9.1f)
                      .arg(DogLeg[1]->motorRecvB->Pos * 57.3f / 9.1f),
                  0);
  emit uploadLogs(QStringLiteral("已读取到 腿3 机械角度A: %1 机械角度B:%2.\n")
                      .arg(DogLeg[2]->motorRecvA->Pos * 57.3f / 9.1f)
                      .arg(DogLeg[2]->motorRecvB->Pos * 57.3f / 9.1f),
                  0);
  emit uploadLogs(QStringLiteral("已读取到 腿4 机械角度A: %1 机械角度B:%2.\n")
                      .arg(DogLeg[3]->motorRecvA->Pos * 57.3f / 9.1f)
                      .arg(DogLeg[3]->motorRecvB->Pos * 57.3f / 9.1f),
                  0);

  emit uploadLogs(QStringLiteral("计算角度误差.\n"), 0);

  for (int i = 0; i < 4; i++) {
    // 将接收到的角度 设为 初始化绝对角度
    DogLeg[i]->InitDesiredPosA = DogLeg[i]->motorRecvA->Pos / 9.1f;
    DogLeg[i]->InitDesiredPosB = DogLeg[i]->motorRecvB->Pos / 9.1f;

    // 计算角度误差
    DogLeg[i]->ErrorPosA =
        -DogLeg[i]->InitDesiredPosA - pow(-1, i) * (30.5f / 57.3f);
    DogLeg[i]->ErrorPosB =
        -DogLeg[i]->InitDesiredPosB + pow(-1, i) * (18.f / 57.3);
    // 算出站立姿态是多少角度，
    DogLeg[i]->setCurrentTimePoint(0);
    DogLeg[i]->AppliedInverseSolution();

#if (SimulationMode == 1)
    {
      DogLeg[i]->motorSendA->mode = 0;
      DogLeg[i]->motorSendB->mode = 0;
    }
#else
    {
      DogLeg[i]->motorSendA->mode = 10;
      DogLeg[i]->motorSendB->mode = 10;
    }
#endif
  }
}

void QCDogggBrainScheduler::InitDogLegState() {
  emit uploadLogs(QStringLiteral("初始化电机角度.\n"), 0);

  float rate = 0.05f;

  // 这里打算使用sigmiod函数
  while (1) {
    // 慢慢靠近期望角度

    // 判断是否已经在误差范围内,是则跳出
    if (InsideTheErrorAngle(DogLeg[0]->DesiredPosA,
                            DogLeg[0]->InitDesiredPosA) &&
        InsideTheErrorAngle(DogLeg[0]->DesiredPosB,
                            DogLeg[0]->InitDesiredPosB) // 第一跟儿 腿
        && InsideTheErrorAngle(DogLeg[1]->DesiredPosA,
                               DogLeg[1]->InitDesiredPosA) &&
        InsideTheErrorAngle(DogLeg[1]->DesiredPosB,
                            DogLeg[1]->InitDesiredPosB) // 第二跟儿 腿
        && InsideTheErrorAngle(DogLeg[2]->DesiredPosA,
                               DogLeg[2]->InitDesiredPosA) &&
        InsideTheErrorAngle(DogLeg[2]->DesiredPosB,
                            DogLeg[2]->InitDesiredPosB) // 第三跟儿 腿
        && InsideTheErrorAngle(DogLeg[3]->DesiredPosA,
                               DogLeg[3]->InitDesiredPosA) &&
        InsideTheErrorAngle(DogLeg[3]->DesiredPosB,
                            DogLeg[3]->InitDesiredPosB) // 第四跟儿 腿

    ) {
      break;
    }

    // 0.01度/次 的速度接近

    for (int i = 0; i < 4; i++) {
      if (DogLeg[i]->InitDesiredPosA > DogLeg[i]->DesiredPosA)
        DogLeg[i]->InitDesiredPosA =
            DogLeg[i]->InitDesiredPosA - (rate / 57.3f);
      else
        DogLeg[i]->InitDesiredPosA =
            DogLeg[i]->InitDesiredPosA + (rate / 57.3f);

      if (DogLeg[i]->InitDesiredPosB > DogLeg[i]->DesiredPosB)
        DogLeg[i]->InitDesiredPosB =
            DogLeg[i]->InitDesiredPosB - (rate / 57.3f);
      else
        DogLeg[i]->InitDesiredPosB =
            DogLeg[i]->InitDesiredPosB + (rate / 57.3f);

      DogLeg[i]->motorSendA->Pos = DogLeg[i]->InitDesiredPosA * 9.1f;
      DogLeg[i]->motorSendB->Pos = DogLeg[i]->InitDesiredPosB * 9.1f;
    }

    // 将数据发送到电机
    LegsSendMotorCommands();
    // delay
    AbsoluteTimeDelay(SystemFerq);
  }

  // 初始化完成
  currentBodyState = DogSquart;
  nextBodyState = currentBodyState;
  targetBodyState = currentBodyState;

  emit uploadLogs(QStringLiteral("腿的机械位置 初始化完成.\n"), 0);
}

void QCDogggBrainScheduler::ExecuteTheAction(BodyState state) {
  static float DHeight;
  static float rate = 0.25f;

  const float zhanligaodu = 190;
  const float dunxiagaodu = 140;

  float cardiacOffsetX = 0;
  float cardiacOffsetZ = 0;
  float currentCardiacOffsetX = 0;
  float currentCardiacOffsetZ = 0;
  int setTimePoint = 0;

  float TriangularGaitRateX = 0;
  float TriangularGaitRateZ = 0;

  // emit uploadLogs(QStringLiteral("ExecuteTheAction.\n"),0);

  printf("即将设定状态: %d\n", state);

  currentBodyState = DogCritical;

  switch (state) {
  case DogNull:
    currentBodyState = state;
    break;

  case DogWalkStairs:
    // 先要重心往后

    TriangularGaitRateX = 0.3f;
    // 先往后偏移
    cardiacOffsetX = 40;

    DogLeg[0]->StepDistance = Configuration->dog.controller.stepDistance * 4;
    DogLeg[1]->StepDistance = Configuration->dog.controller.stepDistance * 4;
    DogLeg[2]->StepDistance = Configuration->dog.controller.stepDistance * 4;
    DogLeg[3]->StepDistance = Configuration->dog.controller.stepDistance * 4;

    DogLeg[0]->StepHeight = Configuration->dog.controller.stepHigh * 2.5;
    DogLeg[1]->StepHeight = Configuration->dog.controller.stepHigh * 2.5;
    DogLeg[2]->StepHeight = Configuration->dog.controller.stepHigh * 2.5;
    DogLeg[3]->StepHeight = Configuration->dog.controller.stepHigh * 2.5;

    // 偏移1 重心后移
    currentCardiacOffsetX = 0;
    while (1) {
      // 0.1的偏移
      currentCardiacOffsetX = currentCardiacOffsetX + TriangularGaitRateX;
      if (currentCardiacOffsetX > cardiacOffsetX) {
        currentCardiacOffsetX = cardiacOffsetX;
        break;
      }
      DogLeg[0]->OriginalOffsetX = -currentCardiacOffsetX;
      DogLeg[1]->OriginalOffsetX = currentCardiacOffsetX;
      DogLeg[2]->OriginalOffsetX = -currentCardiacOffsetX;
      DogLeg[3]->OriginalOffsetX = currentCardiacOffsetX;

      DogLeg[0]->setCurrentTimePoint(0);
      DogLeg[0]->AppliedInverseSolution();

      DogLeg[1]->setCurrentTimePoint(0);
      DogLeg[1]->AppliedInverseSolution();

      DogLeg[2]->setCurrentTimePoint(0);
      DogLeg[2]->AppliedInverseSolution();

      DogLeg[3]->setCurrentTimePoint(0);
      DogLeg[3]->AppliedInverseSolution();

      LegsSendMotorCommands();

      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    // 伸腿 1 - 2腿 伸前退
    setTimePoint = 0;
    for (int i = 0; i < 2; i++) {
      setTimePoint = 0;
      while (setTimePoint++, setTimePoint++, setTimePoint <= 500) {
        DogLeg[i]->setCurrentTimePoint(setTimePoint);
        DogLeg[i]->AppliedInverseSolution();
        LegsSendMotorCommands();

        // delay
        AbsoluteTimeDelay(SystemFerq);
      }
    }

    // 偏移2  重心前移
    while (1) {
      // 0.001的偏移
      currentCardiacOffsetX = currentCardiacOffsetX - TriangularGaitRateX;
      if (currentCardiacOffsetX < (-cardiacOffsetX * 1.85)) {
        currentCardiacOffsetX = (-cardiacOffsetX * 1.85);
        break;
      }

      DogLeg[0]->OriginalOffsetX = -currentCardiacOffsetX;
      DogLeg[1]->OriginalOffsetX = currentCardiacOffsetX;
      DogLeg[2]->OriginalOffsetX = -currentCardiacOffsetX;
      DogLeg[3]->OriginalOffsetX = currentCardiacOffsetX;

      DogLeg[0]->setCurrentTimePoint(500);
      DogLeg[0]->AppliedInverseSolution();

      DogLeg[1]->setCurrentTimePoint(500);
      DogLeg[1]->AppliedInverseSolution();

      DogLeg[2]->setCurrentTimePoint(0);
      DogLeg[2]->AppliedInverseSolution();

      DogLeg[3]->setCurrentTimePoint(0);
      DogLeg[3]->AppliedInverseSolution();

      LegsSendMotorCommands();

      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    currentCardiacOffsetZ = 0;
    TriangularGaitRateZ = 0.08;
    cardiacOffsetZ = -40;
    // 重心前倾
    while (1) {
      // 0.001的偏移
      currentCardiacOffsetZ = currentCardiacOffsetZ - TriangularGaitRateZ;
      if (currentCardiacOffsetZ < (cardiacOffsetZ)) {
        currentCardiacOffsetZ = (cardiacOffsetZ);
        break;
      }

      DogLeg[0]->OriginalOffsetZ = currentCardiacOffsetZ;
      DogLeg[1]->OriginalOffsetZ = currentCardiacOffsetZ;

      DogLeg[2]->OriginalOffsetZ = -currentCardiacOffsetZ;
      DogLeg[3]->OriginalOffsetZ = -currentCardiacOffsetZ;

      DogLeg[0]->setCurrentTimePoint(500);
      DogLeg[0]->AppliedInverseSolution();

      DogLeg[1]->setCurrentTimePoint(500);
      DogLeg[1]->AppliedInverseSolution();

      DogLeg[2]->setCurrentTimePoint(0);
      DogLeg[2]->AppliedInverseSolution();

      DogLeg[3]->setCurrentTimePoint(0);
      DogLeg[3]->AppliedInverseSolution();

      LegsSendMotorCommands();

      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    // 伸腿 3 - 4腿
    setTimePoint = 0;
    for (int i = 2; i < 4; i++) {
      setTimePoint = 0;
      while (setTimePoint++, setTimePoint++, setTimePoint <= 500) {
        DogLeg[i]->setCurrentTimePoint(setTimePoint);
        DogLeg[i]->AppliedInverseSolution();
        LegsSendMotorCommands();

        // delay
        AbsoluteTimeDelay(SystemFerq);
      }
    }

    // 重心恢复
    while (1) {
      // 0.001的偏移
      currentCardiacOffsetZ = currentCardiacOffsetZ + TriangularGaitRateZ;
      if (currentCardiacOffsetZ > (0)) {
        currentCardiacOffsetZ = (0);
        break;
      }

      DogLeg[0]->OriginalOffsetZ = currentCardiacOffsetZ;
      DogLeg[1]->OriginalOffsetZ = currentCardiacOffsetZ;

      DogLeg[2]->OriginalOffsetZ = -currentCardiacOffsetZ;
      DogLeg[3]->OriginalOffsetZ = -currentCardiacOffsetZ;

      DogLeg[0]->setCurrentTimePoint(500);
      DogLeg[0]->AppliedInverseSolution();

      DogLeg[1]->setCurrentTimePoint(500);
      DogLeg[1]->AppliedInverseSolution();

      DogLeg[2]->setCurrentTimePoint(500);
      DogLeg[2]->AppliedInverseSolution();

      DogLeg[3]->setCurrentTimePoint(500);
      DogLeg[3]->AppliedInverseSolution();

      LegsSendMotorCommands();

      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    setTimePoint = 500;
    while (setTimePoint++, setTimePoint++, setTimePoint <= 1000) {

      DogLeg[0]->setCurrentTimePoint(setTimePoint);
      DogLeg[0]->AppliedInverseSolution();
      DogLeg[1]->setCurrentTimePoint(setTimePoint);
      DogLeg[1]->AppliedInverseSolution();
      DogLeg[2]->setCurrentTimePoint(setTimePoint);
      DogLeg[2]->AppliedInverseSolution();
      DogLeg[3]->setCurrentTimePoint(setTimePoint);
      DogLeg[3]->AppliedInverseSolution();

      LegsSendMotorCommands();
      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    // 偏移3
    while (1) {
      // 0.001的偏移
      currentCardiacOffsetX = currentCardiacOffsetX + TriangularGaitRateX;
      if (currentCardiacOffsetX > 0) {
        currentCardiacOffsetX = 0.01;
        break;
      }

      DogLeg[0]->OriginalOffsetX = -currentCardiacOffsetX;
      DogLeg[1]->OriginalOffsetX = currentCardiacOffsetX;
      DogLeg[2]->OriginalOffsetX = -currentCardiacOffsetX;
      DogLeg[3]->OriginalOffsetX = currentCardiacOffsetX;

      DogLeg[0]->setCurrentTimePoint(0);
      DogLeg[0]->AppliedInverseSolution();

      DogLeg[1]->setCurrentTimePoint(0);
      DogLeg[1]->AppliedInverseSolution();

      DogLeg[2]->setCurrentTimePoint(0);
      DogLeg[2]->AppliedInverseSolution();

      DogLeg[3]->setCurrentTimePoint(0);
      DogLeg[3]->AppliedInverseSolution();

      LegsSendMotorCommands();

      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    DogLeg[0]->StepDistance = Configuration->dog.controller.stepDistance * 1;
    DogLeg[1]->StepDistance = Configuration->dog.controller.stepDistance * 1;
    DogLeg[2]->StepDistance = Configuration->dog.controller.stepDistance * 1;
    DogLeg[3]->StepDistance = Configuration->dog.controller.stepDistance * 1;

    DogLeg[0]->StepHeight = Configuration->dog.controller.stepHigh * 1;
    DogLeg[1]->StepHeight = Configuration->dog.controller.stepHigh * 1;
    DogLeg[2]->StepHeight = Configuration->dog.controller.stepHigh * 1;
    DogLeg[3]->StepHeight = Configuration->dog.controller.stepHigh * 1;

    currentBodyState = state;

    AbsoluteTimeDelay(500);
    break;

  case DogDrop:
    for (int i = 0; i < 1; i++) {
      float dangqiandeHeight = 0;

      dangqiandeHeight = zhanligaodu;
      while (1) {
        dangqiandeHeight = dangqiandeHeight - 1;
        if (dangqiandeHeight < (dunxiagaodu - 10)) {
          dangqiandeHeight = (dunxiagaodu - 10);
          break;
        }

        DogLeg[1]->StepStandingHeight = dangqiandeHeight;
        DogLeg[3]->StepStandingHeight = dangqiandeHeight;

        DogLeg[1]->setCurrentTimePoint(0);
        DogLeg[1]->AppliedInverseSolution();

        DogLeg[3]->setCurrentTimePoint(0);
        DogLeg[3]->AppliedInverseSolution();

        LegsSendMotorCommands();

        AbsoluteTimeDelay(SystemFerq);
      }

      // 这里写舵机操作代码

      printf("执行舵机代码\n");

      {
#include <stdlib.h>
        system("python3 /home/xiaoqing/Desktop/GPIO1.py");
      }

      AbsoluteTimeDelay(2000);

      // 这里写舵机操作代码

      while (1) {
        dangqiandeHeight = dangqiandeHeight + 0.5;
        if (dangqiandeHeight > zhanligaodu) {
          dangqiandeHeight = zhanligaodu;
          break;
        }

        DogLeg[1]->StepStandingHeight = dangqiandeHeight;
        DogLeg[3]->StepStandingHeight = dangqiandeHeight;

        DogLeg[1]->setCurrentTimePoint(0);
        DogLeg[1]->AppliedInverseSolution();

        DogLeg[3]->setCurrentTimePoint(0);
        DogLeg[3]->AppliedInverseSolution();

        LegsSendMotorCommands();

        AbsoluteTimeDelay(SystemFerq);
      }
    }

    currentBodyState = state;
    AbsoluteTimeDelay(500);
    break;

  case DogHalt:
    // AbsoluteTimeDelay(1000);

    if (currentProgressA <= StepFrequencyCoefficient) {
      DogLeg[0]->StepDistance = 1;
      DogLeg[3]->StepDistance = 1;

      DogLeg[0]->StepDesiredDistance = 1;
      DogLeg[3]->StepDesiredDistance = 1;

      while (1) {
        if (LegsMotionControl(500)) {
          break;
        }
        // 通信
        LegsSendMotorCommands();
        // delay
        AbsoluteTimeDelay(SystemFerq);
      }

      while (1) {
        currentProgressA += StepFrequencyCoefficient;

        if (currentProgressA > 1000) {
          currentProgressA -= 1000;

          DogLeg[0]->setCurrentTimePoint(currentProgressA);
          DogLeg[0]->AppliedInverseSolution();
          DogLeg[3]->setCurrentTimePoint(currentProgressA);
          DogLeg[3]->AppliedInverseSolution();

          // 通信
          LegsSendMotorCommands();
          // delay
          AbsoluteTimeDelay(SystemFerq);

          break;
        }

        DogLeg[0]->setCurrentTimePoint(currentProgressA);
        DogLeg[0]->AppliedInverseSolution();
        DogLeg[3]->setCurrentTimePoint(currentProgressA);
        DogLeg[3]->AppliedInverseSolution();

        // 通信
        LegsSendMotorCommands();
        // delay
        AbsoluteTimeDelay(SystemFerq);
      }

      DogLeg[0]->StepDistance = DogLeg[1]->StepDistance;
      DogLeg[3]->StepDistance = DogLeg[2]->StepDistance;

      DogLeg[0]->StepDesiredDistance = DogLeg[1]->StepDesiredDistance;
      DogLeg[3]->StepDesiredDistance = DogLeg[2]->StepDesiredDistance;
    } else if (currentProgressB <= StepFrequencyCoefficient) {
      DogLeg[1]->StepDistance = 1;
      DogLeg[2]->StepDistance = 1;

      DogLeg[1]->StepDesiredDistance = 1;
      DogLeg[2]->StepDesiredDistance = 1;

      while (1) {
        if (LegsMotionControl(500)) {
          break;
        }
        // 通信
        LegsSendMotorCommands();
        // delay
        AbsoluteTimeDelay(SystemFerq);
      }

      while (1) {
        currentProgressB += StepFrequencyCoefficient;

        if (currentProgressB > 1000) {
          currentProgressB -= 1000;

          DogLeg[1]->setCurrentTimePoint(currentProgressB);
          DogLeg[1]->AppliedInverseSolution();
          DogLeg[2]->setCurrentTimePoint(currentProgressB);
          DogLeg[2]->AppliedInverseSolution();

          // 通信
          LegsSendMotorCommands();
          // delay
          AbsoluteTimeDelay(SystemFerq);

          break;
        }

        DogLeg[1]->setCurrentTimePoint(currentProgressB);
        DogLeg[1]->AppliedInverseSolution();
        DogLeg[2]->setCurrentTimePoint(currentProgressB);
        DogLeg[2]->AppliedInverseSolution();

        // 通信
        LegsSendMotorCommands();
        // delay
        AbsoluteTimeDelay(SystemFerq);
      }

      DogLeg[1]->StepDistance = DogLeg[0]->StepDistance;
      DogLeg[2]->StepDistance = DogLeg[3]->StepDistance;

      DogLeg[1]->StepDesiredDistance = DogLeg[0]->StepDesiredDistance;
      DogLeg[2]->StepDesiredDistance = DogLeg[3]->StepDesiredDistance;
    }

    currentBodyState = DogStand;

    break;

  case DogWalk:
    bool ok;
    do {

      if (isUpdates) {
        if (currentProgressA >= 0 &&
            currentProgressA <= StepFrequencyCoefficient) {
          DogLeg[0]->StepDistance = Configuration->dog.controller.stepDistance;
          DogLeg[0]->StepHeight = Configuration->dog.controller.stepHigh;

          DogLeg[3]->StepDistance = Configuration->dog.controller.stepDistance;
          DogLeg[3]->StepHeight = Configuration->dog.controller.stepHigh;
        }
        if (currentProgressB >= 0 &&
            currentProgressB <= StepFrequencyCoefficient) {
          DogLeg[1]->StepDistance = Configuration->dog.controller.stepDistance;
          DogLeg[1]->StepHeight = Configuration->dog.controller.stepHigh;

          DogLeg[2]->StepDistance = Configuration->dog.controller.stepDistance;
          DogLeg[2]->StepHeight = Configuration->dog.controller.stepHigh;
        }
      }

      ok = LegsMotionControl(500);
      // DogLeg[0]->AppliedPostiveSolution();
      // DogLeg[0]->AppliedVirtualForce(SystemFerq*0.001);

      // 通信
      LegsSendMotorCommands();

      // printf("虚拟力距 A: %.4f   B:
      // %.4f\n",DogLeg[0]->VirtualTorqueA,DogLeg[0]->VirtualTorqueB);
      // printf("实际力距 A: %.4f   B:
      // %.4f\n\n",DogLeg[0]->motorRecvA->T,DogLeg[0]->motorRecvB->T);
      // printf("腿1: %.4f %.4f
      // \n",DogLeg[0]->DesiredFootEndX,DogLeg[0]->DesiredFootEndZ);

      // delay
      AbsoluteTimeDelay(SystemFerq);
    } while (!ok);

    currentBodyState = state;
    // emit uploadLogs(QStringLiteral("当前姿态 %1\n").arg(currentBodyState),0);
    // emit uploadLogs(QStringLiteral("已切换至 Walk\n"),0);
    break;

  case DogRotation:
    bool ok1;

    printf("rotation rate is: %d \n",
           Configuration->dog.controller.stepDistanceDifferential);

    do {
      if (isUpdates) {
        if (currentProgressA >= 0 &&
            currentProgressA <= StepFrequencyCoefficient) {
          DogLeg[0]->StepDistance =
              Configuration->dog.controller.stepDistanceDifferential / 2 + 5;
          DogLeg[0]->StepHeight = Configuration->dog.controller.stepHigh;

          DogLeg[3]->StepDistance =
              -Configuration->dog.controller.stepDistanceDifferential / 2 + 5;
          DogLeg[3]->StepHeight = Configuration->dog.controller.stepHigh;
        }
        if (currentProgressB >= 0 &&
            currentProgressB <= StepFrequencyCoefficient) {
          DogLeg[1]->StepDistance =
              -Configuration->dog.controller.stepDistanceDifferential / 2 + 5;
          DogLeg[1]->StepHeight = Configuration->dog.controller.stepHigh;

          DogLeg[2]->StepDistance =
              Configuration->dog.controller.stepDistanceDifferential / 2 + 5;
          DogLeg[2]->StepHeight = Configuration->dog.controller.stepHigh;
        }
      }

      ok1 = LegsMotionControl(500);
      DogLeg[0]->AppliedPostiveSolution();
      // DogLeg[0]->AppliedVirtualForce(SystemFerq*0.001);

      // 通信
      LegsSendMotorCommands();

      // delay
      AbsoluteTimeDelay(SystemFerq);
    } while (!ok1);

    currentBodyState = state;

    break;

  case DogWalkTriangularGait:

    // 先要重心往后

    TriangularGaitRateX = 0.3f;
    // 先往后偏移
    cardiacOffsetX = 40;

    DogLeg[0]->StepDistance = Configuration->dog.controller.stepDistance * 3;
    DogLeg[1]->StepDistance = Configuration->dog.controller.stepDistance * 3;
    DogLeg[2]->StepDistance = Configuration->dog.controller.stepDistance * 3;
    DogLeg[3]->StepDistance = Configuration->dog.controller.stepDistance * 3;

    DogLeg[0]->StepHeight = Configuration->dog.controller.stepHigh * 2.5;
    DogLeg[1]->StepHeight = Configuration->dog.controller.stepHigh * 2.5;
    DogLeg[2]->StepHeight = Configuration->dog.controller.stepHigh * 2.5;
    DogLeg[3]->StepHeight = Configuration->dog.controller.stepHigh * 2.5;

    // 偏移1
    currentCardiacOffsetX = 0;
    while (1) {
      // 0.1的偏移
      currentCardiacOffsetX = currentCardiacOffsetX + TriangularGaitRateX;
      if (currentCardiacOffsetX > cardiacOffsetX) {
        currentCardiacOffsetX = cardiacOffsetX;
        break;
      }

      // printf("\t value \t%f\n",currentCardiacOffsetX);

      DogLeg[0]->OriginalOffsetX = -currentCardiacOffsetX;
      DogLeg[1]->OriginalOffsetX = currentCardiacOffsetX;
      DogLeg[2]->OriginalOffsetX = -currentCardiacOffsetX;
      DogLeg[3]->OriginalOffsetX = currentCardiacOffsetX;

      DogLeg[0]->setCurrentTimePoint(0);
      DogLeg[0]->AppliedInverseSolution();

      DogLeg[1]->setCurrentTimePoint(0);
      DogLeg[1]->AppliedInverseSolution();

      DogLeg[2]->setCurrentTimePoint(0);
      DogLeg[2]->AppliedInverseSolution();

      DogLeg[3]->setCurrentTimePoint(0);
      DogLeg[3]->AppliedInverseSolution();

      LegsSendMotorCommands();

      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    // 伸腿 1 - 2腿
    setTimePoint = 0;
    for (int i = 0; i < 2; i++) {
      setTimePoint = 0;
      while (setTimePoint++, setTimePoint <= 500) {
        DogLeg[i]->setCurrentTimePoint(setTimePoint);
        DogLeg[i]->AppliedInverseSolution();
        LegsSendMotorCommands();

        // delay
        AbsoluteTimeDelay(SystemFerq);
      }
    }

    // 偏移2
    while (1) {
      // 0.001的偏移
      currentCardiacOffsetX = currentCardiacOffsetX - TriangularGaitRateX;
      if (currentCardiacOffsetX < (-cardiacOffsetX * 1.8)) {
        currentCardiacOffsetX = (-cardiacOffsetX * 1.8);
        break;
      }

      DogLeg[0]->OriginalOffsetX = -currentCardiacOffsetX;
      DogLeg[1]->OriginalOffsetX = currentCardiacOffsetX;
      DogLeg[2]->OriginalOffsetX = -currentCardiacOffsetX;
      DogLeg[3]->OriginalOffsetX = currentCardiacOffsetX;

      DogLeg[0]->setCurrentTimePoint(500);
      DogLeg[0]->AppliedInverseSolution();

      DogLeg[1]->setCurrentTimePoint(500);
      DogLeg[1]->AppliedInverseSolution();

      DogLeg[2]->setCurrentTimePoint(0);
      DogLeg[2]->AppliedInverseSolution();

      DogLeg[3]->setCurrentTimePoint(0);
      DogLeg[3]->AppliedInverseSolution();

      LegsSendMotorCommands();

      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    currentCardiacOffsetZ = 0;
    TriangularGaitRateZ = 0.1;
    cardiacOffsetZ = -20;
    // jiang di qian mian zhong xin
    while (1) {
      // 0.001的偏移
      currentCardiacOffsetZ = currentCardiacOffsetZ - TriangularGaitRateZ;
      if (currentCardiacOffsetZ < (cardiacOffsetZ)) {
        currentCardiacOffsetZ = (cardiacOffsetZ);
        break;
      }

      DogLeg[0]->OriginalOffsetZ = currentCardiacOffsetZ;
      DogLeg[1]->OriginalOffsetZ = currentCardiacOffsetZ;
      // DogLeg[2]->OriginalOffsetZ = -currentCardiacOffsetZ;
      // DogLeg[3]->OriginalOffsetZ = currentCardiacOffsetZ;

      DogLeg[0]->setCurrentTimePoint(500);
      DogLeg[0]->AppliedInverseSolution();

      DogLeg[1]->setCurrentTimePoint(500);
      DogLeg[1]->AppliedInverseSolution();

      LegsSendMotorCommands();

      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    // 伸腿 3 - 4腿
    setTimePoint = 0;
    for (int i = 2; i < 4; i++) {
      setTimePoint = 0;
      while (setTimePoint++, setTimePoint <= 500) {
        DogLeg[i]->setCurrentTimePoint(setTimePoint);
        DogLeg[i]->AppliedInverseSolution();
        LegsSendMotorCommands();

        // delay
        AbsoluteTimeDelay(SystemFerq);
      }
    }

    while (1) {
      // 0.001的偏移
      currentCardiacOffsetZ = currentCardiacOffsetZ + TriangularGaitRateZ;
      if (currentCardiacOffsetZ > (0)) {
        currentCardiacOffsetZ = (0);
        break;
      }

      DogLeg[0]->OriginalOffsetZ = currentCardiacOffsetZ;
      DogLeg[1]->OriginalOffsetZ = currentCardiacOffsetZ;
      // DogLeg[2]->OriginalOffsetZ = -currentCardiacOffsetZ;
      // DogLeg[3]->OriginalOffsetZ = currentCardiacOffsetZ;

      DogLeg[0]->setCurrentTimePoint(500);
      DogLeg[0]->AppliedInverseSolution();

      DogLeg[1]->setCurrentTimePoint(500);
      DogLeg[1]->AppliedInverseSolution();

      LegsSendMotorCommands();

      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    setTimePoint = 500;
    while (setTimePoint++, setTimePoint <= 1000) {

      DogLeg[0]->setCurrentTimePoint(setTimePoint);
      DogLeg[0]->AppliedInverseSolution();
      DogLeg[1]->setCurrentTimePoint(setTimePoint);
      DogLeg[1]->AppliedInverseSolution();
      DogLeg[2]->setCurrentTimePoint(setTimePoint);
      DogLeg[2]->AppliedInverseSolution();
      DogLeg[3]->setCurrentTimePoint(setTimePoint);
      DogLeg[3]->AppliedInverseSolution();

      LegsSendMotorCommands();
      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    // 偏移3
    while (1) {
      // 0.001的偏移
      currentCardiacOffsetX = currentCardiacOffsetX + TriangularGaitRateX;
      if (currentCardiacOffsetX > 0) {
        currentCardiacOffsetX = 0;
        break;
      }

      DogLeg[0]->OriginalOffsetX = -currentCardiacOffsetX;
      DogLeg[1]->OriginalOffsetX = currentCardiacOffsetX;
      DogLeg[2]->OriginalOffsetX = -currentCardiacOffsetX;
      DogLeg[3]->OriginalOffsetX = currentCardiacOffsetX;

      DogLeg[0]->setCurrentTimePoint(0);
      DogLeg[0]->AppliedInverseSolution();

      DogLeg[1]->setCurrentTimePoint(0);
      DogLeg[1]->AppliedInverseSolution();

      DogLeg[2]->setCurrentTimePoint(0);
      DogLeg[2]->AppliedInverseSolution();

      DogLeg[3]->setCurrentTimePoint(0);
      DogLeg[3]->AppliedInverseSolution();

      LegsSendMotorCommands();

      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    DogLeg[0]->StepDistance = Configuration->dog.controller.stepDistance * 1;
    DogLeg[1]->StepDistance = Configuration->dog.controller.stepDistance * 1;
    DogLeg[2]->StepDistance = Configuration->dog.controller.stepDistance * 1;
    DogLeg[3]->StepDistance = Configuration->dog.controller.stepDistance * 1;

    DogLeg[0]->StepHeight = Configuration->dog.controller.stepHigh * 1;
    DogLeg[1]->StepHeight = Configuration->dog.controller.stepHigh * 1;
    DogLeg[2]->StepHeight = Configuration->dog.controller.stepHigh * 1;
    DogLeg[3]->StepHeight = Configuration->dog.controller.stepHigh * 1;

    currentBodyState = state;

    AbsoluteTimeDelay(500);
    break;
  case DogSquart:
  case DogStand:
    DHeight = zhanligaodu;

    if (state == DogSquart)
      DHeight = dunxiagaodu;

    while (1) {
      if (InsideTheErrorHeight(DogLeg[0]->StepStandingHeight, DHeight) &&
          InsideTheErrorHeight(DogLeg[1]->StepStandingHeight, DHeight) &&
          InsideTheErrorHeight(DogLeg[2]->StepStandingHeight, DHeight) &&
          InsideTheErrorHeight(DogLeg[3]->StepStandingHeight, DHeight)) {
        break;
      }

      // 0.05 mm/次 的速度接近
      for (int i = 0; i < 4; i++) {
        if (DogLeg[i]->StepStandingHeight > DHeight)
          DogLeg[i]->StepStandingHeight = DogLeg[i]->StepStandingHeight - rate;
        else
          DogLeg[i]->StepStandingHeight = DogLeg[i]->StepStandingHeight + rate;

        // 算出站立姿态是多少角度，
        DogLeg[i]->setCurrentTimePoint(0);
        DogLeg[i]->AppliedInverseSolution();
      }

      // 将数据发送到电机
      LegsSendMotorCommands();
      // delay
      AbsoluteTimeDelay(SystemFerq);
    }

    // 调整完成，设定最终高度
    for (int i = 0; i < 4; i++) {
      DogLeg[i]->StepStandingHeight = DHeight;

      // 算出站立姿态是多少角度，
      DogLeg[i]->setCurrentTimePoint(0);
      DogLeg[i]->AppliedInverseSolution();
    }

    currentBodyState = state;

    break;
  }
}

void QCDogggBrainScheduler::pause() {
  if (isRunning()) {
    pauseFlag = true;
  }
}

void QCDogggBrainScheduler::resume() {
  if (isRunning()) {
    pauseFlag = false;
    waitCondition.wakeAll();
  }
}

void QCDogggBrainScheduler::ProcessingControlCommand() {
  // 看看遥控器要切换至啥状态

  switch (Configuration->dog.controller.FuselageGesture) {
  case 0:
    targetBodyState = DogNull;
    break;
  case 1:
    targetBodyState = DogSquart;
    break;
  case 2:
    targetBodyState = DogStand;
    break;
  case 3:
    targetBodyState = DogWalk;
    break;
  case 4:
    targetBodyState = DogWalkTriangularGait;
    break;
  case 5:
    targetBodyState = DogRotation;
    break;
  case 6:
    targetBodyState = DogWalkStairs;
    break;
  case 7:
    targetBodyState = DogDrop;
    break;
  }

  emit uploadLogs(QStringLiteral("设定姿态 %1\n").arg(targetBodyState), 0);

  // 检查速率是不是给的超界
  if (Configuration->dog.controller.stepRate < 17 &&
      Configuration->dog.controller.stepRate >= 1) {
    if (abs(Configuration->dog.controller.stepRate - StepFrequencyCoefficient) >
        1) {
      emit uploadLogs(
          QStringLiteral("设定[步频系数]突变过大,当前值:%1 已恢复\n")
              .arg(StepFrequencyCoefficient),
          0);
      Configuration->dog.controller.stepRate = StepFrequencyCoefficient;
    } else {
      StepFrequencyCoefficient = Configuration->dog.controller.stepRate;
    }
  } else {
    emit uploadLogs(QStringLiteral("设定[步频系数]超出范围,设定值:%1 已恢复\n")
                        .arg(Configuration->dog.controller.stepRate),
                    0);
    Configuration->dog.controller.stepRate = StepFrequencyCoefficient;
  }

  // 检查步距是否合理
  if (Configuration->dog.controller.stepDistance >= 1 &&
      Configuration->dog.controller.stepDistance <= 150) {

  } else {
    emit uploadLogs(QStringLiteral("设定[步距]超出范围,已重置为50,设定值:%1\n")
                        .arg(Configuration->dog.controller.stepDistance),
                    0);
    Configuration->dog.controller.stepDistance = 50;
  }

  // 设定[腿刚度]
  // if()

  isUpdates = true;

  resume();
}

void QCDogggBrainScheduler::AbsoluteTimeDelay(int ms) {
  static long long PreviousTimestamp = QDateTime::currentMSecsSinceEpoch();

  if (Configuration->dog.controller.FreezeDog) {
    emit uploadLogs(QStringLiteral("任务已冻结\n"), 0);
    FreezeLoop->exec();
    emit uploadLogs(QStringLiteral("冻结解除\n"), 0);
  }

  while ((QDateTime::currentMSecsSinceEpoch() - PreviousTimestamp) < ms) {
  }

  systemSchedulingInterval =
      QDateTime::currentMSecsSinceEpoch() - PreviousTimestamp;
  PreviousTimestamp = QDateTime::currentMSecsSinceEpoch();
}
