#include "qcdogggbrain.h"

QCDogggBrain::QCDogggBrain(QObject *parent) : QObject(parent) {}
QCDogggBrain::~QCDogggBrain() {}

void QCDogggBrain::InitializationBrain(QCDogggBrainScheduler *pointer) {
  Scheduler = pointer;

  // 将电机匹配到对应的腿上
  Configuration = new QCDogggBotConfiguration(this);

  DogLeg[0] = new QCDogggLeg(&Configuration->DogggMotor.MotorSend[0],
                             &Configuration->DogggMotor.MotorSend[1],
                             &Configuration->DogggMotor.MotorRecv[0],
                             &Configuration->DogggMotor.MotorRecv[1],
                             LegType::LeftFrontLeg);
  DogLeg[1] = new QCDogggLeg(&Configuration->DogggMotor.MotorSend[2],
                             &Configuration->DogggMotor.MotorSend[3],
                             &Configuration->DogggMotor.MotorRecv[2],
                             &Configuration->DogggMotor.MotorRecv[3],
                             LegType::RightFrontLeg);
  DogLeg[2] = new QCDogggLeg(&Configuration->DogggMotor.MotorSend[4],
                             &Configuration->DogggMotor.MotorSend[5],
                             &Configuration->DogggMotor.MotorRecv[4],
                             &Configuration->DogggMotor.MotorRecv[5],
                             LegType::LeftHindLeg);
  DogLeg[3] = new QCDogggLeg(&Configuration->DogggMotor.MotorSend[6],
                             &Configuration->DogggMotor.MotorSend[7],
                             &Configuration->DogggMotor.MotorRecv[6],
                             &Configuration->DogggMotor.MotorRecv[7],
                             LegType::RightHindLeg);

  DogLeg[0]->DesiredHeightError = -10;
  DogLeg[1]->DesiredHeightError = -6;

  // 网络管理器：电机数据发送，接受指令
  Network = new QCDogggNetwork(Configuration, DogLeg);
  connect(Scheduler, &QCDogggBrainScheduler::uploadLogs, Network,
          &QCDogggNetwork::SendUploadLogs);
  connect(Scheduler, &QCDogggBrainScheduler::uploadLegPos, Network,
          &QCDogggNetwork::SendUploadLegPos);

  connect(this, &QCDogggBrain::uploadLogs, Network,
          &QCDogggNetwork::SendUploadLogs);
  connect(Network, &QCDogggNetwork::hasNewCMD, Scheduler,
          &QCDogggBrainScheduler::ProcessingControlCommand);

  Network->setServerListen();

  QEventLoop loop;

  connect(Configuration, &QCDogggBotConfiguration::UnFreezeDog, &loop,
          &QEventLoop::quit);
  loop.exec();
  emit uploadLogs(QStringLiteral("正在准备初始化.\n"), 0);
  printf("准备初始化 然后解除信号");
  disconnect(&loop);

  // start the thread.
  emit uploadLogs(QStringLiteral(" 2s 后启动线程.\n"), 0);
  QThread::sleep(2);

  Scheduler->setInit(Configuration, Network, DogLeg);
  Scheduler->start(QThread::HighestPriority);

  // start watchdog
  // connect(&SystemSoftwareWatchDogTimer,&QTimer::timeout,this,&QCDogggBrain::SystemSoftwareWatchDog);
  // SystemSoftwareWatchDogTimer.start(1000);
}

void QCDogggBrain::SystemSoftwareWatchDog() {
  // LogOutput(LogType_t::Info,QString("Sys Freq: %1
  // hz").arg(Scheduler->getSystemFrequency())); emit
  // LogOutput(LogType_t::Info,QString("posA: %1  posB: %2 . ")
  //                .arg(Scheduler->*57.3f).arg(DogLeg[0]->DesiredPosB *57.3f));
}
