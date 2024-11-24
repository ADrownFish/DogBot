#ifndef QCDOGGGBRAIN_H
#define QCDOGGGBRAIN_H

#include <QCoreApplication>
#include <QDateTime>
#include <QEventLoop>
#include <QObject>
#include <QThread>
#include <QTimer>

#include "math.h"

#include "qcdogggbotconfiguration.h"
#include "qcdogggbrainscheduler.h"
#include "qcdogggleg.h"

class QCDogggBrain : public QObject {
  Q_OBJECT
public:
  explicit QCDogggBrain(QObject *parent = nullptr);
  ~QCDogggBrain();

  // 初始化机器人
  void InitializationBrain(QCDogggBrainScheduler *);
  // 调度器线程监视
  void SystemSoftwareWatchDog();

private:
  QCDogggBrainScheduler *Scheduler;

  QCDogggBotConfiguration *Configuration;
  QCDogggNetwork *Network;
  QCDogggLeg *DogLeg[4];

  QTimer SystemSoftwareWatchDogTimer;

signals:
  void uploadLogs(QString, int);
};

#endif // QCDOGGGBRAIN_H
