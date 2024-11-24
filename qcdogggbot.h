#ifndef QCDOGGGBOT_H
#define QCDOGGGBOT_H

#include <QObject>
#include <QTextStream>

#include "qcdogggbotconfiguration.h"
#include "qcdogggbrain.h"
#include "qcdogggbrainscheduler.h"
#include "qcdogggnetwork.h"

class QCDogggBot : public QObject {
  Q_OBJECT

public:
  explicit QCDogggBot(QObject *parent = nullptr);
  ~QCDogggBot();

private:
  // 设定信号链接
  void setConnection();

private:
  // 控制大脑和调度器
  QCDogggBrain *dogggBrain;
  QCDogggBrainScheduler *dogggScheduler;

signals:

public slots:
};

#endif // QCDOGGGBOT_H
