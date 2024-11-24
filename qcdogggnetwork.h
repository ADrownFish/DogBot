#ifndef QCDOGGGNETWORK_H
#define QCDOGGGNETWORK_H

#include <QByteArray>
#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>

#include "qcdogggbotconfiguration.h"
#include "qcdogggleg.h"

#include "makeconfig.h"

#include "include/LSerial.h" //串口通信函数
#include <errno.h>           //错误定义
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h> //Unix标准函数定义, usleep()

class QCDogggNetwork : public QObject {
  Q_OBJECT

  friend class QCDogggBrain;
  friend class QCDogggBrainScheduler;

public:
  explicit QCDogggNetwork(QCDogggBotConfiguration *, QCDogggLeg **,
                          QObject *parent = nullptr);
  ~QCDogggNetwork();

  // 设置服务器sock监听
  void setServerListen();

  void SendUploadLogs(QString, int);
  void SendUploadLegPos();

public:
  // 文件句柄
  int fd0, fd1, fd2;
  int Ret0, Ret1, Ret2;
  int epfd0, epfd1, epfd2;

  bool m_initDog = false;

  // 服务器
  QTcpServer *server;
  QTcpSocket *sock;

  // config
  QCDogggBotConfiguration *config;

  // leg
  QCDogggLeg *DogLeg[4];

signals:
  void connectedFromUpper();
  void disconnectedFromUpper();
  void hasNewCMD();
};

#endif // QCDOGGGNETWORK_H
