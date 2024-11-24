#include "qcdogggnetwork.h"

QCDogggNetwork::QCDogggNetwork(QCDogggBotConfiguration *p, QCDogggLeg **p2,
                               QObject *parent)
    : QObject(parent), sock(NULL) {
  // config pointer ..
  config = p;

  DogLeg[0] = p2[0];
  DogLeg[1] = p2[1];
  DogLeg[2] = p2[2];
  DogLeg[3] = p2[3];

  //=================serial 流 操作==========================
  /*
   * 以下操作的是 unitree A1 motor ，如果要更换其他电机，需要修改这个socket fd
   */
  // 打开设备文件
  char dev0Name[] = "/dev/ttyUSB0";
  char dev1Name[] = "/dev/ttyUSB1";
  char dev2Name[] = "/dev/ttyUSB2";

  // open device
  fd0 = open_set(dev0Name);
  fd1 = open_set(dev1Name);
  fd2 = open_set(dev2Name);

  epfd0 = epoll_create(1);
  epfd1 = epoll_create(1);
  epfd2 = epoll_create(1);

  struct epoll_event eventTest0;
  eventTest0.events = EPOLLIN;
  eventTest0.data.fd = fd0;
  Ret0 = epoll_ctl(epfd0, EPOLL_CTL_ADD, fd0, &eventTest0);
  if (Ret0 != 0) {
    printf("[ERROR] 1 Serial communication: epoll set error: %d\n", Ret0);
  }

  struct epoll_event eventTest1;
  eventTest1.events = EPOLLIN;
  eventTest1.data.fd = fd1;
  Ret1 = epoll_ctl(epfd1, EPOLL_CTL_ADD, fd1, &eventTest1);
  if (Ret1 != 0) {
    printf("[ERROR] 2 Serial communication: epoll set error: %d\n", Ret1);
  }

  struct epoll_event eventTest2;
  eventTest2.events = EPOLLIN;
  eventTest2.data.fd = fd2;
  Ret2 = epoll_ctl(epfd2, EPOLL_CTL_ADD, fd2, &eventTest2);
  if (Ret2 != 0) {
    printf("[ERROR] 3 Serial communication: epoll set error: %d\n", Ret2);
  }

  //=================TCP服务器初始化==========================

  // 配置服务器端口监听
  server = new QTcpServer(this);
}

QCDogggNetwork::~QCDogggNetwork() {
  // 关闭串口
  close_serial(fd0);
  close_serial(fd1);
  close_serial(fd2);

  // 关闭TCP服务器监听
  server->close();
  if (sock)
    sock->deleteLater();
}

// 监听TCP的，上位机连接用的
void QCDogggNetwork::setServerListen() {
  if (!server->listen(QHostAddress::Any, 25565)) {
    printf("监听网络端口失败 !\n");
  } else {
    printf("监听网络端口成功!\n");
    printf("当前 ipv4: %s   Port: %hu \n",
           server->serverAddress().toString().toLatin1(), server->serverPort());
    printf("等待上位机建立连接\n");

    connect(server, &QTcpServer::newConnection, this, [&]() {
      if (sock) {
        disconnect(sock);
        sock->deleteLater();
        sock = NULL;
      }

      printf("建立一个新的TCP Sock.\n");
      sock = server->nextPendingConnection();
      sock->write(
          QStringLiteral("已成功建立socket!  dog dog dog\n").toLocal8Bit());
      sock->flush();

      connect(sock, &QTcpSocket::readyRead, this, [&]() {
        static QString bufferPool;
        static QStringList BufferTemp;

        // sock->write();
        QString buffer;
        printf(buffer.toLatin1());

        if (bufferPool.isEmpty()) {
          buffer = sock->readAll();
        } else {
          buffer = bufferPool + QString(sock->readAll());
          bufferPool.clear();
        }

        BufferTemp = buffer.split("\n");

        for (int i = 0; i < BufferTemp.size(); i++) {
          if (!BufferTemp[i].isEmpty()) {
            QString recvData = BufferTemp[i] + "\n";
            // SendUploadLogs("接收到的指令为 >> "+recvData,1);
            SendUploadLogs(config->parsingControlCommands(recvData), 1);
          }
        }
        // 如果设置被更新
        if (m_initDog && config->GetUpdates()) {
          emit hasNewCMD();
        }

        if (config->dog.controller.InitDog) {
          config->dog.controller.InitDog = false;
          m_initDog = true;
        }
      });

      connect(sock, &QTcpSocket::disconnected, this, [&]() {
        sock = NULL;
        printf("已断开 TCP Sock.\n");

        emit disconnectedFromUpper();
      });
    });
  }
}

void QCDogggNetwork::SendUploadLogs(QString logs, int mode) {
  printf(logs.toLocal8Bit());

  if (sock) {
    // 信息 ：back information
    switch (mode) {
    case 0:
      sock->write(QString("bi#" + logs).toLocal8Bit());
      break;
    case 1:
      sock->write(QString("bv#" + logs).toLocal8Bit());
      break;
    }
    sock->flush();
  }
}

void QCDogggNetwork::SendUploadLegPos() {
#if (SimulationMode == 0)
  // 信息 ：back pos
  QString var = QString("bp#%1,%2,%3,%4,%5,%6,%7,%8\n")
                    .arg(QString::number(DogLeg[0]->ActualFootEndX),
                         QString::number(DogLeg[0]->ActualFootEndZ),

                         QString::number(DogLeg[1]->ActualFootEndX),
                         QString::number(DogLeg[1]->ActualFootEndZ),

                         QString::number(DogLeg[2]->ActualFootEndX),
                         QString::number(DogLeg[2]->ActualFootEndZ),

                         QString::number(DogLeg[3]->ActualFootEndX),
                         QString::number(DogLeg[3]->ActualFootEndZ));
#else
  // 信息 ：back pos
  QString var = QString("bp#%1,%2,%3,%4,%5,%6,%7,%8\n")
                    .arg(QString::number(DogLeg[0]->DesiredFootEndX),
                         QString::number(DogLeg[0]->DesiredFootEndZ),

                         QString::number(DogLeg[1]->DesiredFootEndX),
                         QString::number(DogLeg[1]->DesiredFootEndZ),

                         QString::number(DogLeg[2]->DesiredFootEndX),
                         QString::number(DogLeg[2]->DesiredFootEndZ),

                         QString::number(DogLeg[3]->DesiredFootEndX),
                         QString::number(DogLeg[3]->DesiredFootEndZ));
#endif

  sock->write(var.toLocal8Bit());
  sock->flush();
}
