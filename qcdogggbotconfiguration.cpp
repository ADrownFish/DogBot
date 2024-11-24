#include "qcdogggbotconfiguration.h"

QCDogggBotConfiguration::QCDogggBotConfiguration(QObject *p) : QObject(p) {
  // 初始化
  memset(&DogggMotor, 0, sizeof(DogggMotor));
  memset(&dog, 0, sizeof(dog));

  dog.controller.stepRate = 10;
  dog.controller.stepDistance = 30;
  dog.controller.stepHigh = -30;
  dog.controller.stepDistanceDifferential = 30;
}

QString QCDogggBotConfiguration::parsingControlCommands(QString cmd) {
  //\n位置不是在末尾
  if (cmd.indexOf("\n") != (cmd.size() - 1))
    return QStringLiteral("命令不完整，结尾没有发现 结束符\\n\n");

  // 移除
  cmd.remove("\n");

  QStringList cmdList = cmd.split(" ");

  // 参数不够
  if (cmdList.size() < 2)
    return QStringLiteral("命令不完整，可用参数少于 2 个\n");

  if (cmdList[0] == "r" && cmdList.size() == 2) {
    if (cmdList[1] == "dog.host.CPU") {
      return QString("%1\n").arg(dog.host.CPU);
    } else if (cmdList[1] == "dog.host.lock") {
      return QString("%1\n").arg(dog.host.lock);
    } else if (cmdList[1] == "dog.host.ram") {
      return QString("%1\n").arg(dog.host.ram);
    } else if (cmdList[1] == "dog.host.release") {
      return QString("%1\n").arg(dog.host.release);
    }

    else if (cmdList[1] == "dog.controller.FuselageGesture") {
      return QString("%1\n").arg(dog.controller.FuselageGesture);
    } else if (cmdList[1] == "dog.controller.stepDistance") {
      return QString("%1\n").arg(dog.controller.stepDistance);
    } else if (cmdList[1] == "dog.controller.stepHigh") {
      return QString("%1\n").arg(dog.controller.stepHigh);
    } else if (cmdList[1] == "dog.controller.stepRate") {
      return QString("%1\n").arg(dog.controller.stepRate);
    } else if (cmdList[1] == "dog.controller.stepStandTall") {
      return QString("%1\n").arg(dog.controller.stepStandTall);
    } else if (cmdList[1] == "dog.controller.stiffness") {
      return QString("%1\n").arg(dog.controller.stiffness);
    } else if (cmdList[1] == "dog.controller.UploadFootPos") {
      return QString("%1\n").arg(dog.controller.UploadFootPos);
    }
    // 是否开机变量就不查询了
    else if (cmdList[1] == "dog.controller.EmergencyStop") {
      return QString("%1\n").arg(dog.controller.EmergencyStop);
    }

    else if (cmdList[1] == "dog.actuator.leg0.calfLength") {
      return QString("%1\n").arg(dog.actuator.leg0.calfLength);
    } else if (cmdList[1] == "dog.actuator.leg0.MotorMechanicalAngleA") {
      return QString("%1\n").arg(dog.actuator.leg0.MotorMechanicalAngleA);
    } else if (cmdList[1] == "dog.actuator.leg0.MotorMechanicalAngleB") {
      return QString("%1\n").arg(dog.actuator.leg0.MotorMechanicalAngleB);
    } else if (cmdList[1] == "dog.actuator.leg0.pos.x") {
      return QString("%1\n").arg(dog.actuator.leg0.pos.x);
    } else if (cmdList[1] == "dog.actuator.leg0.pos.z") {
      return QString("%1\n").arg(dog.actuator.leg0.pos.z);
    } else if (cmdList[1] == "dog.actuator.leg0.thighLength") {
      return QString("%1\n").arg(dog.actuator.leg0.thighLength);
    } else if (cmdList[1] == "dog.actuator.leg0.trajectoryProgress") {
      return QString("%1\n").arg(dog.actuator.leg0.trajectoryProgress);
    }

    else if (cmdList[1] == "dog.actuator.leg1.calfLength") {
      return QString("%1\n").arg(dog.actuator.leg1.calfLength);
    } else if (cmdList[1] == "dog.actuator.leg1.MotorMechanicalAngleA") {
      return QString("%1\n").arg(dog.actuator.leg1.MotorMechanicalAngleA);
    } else if (cmdList[1] == "dog.actuator.leg1.MotorMechanicalAngleB") {
      return QString("%1\n").arg(dog.actuator.leg1.MotorMechanicalAngleB);
    } else if (cmdList[1] == "dog.actuator.leg1.pos.x") {
      return QString("%1\n").arg(dog.actuator.leg1.pos.x);
    } else if (cmdList[1] == "dog.actuator.leg1.pos.z") {
      return QString("%1\n").arg(dog.actuator.leg1.pos.z);
    } else if (cmdList[1] == "dog.actuator.leg1.thighLength") {
      return QString("%1\n").arg(dog.actuator.leg1.thighLength);
    } else if (cmdList[1] == "dog.actuator.leg1.trajectoryProgress") {
      return QString("%1\n").arg(dog.actuator.leg1.trajectoryProgress);
    }

    else if (cmdList[1] == "dog.actuator.leg2.calfLength") {
      return QString("%1\n").arg(dog.actuator.leg2.calfLength);
    } else if (cmdList[1] == "dog.actuator.leg2.MotorMechanicalAngleA") {
      return QString("%1\n").arg(dog.actuator.leg2.MotorMechanicalAngleA);
    } else if (cmdList[1] == "dog.actuator.leg2.MotorMechanicalAngleB") {
      return QString("%1\n").arg(dog.actuator.leg2.MotorMechanicalAngleB);
    } else if (cmdList[1] == "dog.actuator.leg2.pos.x") {
      return QString("%1\n").arg(dog.actuator.leg2.pos.x);
    } else if (cmdList[1] == "dog.actuator.leg2.pos.z") {
      return QString("%1\n").arg(dog.actuator.leg2.pos.z);
    } else if (cmdList[1] == "dog.actuator.leg2.thighLength") {
      return QString("%1\n").arg(dog.actuator.leg2.thighLength);
    } else if (cmdList[1] == "dog.actuator.leg2.trajectoryProgress") {
      return QString("%1\n").arg(dog.actuator.leg2.trajectoryProgress);
    }

    else if (cmdList[1] == "dog.actuator.leg3.calfLength") {
      return QString("%1\n").arg(dog.actuator.leg3.calfLength);
    } else if (cmdList[1] == "dog.actuator.leg3.MotorMechanicalAngleA") {
      return QString("%1\n").arg(dog.actuator.leg3.MotorMechanicalAngleA);
    } else if (cmdList[1] == "dog.actuator.leg3.MotorMechanicalAngleB") {
      return QString("%1\n").arg(dog.actuator.leg3.MotorMechanicalAngleB);
    } else if (cmdList[1] == "dog.actuator.leg3.pos.x") {
      return QString("%1\n").arg(dog.actuator.leg3.pos.x);
    } else if (cmdList[1] == "dog.actuator.leg3.pos.z") {
      return QString("%1\n").arg(dog.actuator.leg3.pos.z);
    } else if (cmdList[1] == "dog.actuator.leg3.thighLength") {
      return QString("%1\n").arg(dog.actuator.leg3.thighLength);
    } else if (cmdList[1] == "dog.actuator.leg3.trajectoryProgress") {
      return QString("%1\n").arg(dog.actuator.leg3.trajectoryProgress);
    } else
      return QStringLiteral("没有这个变量\n");
  } else if (cmdList[0] == "w" && cmdList.size() == 3) {
    bool Matchsuccess = false;

#define RETURNSTRING QStringLiteral("ok\n")

    if (cmdList[1] == "dog.controller.FuselageGesture") {
      dog.controller.FuselageGesture = cmdList[2].toInt();
      Matchsuccess = true;
    } else if (cmdList[1] == "dog.controller.stepDistance") {
      dog.controller.stepDistance = cmdList[2].toInt();
      Matchsuccess = true;
    } else if (cmdList[1] == "dog.controller.stepHigh") {
      dog.controller.stepHigh = cmdList[2].toInt();
      Matchsuccess = true;
      if (dog.controller.stepHigh > 0) {
        dog.controller.stepHigh = -dog.controller.stepHigh;
      } else {
        dog.controller.stepHigh = -75;
      }
    } else if (cmdList[1] == "dog.controller.stepRate") {
      dog.controller.stepRate = cmdList[2].toInt();
      Matchsuccess = true;
      ;
    } else if (cmdList[1] == "dog.controller.stepStandTall") {
      dog.controller.stepStandTall = cmdList[2].toInt();
      Matchsuccess = true;
    } else if (cmdList[1] == "dog.controller.stiffness") {
      dog.controller.stiffness = cmdList[2].toFloat();
      Matchsuccess = true;
    } else if (cmdList[1] == "dog.controller.UploadFootPos") {
      dog.controller.UploadFootPos = QVariant(cmdList[2]).toBool();
      Matchsuccess = true;
    } else if (cmdList[1] == "dog.controller.InitDog") {
      dog.controller.InitDog = QVariant(cmdList[2]).toBool();
      Matchsuccess = true;
    } else if (cmdList[1] == "dog.controller.EmergencyStop") {
      dog.controller.EmergencyStop = QVariant(cmdList[2]).toBool();
      Matchsuccess = true;
    } else if (cmdList[1] == "dog.controller.FreezeDog") {
      dog.controller.FreezeDog = QVariant(cmdList[2]).toBool();
      Matchsuccess = true;
      if (!dog.controller.FreezeDog) {
        emit UnFreezeDog();
      }
    }

    else if (cmdList[1] == "dog.controller.stepDistanceDifferential") {
      dog.controller.stepDistanceDifferential = cmdList[2].toInt();
      Matchsuccess = true;
    }

    else
      return QStringLiteral("没有这个变量\n");

    if (Matchsuccess) {
      m_GetUpdates = true;
      return RETURNSTRING;
    }
  } else if (cmdList[0] == "e") {
    if (cmdList[1] == "quit()") {
    } else if (cmdList[1] == "init()") {
    } else
      return QStringLiteral("没有这个函数\n");
  }
}

bool QCDogggBotConfiguration::GetUpdates() { return m_GetUpdates; }

void QCDogggBotConfiguration::setExpired() { m_GetUpdates = false; }
