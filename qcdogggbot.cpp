#include "qcdogggbot.h"

QCDogggBot::QCDogggBot(QObject *parent) : QObject(parent) {
  dogggBrain = new QCDogggBrain(this);
  dogggScheduler = new QCDogggBrainScheduler(dogggBrain);

  setConnection();

  dogggBrain->InitializationBrain(dogggScheduler);
}

QCDogggBot::~QCDogggBot() {}

void QCDogggBot::setConnection() {}
