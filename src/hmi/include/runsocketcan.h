#ifndef RUNSOCKETCAN_H
#define RUNSOCKETCAN_H
#include <iostream>
#include <QDebug>
#include <QObject>
#include <QThread>

class runsocketcan : public QThread {
  Q_OBJECT  
 public:
  runsocketcan();
  void run() override;  
};
#endif  // RCLCOMM_H