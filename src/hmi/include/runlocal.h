#ifndef RUNLOCAL_H
#define RUNLOCAL_H
#include <iostream>
#include <QDebug>
#include <QObject>
#include <QThread>

class runlocal : public QThread {
  Q_OBJECT  
 public:
  runlocal();
  void run() override;  
};
#endif  // RCLCOMM_H
