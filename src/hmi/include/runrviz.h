#ifndef RUNRVIZ_H
#define RUNRVIZ_H
#include <iostream>
#include <QDebug>
#include <QObject>
#include <QThread>

class runrviz : public QThread {
  Q_OBJECT  
 public:
  runrviz();
  void run() override;  
};
#endif  // RCLCOMM_H
