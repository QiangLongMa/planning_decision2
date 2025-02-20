#ifndef RUNCONTROL_H
#define RUNCONTROL_H
#include <iostream>
#include <QDebug>
#include <QObject>
#include <QThread>

class runcontrol : public QThread {
  //使用了Q_OBJECT宏，这表示runcontrol类使用了Qt的元对象系统，因此可以使用Qt的信号与槽机制。
  Q_OBJECT  
 public:
  runcontrol();
  void run() override;  
};
#endif  // RCLCOMM_H
