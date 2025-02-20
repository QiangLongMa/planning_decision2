#ifndef RUNGPS_H
#define RUNGPS_H
#include <iostream>
#include <QDebug>
#include <QObject>
#include <QThread>

class rungps : public QThread {
  Q_OBJECT  
 public:
  rungps();
  void run() override;  
};
#endif  // RCLCOMM_H
