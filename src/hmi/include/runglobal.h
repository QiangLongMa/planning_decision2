#ifndef RUNGLOBAL_H
#define RUNGLOBAL_H
#include <iostream>
#include <QDebug>
#include <QObject>
#include <QThread>

class runglobal : public QThread {
  Q_OBJECT  
 public:
  runglobal();
  void run() override;  
};
#endif  // RCLCOMM_H
