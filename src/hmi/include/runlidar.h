#ifndef RUNLIDAR_H
#define RUNLIDAR_H
#include <iostream>
#include <QDebug>
#include <QObject>
#include <QThread>

class runlidar : public QThread{
    Q_OBJECT
    public:
        runlidar();
        void run() override;
};
#endif //RCLCOMM_H