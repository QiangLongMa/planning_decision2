#ifndef RUNCAM_H
#define RUNCAM_H
#include <iostream>
#include <QDebug>
#include <QObject>
#include <QThread>

class runcam : public QThread{
    Q_OBJECT
    public:
        runcam();
        void run() override;
};

#endif //RCLCOMM_H