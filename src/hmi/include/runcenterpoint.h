#ifndef RUNCENTERPOINT_H
#define RUNCENTERPOINT_H
#include <iostream>
#include <QDebug>
#include <QObject>
#include <QThread>

class runcenterpoint : public QThread{
    Q_OBJECT
    public:
        runcenterpoint();
        void run() override;
};

#endif //RUNCENTERPOINT_H