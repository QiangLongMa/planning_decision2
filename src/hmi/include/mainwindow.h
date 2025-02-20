#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "opencv2/opencv.hpp"
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QFileDialog>
#include <QTimer>
#include <QPoint>
#include <QObject>
#include <QPainter>
#include <QVector> 
#include <QtGlobal>
#include <QCloseEvent>
#include "mItem.h"
#include "mmItem.h"
#include  <unistd.h>
#include "Eigen/Eigen"
#include "toolKits.h"
#include "rclcomm.h"
#include "rungps.h"
#include "runglobal.h"
#include "runlocal.h"
#include "runcam.h"
#include "runlidar.h"
#include "runcontrol.h"
#include "runcenterpoint.h"
#include"runsocketcan.h"
#include <cstdlib>
#include"runrviz.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    // bool eventFilter(QObject *target, QEvent *event);

private slots:
    void pushButton_clicked();
    void pushButton_2clicked();
    void pushButton_3clicked();
    void pushButton_4clicked();
    void pushButton_5clicked();
    void pushButton_6clicked();
    void pushButton_7clicked();
    void pushButton_8clicked();
    void label_6update();
    void recvGpsCallback(qreal, qreal, QString, QString);
    void recvGlobalCallback(QVector<qreal>);
    void recvCamCallback(QVector<qreal>);
    void recvLidarCallback(QVector<qreal>);
    // void recvPointCloudCallback(QVector<qreal>);
    void recvLocalCallback(QVector<qreal>);
    void recvLocalCallback_1(QVector<qreal>);

    void recstopline(qreal);


 

    void lineEditState();
    void callGps();
    void callGlobal();
    void callLocal();
    void carPosUpdate();
    void callCam();
    void callLidar();
    void callcenterpoint();
    void callsocketcan();
    void callControl();
    void callrviz();
    void camObjPosUpdate();
    void lidarObjPosUpdate();

    // void pointCloudUpdate();
    void roadUpdate();
    void localTrajUpdate();
    void localTrajUpdate_1();



 signals:
    void emitGps();
    void emitGlobal();
    void emitLocal();
    void emitCam();
    void emitLidar();
    void emitControl();
    void emitcenterpoint();
    void emitsocketcan();
    void emitVelocityAndGearsData(QVector<qint64>);
    void emitrviz();


    // void getDestination(QPointF);

protected:
    void closeEvent (QCloseEvent *event) override; 

private:
    Ui::MainWindow *ui;
    rclcomm *commNode;  //communication Node  Def
    rungps *trungps;
    runglobal *trunglobal;
    runlocal *trunlocal;
    runcam *truncam;
    runlidar *trunlidar;
    runcontrol *truncontrol;
    runcenterpoint *turnenterpoint;
    runsocketcan *trunsocketcan;
    runrviz * trunrviz;
    


    QGraphicsScene* scene1; // 场景left
    QGraphicsScene* scene2; // 场景right
    QGraphicsPixmapItem* srcPixmap;	// 对象元素
    QImage* srcImage;	// QImage对象
    cv::Mat* srcMat;	//OpenCV中的Mat对象
    QImage m_images;

    // mItem mmItem Objs  Def;
    mItem *item = new mItem();
    mmItem *mm_item = new mmItem();

    //subscribe Local trajs
    double auto_SW = -25.6;//需要实时显示的变量
    //
    // int set_speed=0;

    // subscribe GPS
    double gpsX = 0.0; 
    double gpsY = 0.0;  
    double heading = 0.0; 
    double speed = 0.0;    

    //map variables
    Eigen::VectorXd  car; 
    Eigen::Matrix<double, 1, 2> carImagePosition;
    Eigen::VectorXd realCar;
    double x, y;
    QPointF car_pos;
    // Eigen::Matrix<double, 7, Eigen::Dynamic>  globalPath;
    Eigen::MatrixXd globalPath;
    Eigen::MatrixXd localPath;
    Eigen::MatrixXd localPath1;
    //subscribe Cam, Lidar
    Eigen::MatrixXd cam_objs;
    Eigen::MatrixXd lidar_objs;
    Eigen::MatrixXd pointCloud_objs;

    float stopline;

};
#endif // MAINWINDOW_H
