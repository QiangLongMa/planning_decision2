#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<math.h>
#include<cmath>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow){
    ui->setupUi(this);
    //scene Obj Def
    scene1 = new QGraphicsScene(); //用于在场景中操作大量的2D图形元素
    scene2 = new QGraphicsScene();
    srcPixmap = new QGraphicsPixmapItem();
    srcImage = new QImage();
    srcMat = new cv::Mat();
    // ui Initialization   
    ui->label->setText("Load Map");
    ui->label->setWordWrap(false); // 设置是否允许换行 
    ui->lineEdit->setEnabled(false);

    ui->pushButton_2->setEnabled(false);// check gps
    ui->pushButton_3->setEnabled(false);//get path
    ui->pushButton_4->setEnabled(false);//check cam
    ui->pushButton_5->setEnabled(false);//lidar
    ui->pushButton_6->setEnabled(false);//radar
    ui->pushButton_7->setEnabled(false);//run
    ui->pushButton_8->setEnabled(false); //stop

    //启用鼠标穿透
    /*这段代码是用于设置Qt图形视图部件（graphicsView）为鼠标事件透明。当设置为true时，意味着该部件将不会接收鼠标事件，
    而是会将这些事件传递给其下方的部件进行处理。这通常用于在图形视图中叠加多个部件时，可以使下方的部件能够接收鼠标事件。*/
    ui->graphicsView->setAttribute(Qt::WA_TransparentForMouseEvents,true);

    // communication Node Def 
    commNode=new rclcomm();
    commNode->start();
    trungps = new rungps();
    trunglobal = new runglobal();
    trunlocal = new runlocal();
    truncam = new runcam();
    trunlidar = new runlidar();
    truncontrol = new runcontrol();
    turnenterpoint = new runcenterpoint();
    trunsocketcan = new runsocketcan();
    trunrviz = new runrviz();

    // connect函数
    //checkBox connect  TEST 
    connect(ui->checkBox, SIGNAL(stateChanged(int)), this, SLOT(lineEditState()));
    //Button connect
    connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(pushButton_clicked())); //load map
    connect(ui->pushButton_2, SIGNAL(clicked()), this, SLOT(pushButton_2clicked())); //check GPS
    connect(ui->pushButton_3, SIGNAL(clicked()), this, SLOT(pushButton_3clicked())); //gen Path
    connect(ui->pushButton_4, SIGNAL(clicked()), this, SLOT(pushButton_4clicked())); //check cam
    connect(ui->pushButton_5, SIGNAL(clicked()), this, SLOT(pushButton_5clicked())); //check lidar
    connect(ui->pushButton_6, SIGNAL(clicked()), this, SLOT(pushButton_6clicked())); //check radar
    connect(ui->pushButton_7, SIGNAL(clicked()), this, SLOT(pushButton_7clicked())); //run
    connect(ui->pushButton_8, SIGNAL(clicked()), this, SLOT(pushButton_8clicked())); //stop

    //Label display connect
    //gps signal
    /*这段代码是用于连接一个名为commNode的对象的信号emitGpsTopicData到当前对象的槽函数recvGpsCallback。当commNode对象发出emitGpsTopicData信号时，
    当前对象的recvGpsCallback槽函数将被调用，并且传递四个参数：两个qreal类型的数值和两个QString类型的字符串。这样可以实现在两个对象之间进行数据传输和通信。*/
    connect(commNode, SIGNAL(emitGpsTopicData(qreal, qreal, QString, QString)), this,SLOT(recvGpsCallback(qreal, qreal, QString, QString)));
    //connect global path data
    connect(commNode, SIGNAL(emitGlobalData(QVector<qreal>)), this,SLOT(recvGlobalCallback(QVector<qreal>)));
    //connect cam data
    connect(commNode, SIGNAL(emitCamTopicData(QVector<qreal>)), this,SLOT(recvCamCallback(QVector<qreal>)));
    //connect lidar data
    // connect(commNode, SIGNAL(emitLidarTopicData(QVector<qreal>)), this,SLOT(recvLidarCallback(QVector<qreal>)));
    //connect pointCloud data
    // connect(commNode, SIGNAL(emitPointCloudData(QVector<qreal>)), this,SLOT(recvPointCloudCallback(QVector<qreal>)));
    //connect local data
    connect(commNode, SIGNAL(emitLocalData(QVector<qreal>)), this,SLOT(recvLocalCallback(QVector<qreal>)));
    //connect local data_1
    connect(commNode, SIGNAL(emitLocalData1(QVector<qreal>)), this,SLOT(recvLocalCallback_1(QVector<qreal>)));

    connect(commNode, SIGNAL(emitLidarTopicData(QVector<qreal>)), this,SLOT(recvLidarCallback(QVector<qreal>)));

    connect(commNode, SIGNAL(emitstopline(qreal)), this,SLOT(recstopline(qreal)));






    // //dest connect
    connect(item, SIGNAL(getDestination(QPointF)), commNode,SLOT(recvDestCallback(QPointF)));
    //check sensor ,ie. start xx node of  xx package 
    connect(this, SIGNAL(emitGps()), this,SLOT(callGps()));
    //check sensor ,ie. start xx node of  xx package 
    connect(this, SIGNAL(emitGlobal()), this,SLOT(callGlobal()));
    //pushButton run 
    connect(this, SIGNAL(emitLocal()), this,SLOT(callLocal()));
    connect(this, SIGNAL(emitControl()), this,SLOT(callControl()));
    connect(this, SIGNAL(emitCam()), this,SLOT(callCam()));
    connect(this, SIGNAL(emitLidar()), this,SLOT(callLidar()));
    connect(this, SIGNAL(emitcenterpoint()), this,SLOT(callcenterpoint()));
    connect(this,SIGNAL(emitsocketcan()),this,SLOT(callsocketcan()));
    connect(this,SIGNAL(emitrviz()),this,SLOT(callrviz()));


    //Start gps_node, global_node et...
    emit emitGps();
    sleep(0.1);
    emit emitGlobal();
    sleep(0.1);
    emit emitLocal();
    sleep(0.1);
    emit emitLidar();
    sleep(0.1);
    emit emitcenterpoint();
    sleep(0.1);
    emit emitControl();
    sleep(0.1);
    emit emitsocketcan();
    sleep(1);
    emit emitrviz();
    connect(this, SIGNAL(emitVelocityAndGearsData(QVector<qint64>)), commNode,SLOT(recvVelocityAndGearsData(QVector<qint64>)));
}

MainWindow::~MainWindow(){
    delete ui;
    delete item;
    delete mm_item;
    delete commNode;
    delete trungps;
    delete trunglobal;
    delete trunlocal;
    delete truncam;
    delete trunlidar;
    delete truncontrol;
    delete turnenterpoint;
    delete trunsocketcan;
    delete trunrviz;
    //system("ps -a | grep gnss_node | awk '{print $1}' | xargs kill -9");
    system("ps -a | grep global_node | awk '{print $1}' | xargs kill -15");    
    system("ps -a | grep local_node3 | awk '{print $1}' | xargs kill -15"); 
    system("ps -a | grep image_pub | awk '{print $1}' | xargs kill -15");  
    system("ps -a | grep detect_node | awk '{print $1}' | xargs kill -15");    
    system("ps -a | grep control_node | awk '{print $1}' | xargs kill -15");
    system("ps -a | grep can_node | awk '{print $1}' | xargs kill -15");   
    system("ps -a | grep Lidar_Processing_node | awk '{print $1}' | xargs kill -15");  
    system("ps -a | grep demo_node | awk '{print $1}' | xargs kill -15");  
    system("ps -a | grep points_and_lines_publisher | awk '{print $1}' | xargs kill -15");  

}

void MainWindow::callGps(){
    trungps->start();
}

void MainWindow::callGlobal(){
    trunglobal->start();
}

void MainWindow::callLocal(){
    trunlocal->start();
}

void MainWindow::callControl(){
    truncontrol->start();
}

void MainWindow::callCam(){
    truncam->start();
}
void MainWindow::callLidar(){
    trunlidar->start();
}

void MainWindow::callcenterpoint(){
    turnenterpoint->start();
}
void MainWindow::callsocketcan(){
    trunsocketcan->start();
}
void MainWindow::callrviz(){
    trunrviz->start();
}

void MainWindow::closeEvent (QCloseEvent *event) {
   // system("ps -a | grep gnss_node | awk '{print $1}' | xargs kill -9");
    system("ps -a | grep global_node | awk '{print $1}' | xargs kill -15");
    system("ps -a | grep local_node3 | awk '{print $1}' | xargs kill -15");
    system("ps -a | grep image_pub | awk '{print $1}' | xargs kill -15");  
    system("ps -a | grep detect_node | awk '{print $1}' | xargs kill -15");   
    system("ps -a | grep control_node | awk '{print $1}' | xargs kill -15");   
    system("ps -a | grep can_node | awk '{print $1}' | xargs kill -15"); 
    system("ps -a | grep Lidar_Processing_node | awk '{print $1}' | xargs kill -15");  
    system("ps -a | grep demo_node | awk '{print $1}' | xargs kill -15");     
    system("ps -a | grep points_and_lines_publisher | awk '{print $1}' | xargs kill -15");  
    event->accept(); 
}

void MainWindow::pushButton_clicked(){
    //load map
    /*设置按钮（pushButton）的文本颜色为绿色。
    设置标签（label）的文本内容为"Check Gps !"。
    将图形项（item）添加到场景（scene1）中。
    将场景（scene1）设置为图形视图（graphicsView）的场景。
    设置图形视图（graphicsView）的视口更新模式为全视口更新（FullViewportUpdate）*/
    ui->pushButton->setStyleSheet("color: green");  //字体颜色 
    ui->label->setText("Check Gps !");
    // 对象元素-->场景
    scene1->addItem(item);  //此时可以加载图片了
    // 场景-->视图
    ui->graphicsView->setScene(scene1);
    // 设置QGraphicsView更新
    ui->graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);//当场景的任何可见部分发生变化或重新曝光时，视图将更新整个视口
    ui->pushButton->setEnabled(false);
    ui->pushButton_2->setEnabled(true);
}

void MainWindow::pushButton_2clicked(){
    //check gps,  system ros2 run gps gps_node
    ui->pushButton_2->setStyleSheet("color: green");
    ui->label->setText("Select Destination!");
    scene2->addItem(mm_item);//将mm_item添加到场景scene2中。
    ui->graphicsView_2->setScene(scene2);//将图形项（item）添加到场景（scene1）中。
    /**视图将确定需要重绘的最小视区区域，通过避免重绘未更改的区域来最小化绘制时间。 
    /* 这是视图的默认模式。虽然这种方法总体上提供了最好的性能，但如果场景中有许多小的可见变化，视图最终可能会花费更多的时间来寻找最小化区域的方法而不是绘制。*/
    ui->graphicsView_2->setViewportUpdateMode(QGraphicsView:: MinimalViewportUpdate);//将graphicsView_2的视口更新模式设置为最小视口更新。
    //Left figure  car Pos
    QTimer *timer = new QTimer(this);
    timer->start(1000/30);
    /*timer->start(1000/30);：以1000/30毫秒（大约33毫秒）的超时启动定时器。
    connect(timer, SIGNAL(timeout()), this, SLOT(carPosUpdate()));：将定时器的超时信号连接到carPosUpdate槽。*/
    connect(timer, SIGNAL(timeout()), this, SLOT(carPosUpdate()));
    sleep(1);
    //Right figure car Pos
    QPointF carPointScene =  QPointF(0, 0);
    
    mm_item->updatePoint(carPointScene, 10);    
    ui->pushButton_2->setEnabled(false);
    ui->pushButton_3->setEnabled(true);
    //禁用鼠标穿透
    ui->graphicsView->setAttribute(Qt::WA_TransparentForMouseEvents,false); //可以进行鼠标点击事件 
}

void MainWindow::pushButton_3clicked(){
    //gen Path, select  destination, system ros2 run global global_node,
    ui->pushButton_3->setStyleSheet("color: green");
    ui->label->setText("Check Cam!");
    //pain global path
    QPolygonF points;
    Eigen::Matrix<double, 1, 2> imagePosition;
    double item_path_y, item_path_x;
    for (int i = 0; i < globalPath.cols(); i++){
        tool::realPosition2imagePosition( globalPath(0, i),  globalPath(1, i), imagePosition);
        item_path_x = imagePosition(0);
        item_path_y = imagePosition(1);
        QPointF point(item_path_x, item_path_y);
        points.push_back(point);
    }
    item->updatePoints(points);//在图片上绘制全局路径     
    ui->pushButton_3->setEnabled(false);
    ui->pushButton_4->setEnabled(true);
    ui->graphicsView->setAttribute(Qt::WA_TransparentForMouseEvents,true); //启用鼠标穿透
}

void MainWindow::pushButton_4clicked(){
    // system ros2 run cam cam_node, 
    ui->pushButton_4->setStyleSheet("color: green");
    ui->label->setText("Check Lidar!");
    //objs show right
    // QTimer *timer = new QTimer(this);
    // timer->start(1000/30);
    // connect(timer, SIGNAL(timeout()), this, SLOT(camObjPosUpdate()));
    // connect(timer, SIGNAL(timeout()), this, SLOT(roadUpdate()));
    // sleep(1);
    ui->pushButton_4->setEnabled(false);
    ui->pushButton_5->setEnabled(true);
}

void MainWindow::pushButton_5clicked(){
    // system ros2 run lidar lidar_node
    ui->pushButton_5->setStyleSheet("color: green");
    ui->label->setText("Check Radar!");
    // Subscribe Lidar info
    // QTimer *timer = new QTimer(this);
    // timer->start(1000/30);
    // connect(timer, SIGNAL(timeout()), this, SLOT(lidarObjPosUpdate()));
    // connect(timer, SIGNAL(timeout()), this, SLOT(pointCloudUpdate()));
    QTimer *timer = new QTimer(this);
    timer->start(10);
    connect(timer, SIGNAL(timeout()), this, SLOT(lidarObjPosUpdate()));
    connect(timer, SIGNAL(timeout()), this, SLOT(roadUpdate()));
    sleep(1);
    ui->pushButton_5->setEnabled(false);
    ui->pushButton_6->setEnabled(true);
}

void MainWindow::pushButton_6clicked(){
    //system ros2 run radar radar_node
    ui->pushButton_6->setStyleSheet("color: green");
    ui->label->setText("Set Speed, Gear And Run!");

    //
    ui->pushButton_6->setEnabled(false);
    ui->pushButton_7->setEnabled(true);
}

void MainWindow::pushButton_7clicked(){
    //system ros2 run local local_node
    ui->label->setText("Is Running!");
    ui->pushButton_7->setStyleSheet("color: gray");
    ui->pushButton_7->setEnabled(false);
    ui->pushButton_8->setStyleSheet("color: black");
    ui->pushButton_8->setEnabled(true);    
    int set_speed =ui->spinBox->value();
    int index = ui->comboBox->currentIndex();  //获得档位索引P = 0, N = 1, D = 2, R = 3;
    int flag = 1;
    QVector<qint64> vref_gears_flag;
    vref_gears_flag.push_back(set_speed);
    vref_gears_flag.push_back(index); 
    vref_gears_flag.push_back(flag);
    emit emitVelocityAndGearsData(QVector<qint64>(vref_gears_flag));
    QTimer *timer = new QTimer(this);
    timer->start(50);
    connect(timer, SIGNAL(timeout()), this, SLOT(localTrajUpdate()));
    //connect(timer, SIGNAL(timeout()), this, SLOT(localTrajUpdate_1()));
    sleep(1);
}

void MainWindow::pushButton_8clicked(){
    ui->label->setText("Stopping!!!");
    ui->pushButton_8->setStyleSheet("color: gray");
    ui->pushButton_8->setEnabled(false);
    ui->pushButton_7->setStyleSheet("color: black");
    ui->pushButton_6->setStyleSheet("color: black");
    ui->pushButton_5->setStyleSheet("color: black");
    ui->pushButton_4->setStyleSheet("color: black");
    ui->pushButton_3->setStyleSheet("color: black");
    ui->pushButton_2->setStyleSheet("color: black");
    ui->pushButton->setStyleSheet("color: black");
    ui->pushButton->setEnabled(true);
    // Reset Gear = P; set_speed = 0; SW = 0; break = 1;
    int set_speed = 0;
    int index = 1;  //获得档位索引P = 0, N = 1, D = 2, R = 3;
    int flag = 0;
    QVector<qint64> vref_gears_flag;
    vref_gears_flag.push_back(set_speed);
    vref_gears_flag.push_back(index); 
    vref_gears_flag.push_back(flag);
    emit emitVelocityAndGearsData(QVector<qint64>(vref_gears_flag));
}

void MainWindow::label_6update(){
    char buffer[256];
    sprintf(buffer, " %f", auto_SW);
    ui->label_6->setText(buffer);
}

void MainWindow::recvGpsCallback(qreal x, qreal y, QString dir, QString v){
    ui->label_8->clear();
    ui->label_10->clear(); 
    ui->label_8->setText(v);
    ui->label_10->setText(dir);
    gpsX = x; 
    gpsY = y;  
    heading = dir.toDouble();
    speed =v.toDouble();
    if(std::abs(gpsX) < 1 && std::abs(gpsY) < 1){
        std::cout << "gps get error!!!" << std::endl;
        QMainWindow::close();
    }
    // car.resize(5, 1);
    // int resGet = tool:: getCarPosition(gpsX,  gpsY,  heading,  speed, car);
}

void MainWindow::recvGlobalCallback(QVector<qreal> pathPoint){
    int cols = pathPoint.back();
    pathPoint.pop_back();
    globalPath.resize(7, cols);
    Eigen::Map<Eigen::MatrixXd> globalPathReshaped(pathPoint.data(), 7, cols);
    globalPath= globalPathReshaped;
    // for (size_t i = 0; i < globalPath.cols(); i++)
    //     {
    //         globalPath(0,i) = globalPath(0,i)-(2.647/2)*cos(globalPath(3,i));
    //         globalPath(1,i) = globalPath(1,i)-(2.647/2)*sin(globalPath(3,i));
    //     }
}

void MainWindow::recvCamCallback(QVector<qreal> objs){
    int cols = objs.size() / 2;
    cam_objs.resize(2, cols);
    // for(int i=0;i < objs.size();i++){
    //     std::cout << "data__:" <<objs[i] << std::endl;
    // }
    Eigen::Map<Eigen::MatrixXd>_objs(objs.data(), 2, cols);
    cam_objs = _objs;
     
}

void MainWindow::recvLidarCallback(QVector<qreal> obseslidar){
    lidar_objs.resize(5,obseslidar.size()/5);
    Eigen::Map<Eigen::MatrixXd>_lidar(obseslidar.data(),5,obseslidar.size()/5);
    lidar_objs = _lidar;
}


void MainWindow::recvLocalCallback(QVector<qreal> pathPoint){
    int cols = pathPoint.size() / 9;
    Eigen::Map<Eigen::MatrixXd> localPathReshaped(pathPoint.data(), 9, cols);
    localPath = localPathReshaped;
    //plot path line variable
}

void MainWindow::recvLocalCallback_1(QVector<qreal> pathPoint){
    int cols = pathPoint.size() / 9;
    Eigen::Map<Eigen::MatrixXd> localPathReshaped(pathPoint.data(), 9, cols);
    localPath1 = localPathReshaped;
    //plot path line variable
}

void MainWindow::recstopline(qreal stoplines){
    stopline =stoplines;
}



void MainWindow::lineEditState(){
    if (ui->checkBox->isChecked()){
		ui->lineEdit->setEnabled(true);
	}
	else{
		ui->lineEdit->setEnabled(false);
	}
}

void MainWindow::carPosUpdate(){
    car.resize(5, 1);
    if(std::abs(gpsX) < 1 && std::abs(gpsY) < 1){
        std::cout << "gps data can't receive !!!" << std::endl;
       // QMainWindow::close();
    }else{
        int resGet = tool:: getCarPosition(gpsX,  gpsY,  heading,  speed, car);

        if (resGet!= 0){
            std::cout << "get car pos failed!!" << std::endl;
        }else{
            // car(0) = car(0)-car(4)/2*cos(car(3));//hou zhou
            // car(1) = car(1)-car(4)/2*sin(car(3));
            tool::realPosition2imagePosition(car(0), car(1), carImagePosition);
            x = carImagePosition(0);
            y = carImagePosition(1);
            car_pos = QPointF(x, y);
            item->updatecPoint(car_pos);  //车子的位置 
        }
    }
}

void MainWindow::camObjPosUpdate(){
    QPolygonF points;
    double x_k = 10, y_k = -10;
    for(unsigned int i = 0; i < cam_objs.cols(); i++){
        points.push_back(QPoint(x_k*cam_objs(0, i), y_k*(cam_objs(1, i))));    
    }
    mm_item->updateCamObjsPoints(points, 10);
}

void MainWindow::lidarObjPosUpdate(){
    QPolygonF points;
    double x_k = 10, y_k =10;
    for (size_t i = 0; i < lidar_objs.cols(); i++){
        //std::cout<<"x_min:"<<lidar_objs(0,i)<<" y_min:"<<lidar_objs(1,i)<<" x_max:"<<lidar_objs(2,i)<<" y_max:"<<lidar_objs(3,i)<<std::endl;
        double angle =-(90-car(3))* M_PI / 180.0;
        double xuanzhuan_x_1 = -lidar_objs(1,i);
        double xuanzhuan_y_1 = -lidar_objs(0,i);
        double xuanzhuan_x_2 = -lidar_objs(3,i);
        double xuanzhuan_y_2 = -lidar_objs(2,i);
        // double xuanzhuan_x_1 = (lidar_objs(0,i))*cos(angle) - (lidar_objs(1,i))*sin(angle);
        // double xuanzhuan_y_1 = (lidar_objs(0,i))*sin(angle) + (lidar_objs(1,i))*cos(angle);
        // double xuanzhuan_x_2 = (lidar_objs(2,i))*cos(angle) - (lidar_objs(3,i))*sin(angle);
        // double xuanzhuan_y_2 = (lidar_objs(2,i))*sin(angle) + (lidar_objs(3,i))*cos(angle);
        //std::cout<<"x_min_zhaun:"<<xuanzhuan_x_1<<" y_min_zhuan:"<<xuanzhuan_y_1<<" x_max_zhuan:"<<xuanzhuan_x_2<<" y_max_zhuan:"<<xuanzhuan_y_2<<std::endl;
        points.push_back(QPoint(x_k* xuanzhuan_x_1,y_k* xuanzhuan_y_1));
        points.push_back(QPoint(x_k* xuanzhuan_x_2,y_k* xuanzhuan_y_2));
          
    }
    mm_item->updateLidarObjsPoints(points,2);
}

void MainWindow::roadUpdate(){  
    int minIndex ; 
    tool::findClosestPoint(car, globalPath, minIndex);
    int all_points_num;
    all_points_num = (minIndex + 120) > globalPath.cols()? globalPath.cols() : (minIndex + 120);
    QPolygonF points;
    Eigen::Matrix<double, 1, 3> pathPosTheta;
    Eigen::Matrix<double, 1, 3> pathPosThetaL;
    Eigen::Matrix<double, 1, 3> pathPosThetaR;
    realCar.resize(3, 1);
    Eigen::VectorXd pathTransPointL;
    Eigen::VectorXd pathTransPointR;
    Eigen::VectorXd pathTransPointM;
    double distance=0; size_t index=1e10;
    for (size_t i = minIndex+1; i < all_points_num; ++i){
        distance += std::sqrt(std::pow(globalPath(0,i)-globalPath(0,i-1),2)+std::pow(globalPath(1,i)-globalPath(1,i-1),2));
        if (distance>=stopline){
            index = i;
            //std::cout<<"pointstopline: "<<pointstopline.size()<<std::endl;
            break;
        }
    }
    QPolygonF pointstopline;
    double offset_r = -3.3, offset_l = 3.3;
    double x_k =10, y_k = -10;
    Eigen::Vector2d cartesianXY_r,cartesianXY_l;
    realCar << car(0), car(1), car(3);
    for (int i = minIndex + 1; i < all_points_num; i++){
        pathPosTheta << globalPath(0, i), globalPath(1, i), globalPath(3, i);
        
        tool::coordinateTransform(realCar, pathPosTheta, pathTransPointM);
        QPointF pointM(x_k*pathTransPointM(0), y_k*pathTransPointM(1));
        points.push_back(pointM);

        tool::frenet2Cartesian(globalPath, globalPath(6, i), offset_r, i, cartesianXY_r);//右边
        pathPosThetaR << cartesianXY_r(0), cartesianXY_r(1), globalPath(3, i);

        tool::coordinateTransform(realCar, pathPosThetaR, pathTransPointR);
        QPointF pointR(x_k*(pathTransPointR(0)), y_k*(pathTransPointR(1)));
        points.push_back(pointR);

        tool::frenet2Cartesian(globalPath, globalPath(6, i), offset_l, i, cartesianXY_l);//左边 
        pathPosThetaL << cartesianXY_l(0), cartesianXY_l(1), globalPath(3, i);
        tool::coordinateTransform(realCar, pathPosThetaL, pathTransPointL);

        QPointF pointL(x_k*(pathTransPointL(0)), y_k*(pathTransPointL(1)));
        points.push_back(pointL);
        // if(i==index){
        //     pointstopline.push_back(pointL);
        //     pointstopline.push_back(pointR);
        // }
    }
    mm_item->updateRoadPoints(points, 2);
    mm_item->updatestopline(pointstopline,2);
    

}

void MainWindow::localTrajUpdate(){  
    QPolygonF points; 
    realCar.resize(3, 1);
    realCar << car(0), car(1), car(3);
    double  x_k = 10, y_k = -10; 
    Eigen::Matrix<double, 1, 3> pathPosTheta;
    Eigen::VectorXd pathTransPointLocal;

    for (unsigned int i = 0; i < localPath.cols(); i++){
        pathPosTheta << localPath(0, i), localPath(1, i), localPath(3, i);
        tool::coordinateTransform(realCar, pathPosTheta, pathTransPointLocal);
        QPointF relatPoints(x_k*pathTransPointLocal(0), y_k*pathTransPointLocal(1));
        points.push_back(relatPoints);
    }
    mm_item->updateLocalTrajPoints(points, 4);
}

void MainWindow::localTrajUpdate_1(){  
    QPolygonF points; 
    realCar.resize(3, 1);
    realCar << car(0), car(1), car(3);
    double  x_k = 10, y_k = -10; 
    Eigen::Matrix<double, 1, 3> pathPosTheta;
    Eigen::VectorXd pathTransPointLocal;
    for (unsigned int i = 0; i < localPath1.cols(); i++){
        pathPosTheta << localPath1(0, i), localPath1(1, i), localPath1(3, i);
        tool::coordinateTransform(realCar, pathPosTheta, pathTransPointLocal);
        // std::cout << "ii:" << i << std::endl;
        QPointF relatPoints(x_k*pathTransPointLocal(0), y_k*pathTransPointLocal(1));
        // QPointF relatPoints((localPath(0, i) - realCar(0)) * x_k, (localPath(1, i) - realCar(1)) * y_k);
        points.push_back(relatPoints);
    }
    // std::cout << "conlocalPath.cols:" << localPath1.cols() << std::endl;
    mm_item->updateLocalTrajPoints_1(points, 2);
}

