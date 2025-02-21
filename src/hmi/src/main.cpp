#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[]){
    // kill ps
    //system("ps -a | grep gnss_node | awk '{print $1}' | xargs kill -9");
    system("ps -a | grep global_node | awk '{print $1}' | xargs kill -9");    
    system("ps -a | grep local_node3 | awk '{print $1}' | xargs kill -9"); 
    system("ps -a | grep image_pub | awk '{print $1}' | xargs kill -9");  
    system("ps -a | grep detect_node | awk '{print $1}' | xargs kill -9"); 
    system("ps -a | grep Lidar_Processing_node | awk '{print $1}' | xargs kill -9");  
    system("ps -a | grep demo_node | awk '{print $1}' | xargs kill -9");  
    //can_node
    system("ps -a | grep can_node | awk '{print $1}' | xargs kill -9");  
    system("ps -a | grep points_and_lines_publisher | awk '{print $1}' | xargs kill -9");   
    //调用了QApplication类的setSetuidAllowed函数，该函数用于设置是否允许以setuid的方式运行应用程序 
    QApplication::setSetuidAllowed(true);
    //创建了一个QApplication对象a，并将命令行参数argc和argv传递给它
    QApplication a(argc, argv);
    //调用了qRegisterMetaType函数，用于注册自定义类型QVector<qreal>，以便在信号与槽之间传递该类型的数据。
    qRegisterMetaType<QVector<qreal>>("QVector<qreal>");
    MainWindow w;
    //创建了一个MainWindow对象w，并调用其show函数显示窗口。最后调用了a.exec()来启动应用程序的事件循环。
    w.show();
    return a.exec();
}
