#ifndef MMITEM_H
#define MMITEM_H

#include <QCursor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include <QObject>
#include <QPainter>
#include<iostream>

class mmItem: public QObject, public QGraphicsItem{
    Q_OBJECT
public:
    mmItem();
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
              QWidget *widget = nullptr) override;
    void updateImage(QImage img);
    // void updatePoints(QPolygonF points);
    // void updatePoints(QPolygonF points, int size_input);
    void updateCamObjsPoints(QPolygonF points, int points_size_input);
    void updateLidarObjsPoints(QPolygonF points, int points_size_input);
    void updatePointCloudPoints(QPolygonF points, int points_size_input);
    void updateRoadPoints(QPolygonF points, int points_size_input);
    void updateLocalTrajPoints(QPolygonF points, int points_size_input);
    void updateLocalTrajPoints_1(QPolygonF points, int points_size_input) ;
    // void updatePoint(QPointF point);
    void updatePoint(QPointF point, int point_size_input);
    void updateCircle(QPointF point);
    // void updateCircles(QPolygonF points);

    void updatestopline(QPolygonF points, int points_size_input);

    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;

private:
    QImage m_images;
    QPolygonF road_m_points;
    QPolygonF cam_objs_m_points;
    QPolygonF lidar_objs_m_points;
    QPolygonF pointCloud_m_points;
    QPolygonF local_m_points;
    QPolygonF local_m_points_1;
    QPolygonF stoplines_points;
    QPointF m_point;
    bool m_isMousePress = false;
    QPointF m_pressedPose;
    QPointF m_selectPose;
    QPointF m_center;
    QPolygonF m_centers;
    int road_points_size = 2; //points size
    int cam_objs_points_size = 2;
    int lidar_objs_points_size = 2;
    int pointCloud_points_size = 0.1;
    int local_points_size = 2;
    int local_points_size_1 = 2;
    int point_size = 2; //point size
    int rectLine_size = 2;
    int xAxisLine_size = 2;
    int yAxisLine_size = 2;
    
private:
    void drawImage(QPainter *painter);
    // void drawPoints(QPainter *painter);
    void drawCamObjsPoints(QPainter *painter);
    void drawLidarObjsPoints(QPainter *painter);
    void drawPointCloudPoints(QPainter *painter);
    void drawRoadPoints(QPainter *painter);
    void drawLocalTrajsPoints(QPainter *painter);
    void drawLocalTrajsPoints_1(QPainter *painter);
    void drawPoint(QPainter *painter);
    void drawCircle(QPainter *painter);   
    // void drawCircles(QPainter *painter);
    void drawRecInit(QPainter *painter);
    void drawX_axisInit(QPainter *painter);
    void drawY_axisInit(QPainter *painter);

    void drawstopline(QPainter *painter);



 
};

#endif // MMITEM_H