#ifndef MITEM_H
#define MITEM_H

#include <QCursor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include <QObject>
#include <QPainter>
#include<iostream>
#include "Eigen/Eigen"
// #include "rclcomm.h"

class mItem : public QObject, public QGraphicsItem {
    Q_OBJECT
  public:
    mItem();
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
              QWidget *widget = nullptr) override;
    QRectF boundingRect() const override;
    void updateImage(QImage img);
    void updatePoints(QPolygonF points);
    void updatecPoint(QPointF point);
    void updatedPoint(QPointF point);

  public:
    //鼠标滚轮事件、鼠标移动事件、鼠标按下事件、鼠标释放事件和鼠标双击事件
    void wheelEvent(QGraphicsSceneWheelEvent *event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) override;
    // void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event, Eigen::Matrix<double, 1, 2>& endPoint);
    // void getPos(Eigen::Matrix<double, 1, 2>& endPoint);
    void getPos(QPointF& selecEndPoint);

  private:
    QImage m_images;
    QPolygonF m_points;
    QPointF m_dpoint, m_cpoint;
    // QPintF selecEndPoint;
    double m_scaleValue = 1;
    bool m_isMousePress = false;
    QPointF m_pressedPose;
    QPointF m_selectPose;
    // rclcomm *commNode;  //communication Node  Def
  private:
    void drawImage(QPainter *painter);
    void drawPoints(QPainter *painter);
    void drawcPoint(QPainter *painter);
    void drawdPoint(QPainter *painter);

  signals:
    void getDestination(QPointF);
};

#endif  // MITEM_H
