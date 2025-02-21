#include "mmItem.h"

mmItem::mmItem(){
    setAcceptHoverEvents(true);
    setAcceptedMouseButtons(Qt::AllButtons);
    setAcceptDrops(true);
    setFlag(ItemAcceptsInputMethod, true);
    moveBy(0, 0);
}

QRectF mmItem::boundingRect() const {
  return QRectF(-400, -400, 800, 800);
}

void mmItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                                            QWidget *widget) {
  painter->setRenderHint(QPainter::Antialiasing, true);  //设置反锯齿 反走样
  drawImage(painter);
  drawRoadPoints(painter);
  drawLocalTrajsPoints(painter);
  drawLocalTrajsPoints_1(painter);
  drawPoint(painter);
  drawCircle(painter);
  drawRecInit(painter);
  drawX_axisInit(painter);
  drawY_axisInit(painter);
  drawLidarObjsPoints(painter);
  drawstopline(painter);

}

void mmItem::drawRecInit(QPainter *painter){
  painter->setPen(QPen(QColor(255, 0, 0), rectLine_size));
  QRectF rect(-200, -200, 400, 400); 
  painter->drawRect(rect);   //绘制矩形  
}

void mmItem::drawX_axisInit(QPainter *painter){
  painter->setPen(QPen(QColor(0, 255, 0), xAxisLine_size));
  QLine line_x(0, 0, 40, 0);
  painter->drawLine(line_x);  
}

void mmItem::drawY_axisInit(QPainter *painter){
  painter->setPen(QPen(QColor(0, 0, 255), yAxisLine_size));
  QLine line_y(0, 0, 0, -40);
  painter->drawLine(line_y);  
}

void mmItem::updateCamObjsPoints(QPolygonF points, int points_size_input) {
  cam_objs_points_size = points_size_input;
  cam_objs_m_points = points;
  //update();
}
void mmItem::updateLidarObjsPoints(QPolygonF points, int points_size_input) {
  lidar_objs_points_size = points_size_input;
  lidar_objs_m_points = points;
  //update();
}
void mmItem::updatePointCloudPoints(QPolygonF points, int points_size_input) {
  pointCloud_points_size = points_size_input;
  pointCloud_m_points = points;
 // update();
}
void mmItem::updateRoadPoints(QPolygonF points, int points_size_input) {
  road_points_size = points_size_input;
  road_m_points = points;
  update();
}

void mmItem::updateLocalTrajPoints(QPolygonF points, int points_size_input) {
  local_points_size = points_size_input;
  local_m_points = points;
  //update();
}

void mmItem::updateLocalTrajPoints_1(QPolygonF points, int points_size_input) {
  local_points_size_1 = points_size_input;
  local_m_points_1 = points;
  //update();
}

void mmItem::updatePoint(QPointF point, int point_size_input) {
  point_size = point_size_input;
  m_point = point;
//update();
}

void mmItem::updateImage(QImage img) {
  m_images = img;
  update();
}

void mmItem::updatestopline(QPolygonF points, int points_size_input){
  stoplines_points = points;
  //update();
}

void mmItem::drawstopline(QPainter *painter){
  if (stoplines_points.size()==2){
    painter->drawLine(stoplines_points[0],stoplines_points[1]);
  }
}

void mmItem::drawImage(QPainter *painter) {
  painter->drawImage(0, 0, m_images);
}

void mmItem::drawRoadPoints(QPainter *painter) {
  painter->setPen(QPen(QColor(255, 0, 0), road_points_size));
  painter->drawPoints(road_m_points);
}

void mmItem::drawCamObjsPoints(QPainter *painter) {
  painter->setPen(QPen(QColor(255, 0, 0), cam_objs_points_size));
  painter->drawPoints(cam_objs_m_points);
}

void mmItem::drawLidarObjsPoints(QPainter *painter) {
  painter->setPen(Qt::black);
  for (size_t i = 0; i < lidar_objs_m_points.size()/2; i+=2)
  {
    qreal x = lidar_objs_m_points[i].x();
    qreal y = lidar_objs_m_points[i].y();
    qreal width = std::abs(lidar_objs_m_points[i+1].x() - x);
    qreal height =std::abs(lidar_objs_m_points[i+1].y() - y);
    //std::cout<<"drawx:"<<x<<" "<<"drawy:"<<y<<std::endl;
    painter->drawRect(lidar_objs_m_points[i+1].x(), lidar_objs_m_points[i+1].y(), width,height);
  }

}

void mmItem::drawPointCloudPoints(QPainter *painter) {
  painter->setPen(QPen(QColor(255, 0, 255), pointCloud_points_size));
  painter->drawPoints(pointCloud_m_points);
}

void mmItem::drawLocalTrajsPoints(QPainter *painter) {
  painter->setPen(QPen(QColor(0, 255, 0), local_points_size));
  painter->drawPoints(local_m_points);
}

void mmItem::drawLocalTrajsPoints_1(QPainter *painter) {
  painter->setPen(QPen(QColor(0, 0, 0), local_points_size_1));
  painter->drawPoints(local_m_points_1);
}

void mmItem::drawPoint(QPainter *painter) {
  painter->setPen(QPen(QColor(255, 0, 255),point_size));
  painter->drawPoint(m_point);
}

void mmItem::drawCircle(QPainter *painter) {
  painter->setPen(QPen(QColor(255, 0, 0), 5));
  painter->drawEllipse(m_center, 1, 1);//绘制椭圆 
}

void mmItem::updateCircle(QPointF point) {
  m_center = point;
  update();
}

void mmItem::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
	  m_pressedPose = event->pos();
    m_isMousePress = true;
  }
  update();
}