#include "mItem.h"
#include "QDebug"

mItem::mItem(){
  //首先调用了一些成员函数来设置该对象的一些属性，比如启用鼠标悬停事件、接受所有鼠标按钮的事件、接受拖放操作，并且设置该对象接受输入法输入。然后调用了moveBy函数来移动该对象的位置
  setAcceptHoverEvents(true);//使其在鼠标未点击也能响应悬停事件
  setAcceptedMouseButtons(Qt::AllButtons);
  setAcceptDrops(true);
  setFlag(ItemAcceptsInputMethod, true);
  moveBy(0, 0);
  // m_images.load("/home/liuwei/Desktop/ex_Qt/ex6_ws_Qt/map.jpg");
  m_images.load("src/hmi/high_precision_map3.jpg");//背景 
}
/*在该函数中，使用QRectF类创建了一个矩形对象，其左上角坐标为(0, 0)，宽度为900，高度为780。然后将该矩形对象作为返回值。
该函数的作用是返回一个用于描述该类对象边界的矩形区域，通常用于在绘制图形时确定对象的位置和大小。在本例中，
该函数返回的矩形区域表示了mItem对象的边界范围，宽度为900，高度为780，左上角坐标为(0, 0)。*/
QRectF mItem::boundingRect() const{
  return QRectF(0, 0, 900, 780);
}

void mItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                  QWidget *widget){
  painter->setRenderHint(QPainter::Antialiasing, true); // 设置反锯齿 反走样
  drawImage(painter);
  drawPoints(painter);
  drawcPoint(painter);
  drawdPoint(painter);
}

void mItem::updatePoints(QPolygonF points){
  m_points = points;
  update();
}

void mItem::updatecPoint(QPointF point){
  m_cpoint = point;
  update();
}

void mItem::updatedPoint(QPointF point){
  m_dpoint = point;
  update();
}

void mItem::updateImage(QImage img){
  m_images = img;
  update();
}

void mItem::drawImage(QPainter *painter){
  painter->drawImage(0, 0, m_images);
}


void mItem::drawPoints(QPainter *painter){
  painter->setPen(QPen(QColor(255, 0, 0), 2));
  painter->drawPoints(m_points);
}

void mItem::drawcPoint(QPainter *painter){
  painter->setPen(QPen(QColor(0, 0, 255), 10));
  painter->drawPoint(m_cpoint);
}

void mItem::drawdPoint(QPainter *painter){
  painter->setPen(QPen(QColor(0, 255, 0), 10));
  painter->drawPoint(m_dpoint);
}

void mItem::wheelEvent(QGraphicsSceneWheelEvent *event){
  double beforeScaleValue = m_scaleValue;
  if (event->delta() > 0)  {
    m_scaleValue *= 1.1; // 每次放大10%
  }
  else  {
    m_scaleValue *= 0.9; // 每次缩小10%
  }
  setScale(m_scaleValue);
  // 使放大缩小的效果看起来像是以鼠标中心点进行放大缩小
  if (event->delta() > 0){
    moveBy(-event->pos().x() * beforeScaleValue * 0.1,
           -event->pos().y() * beforeScaleValue * 0.1);
  }
  else{
    moveBy(event->pos().x() * beforeScaleValue * 0.1,
           event->pos().y() * beforeScaleValue * 0.1);
  }
  update();
}

void mItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
  if (m_isMousePress)  {
    QPointF point = (event->pos() - m_pressedPose) * m_scaleValue;
    moveBy(point.x(), point.y());
  }
  update();
}

void mItem::mousePressEvent(QGraphicsSceneMouseEvent *event){
  if (event->button() == Qt::LeftButton)  {
    m_pressedPose = event->pos();
    // std::cout<<m_pressedPose.x()<<" "<<m_pressedPose.y()<<std::endl;
    m_isMousePress = true;
  }
  update();
}

void mItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
  if (event->button() == Qt::LeftButton)  {
    m_isMousePress = false;
  }
  update();
}
// void mItem::getPos(Eigen::Matrix<double, 1, 2>& endPoint){
void mItem::getPos(QPointF& selecEndPoint){
  selecEndPoint = m_selectPose;
}

void mItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event){
  if (event->button() & Qt::LeftButton){
    m_selectPose = event->pos();//返回事件产生时的鼠标指针相对于接收事件的控件的位置。
  }
  emit getDestination(m_selectPose);
  updatedPoint(m_selectPose);
}
