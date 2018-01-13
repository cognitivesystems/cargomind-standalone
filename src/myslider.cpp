#include "myslider.h"

MySlider::MySlider( QWidget *parent = 0 ){
    this->installEventFilter(this);
}

bool MySlider::eventFilter(QObject *o, QEvent *e){
    bool ret = false;

    if(e->type() == QEvent::KeyPress || e->type() == QEvent::KeyRelease )
    {
//        qDebug() << "key";
        ret = true;
    }
    else if( e->type() == QEvent::MouseButtonPress
        || e->type() == QEvent::MouseButtonRelease
        || e->type() == QEvent::MouseMove
        || e->type() == QEvent::MouseButtonDblClick
        || e->type() == QEvent::Wheel)
    {
//        qDebug() << "mouse";
        ret = true;
    }

    return ret;
}

void MySlider::paintEvent(QPaintEvent *e){
    QSlider::paintEvent(e);

    QStyle *st = style();
    QPainter p(this);

    int v = this->minimum();

    QStyleOptionSlider slider;
    slider.initFrom(this);

    int len = st->pixelMetric(QStyle::PM_SliderLength, &slider, this);

    int available = st->pixelMetric(QStyle::PM_SliderSpaceAvailable, &slider, this);

    QRect r;
    p.drawText(rect(), Qt::TextDontPrint, QString::number(9999), &r);

    while (v<this->maximum()){
        QString vs = QString::number(v)+" minutes";

        int left = QStyle::sliderPositionFromValue(minimum(), maximum(), v, available) + len;
        int left_next = QStyle::sliderPositionFromValue(minimum(), maximum(), v+tickInterval(), available);
        
        QPoint pos(left,rect().bottom());
        int right = left+r.width();

        if ((right<rect().right() && right<left_next) || left_next==0) // the OR is needed in order to show last element
            p.drawText(pos,vs);

        v += this->tickInterval();
    }

}

