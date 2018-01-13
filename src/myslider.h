#ifndef MYSLIDER_H
#define MYSLIDER_H

#include <QWidget>
#include <QPainter>
#include <QSlider>
#include <QPaintEvent>
#include <QStyle>
#include <QStyleOptionSlider>
#include <QObject>
#include <QEvent>
#include <iostream>

class MySlider : public QSlider
{

    Q_OBJECT

public:

//    MySlider():QSlider(Qt::Horizontal)
//    {
//    };

//    MySlider():QSlider() {};
    explicit MySlider(QWidget *parent);

    void paintEvent(QPaintEvent *e);
    bool eventFilter(QObject *o, QEvent *e);

};

#endif // MYSLIDER_H
