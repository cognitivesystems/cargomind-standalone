#ifndef LISTITEM_H
#define LISTITEM_H

#include <QWidget>
#include <QPainter>
#include <QSlider>
#include <QPaintEvent>
#include <QStyle>
#include <QStyleOptionSlider>
#include <QObject>
#include <QEvent>
#include <iostream>
#include <QListWidgetItem>

#include <ui_listitem.h>

namespace Ui {
class ListItem;
}

//class ListItem : public QListWidgetItem, public QWidget
class ListItem : public QWidget
{

    Q_OBJECT

public:

//    MySlider():QSlider(Qt::Horizontal)
//    {
//    };

//    ListItem(std::string string):
//        ui(new Ui::ListItem)
//    {
//        ui->setupUi(this);
//        ui->boxID_value->setText(string.c_str());
//    };
//    explicit ListItem(QWidget *parent);


    explicit ListItem(QWidget *parent = 0);
    virtual ~ListItem();

    Ui::ListItem *ui;

};

#endif // LISTITEM_H
