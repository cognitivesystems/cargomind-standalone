#include "listitem.h"
ListItem::ListItem(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ListItem)
{
    ui->setupUi(this);
//    //configure list
//    ui->listWidget->setSelectionMode(QAbstractItemView::NoSelection);
//    ui->listWidget->setIconSize(QSize(100,100));
//    ui->listWidget->setMovement(QListWidget::Static);
//    ui->listWidget->setViewMode(QListWidget::IconMode);//deactivate this to look more like a list
//    ui->listWidget->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
//    ui->listWidget->setFrameStyle(QFrame::NoFrame);

//    //set up scrolling of list
//    this->charm = new FlickCharm();
//    this->charm->activateOn(ui->listWidget);

//    isDone = false;
}

ListItem::~ListItem()
{
    delete ui;
}
