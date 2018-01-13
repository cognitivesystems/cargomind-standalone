#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent, Qt::WindowFlags f) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    std::cout << "MainWindow constructor" << std::endl;

    MainWindow::singleton_ = this;

    ui->setupUi(this);

    this->timer_id_ = startTimer(1);

    this->ui->stackedWidget->setCurrentIndex(0); // always select the first page of the stacked widget in the beginning

}

MainWindow::~MainWindow()
{
    delete ui;
}

MainWindow* MainWindow::instance()
{
    if (NULL == MainWindow::singleton_)
    {
        new MainWindow();
    }
    return MainWindow::singleton_;
}

BinPackingGUI* MainWindow::binPackingGUIInstance()
{
    if (NULL == MainWindow::singleton_)
    {
        (new MainWindow())->ui->page_2;
    }
    return MainWindow::singleton_->ui->page_2;
}

void MainWindow::switchToBinPackingGui(){
    MainWindow::singleton_->ui->stackedWidget->setCurrentIndex(0);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    std::cout << "Closing main gui" << std::endl;
    close();

    killTimer(this->timer_id_);
}

void MainWindow::timerEvent(QTimerEvent* timerEvent)
{    
    //this->viewer_->viewAll();
}
