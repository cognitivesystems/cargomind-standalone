#include "mainwindow.h"
#include <QApplication>
#include <QTimer>
#include <bpa/Parameters.h>
#include <QThread>
#include <iostream>


MainWindow* MainWindow::singleton_ = NULL;
bpa::Params* bpa::Params::singleton_ = NULL;


class I : public QThread
{
public:
        static void sleep(unsigned long secs)
        {
            QThread::sleep(secs);
        }
};

int main(int argc, char **argv)
{   
    QApplication app(argc, argv);

    QObject::connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    MainWindow::instance()->showFullScreen();

    std::cout << __LINE__ << std::endl;

    return app.exec();

    return 0;
}
