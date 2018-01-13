#include "mainwindow.h"
#include <QApplication>
#include <QSplashScreen>
#include <QTimer>
#include <bpa/Parameters.h>
#include <QThread>

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

//    QSplashScreen *splash = new QSplashScreen;
//    splash->setPixmap(QPixmap("/home/somani/catkin_ac2_gui/src/gui/system_gui/src/logo.png"));
//    splash->setPixmap(QPixmap(":/images/logo.png"));
//    splash->show();
//    QTimer::singleShot(2500, splash, SLOT(close())); // Timer
//    QTimer::singleShot(2500,MainWindow::instance(),SLOT(showFullScreen()));

    MainWindow::instance()->showFullScreen();
//    splash.finish(MainWindow::instance());

    std::cout << "Starting System GUI" << std::endl;
//    while(true){
//        app.processEvents();
////        QApplication::processEvents();

//        usleep(10000);
//    }
    return app.exec();

    return 0;
}
