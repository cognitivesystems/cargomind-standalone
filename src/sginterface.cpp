/*
 * SGInterface.cpp
 *
 *  Created on: Mar 8, 2012
 *      Author: nair
 */
#include "sginterface.h"
#include "mainwindow.h"

namespace sginf
{

SGInterface::SGInterface(){

    qRegisterMetaType<Transforms >("Transforms");
    qRegisterMetaType<ObjectModel >("ObjectModel");
    qRegisterMetaType<ObjectBody >("ObjectBody");
    qRegisterMetaType<bpp_actor::ActorVec >("bpp_actor::ActorVec");
    qRegisterMetaType<bpp_actor::Actor >("bpp_actor::Actor");
    qRegisterMetaType<std::vector<QString> >("std::vector<QString>");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<float>("float");
    qRegisterMetaType<std::vector<ObjectModel> >("std::vector<ObjectModel>");
    qRegisterMetaType<std::vector<bpp_actor::Actor>>("std::vector<bpp_actor::Actor>");
    qRegisterMetaType<gui_msgs::Progress>("gui_msgs::Progress");



    QObject::connect(MainWindow::binPackingGUIInstance(), SIGNAL(requestBinPackingPlan(const std::string&)),
                     this, SLOT(fetchNewBoxPlan(const std::string&)));

    QObject::connect(MainWindow::binPackingGUIInstance(), SIGNAL(evaluateBinPackingPlan()),
                     this, SLOT(evaluateBinPlan()));

    QObject::connect(MainWindow::binPackingGUIInstance(), SIGNAL(requestMALocation()),
                     this, SLOT(simulateMALocation()));

    QObject::connect(MainWindow::binPackingGUIInstance(), SIGNAL(requestActorFromUUID(const std::string&)),
                     this, SLOT(getActorFromUUID(const std::string&)));

    QObject::connect(MainWindow::binPackingGUIInstance(), SIGNAL(requestPalletizationPlan()),
                     this, SLOT(getPalletizePlan()));

    QObject::connect(MainWindow::binPackingGUIInstance(), SIGNAL(requestRePalletization(const std::string&)),
                     this, SLOT(removeBoxReplan(const std::string&)));

    QObject::connect(MainWindow::binPackingGUIInstance(), SIGNAL(loadWorldStateSignal(const std::string &)), this, SLOT(loadWorldStateSlot(const std::string&)));

    QObject::connect(MainWindow::binPackingGUIInstance(), SIGNAL(backupCurrentWorldStateSignal(const std::string &)), this, SLOT(backupCurrentWorldStateSlot(const std::string&)));



    run_thread_=true;

}

SGInterface::~SGInterface()
{
    std::cout << "killing SGInterface" << std::endl;
    std::cout << "killed SGInterface" << std::endl;
}

void SGInterface::run()
{
    std::cout << "Stating SGInterface thread" << std::endl;
    while(run_thread_)
    {
        //        ros::spinOnce();
        QThread::msleep(10);
    }
    emit finished();
    std::cout << "SGInterface thread finished" << std::endl;
}


void SGInterface::stopThread()
{
    run_thread_=false;
}

}
