#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QSharedPointer>
#include <QDir>
#include <QFileInfo>
#include <QTextStream>
#include <QMessageBox>
#include <QMutex>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodekits/SoBaseKit.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoFile.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoSearchAction.h>

#include <sstream>
#include "sginterface.h"
#include "BlobStoreHelper.h"
#include "ui_mainwindow.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    virtual ~MainWindow();
    static MainWindow* instance();
    static BinPackingGUI* binPackingGUIInstance();

public slots:
    void switchToBinPackingGui();

protected:
    MainWindow(QWidget* parent = NULL, Qt::WindowFlags f = 0);

    void closeEvent(QCloseEvent *event);

    virtual void timerEvent(QTimerEvent* timerEvent);


private:
    Ui::MainWindow *ui;

    static MainWindow* singleton_;

    sginf::SGInterface* sg_interface_;

    int timer_id_;

};

#endif // MAINWINDOW_H
