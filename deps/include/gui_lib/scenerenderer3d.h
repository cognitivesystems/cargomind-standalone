#ifndef SCENERENDERER3D_H
#define SCENERENDERER3D_H

#include <QWidget>
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
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoFile.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoText3.h>
#include <Inventor/nodes/SoFont.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/actions/SoBoxHighlightRenderAction.h>
#include <Inventor/SbBasic.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/SbLine.h>
#include <Inventor/nodes/SoVertexProperty.h>
#include <Inventor/nodes/SoDrawStyle.h>

#include <QEvent>
#include <QDebug>
#include <QTouchEvent>
#include <QUrl>
#include <QMatrix4x4>
#include <QQuaternion>
#include <iostream>
#include <sstream>
#include <math.h>
//#include <web_tools/BlobStoreHelper.h>

typedef std::vector<QMatrix4x4> Transforms;

struct BodyData{
    std::string type;
    std::vector<std::string > labels;
    QVector3D bbox;
    std::string material;
    std::string uuid;

};

struct ObjectBody {
    QString body_name;
    QString mesh_url;
    QMatrix4x4 tr;
    BodyData actor;
};

struct ObjectModel {
    QString object_name;
    std::vector<ObjectBody > bodies;
    bool has_parent;
    QString parent_body_name;
    QMatrix4x4 parent_transform;
};

struct MotionTrajetoryData{
    QString trajectory_name;
    std::vector<QVector3D > points;
};


typedef std::vector< ObjectModel > ObjectModels;

class SceneRenderer3D : public QWidget
{
    Q_OBJECT

public:
    SceneRenderer3D(QWidget *parent = 0);

    void addObjectBody(const ObjectBody& info);

    void addObjectModel(const ObjectModel& info);

    void removeObjectModel(const ObjectModel& model);

    void updateObjectModel(const ObjectModel &info);
    void updateObjectModelPlanning(const ObjectModel &info);

    QQuaternion getQuatFromVec(const QVector3D& vec1, const QVector3D& vec2, double& angle, QVector3D& dir);

    void addMotionTrajectory(const MotionTrajetoryData& info);
    void removeMotionTrajectory(const QString& trajectory_name);

    QString fetchFileFromUrl(const QString& url);

    SbMatrix getSbMatrix(const QMatrix4x4 q_mat);
    QMatrix4x4 getQMatrix(const SbMatrix sb_mat);

    ObjectModels getModels();

    bool getSceneUpdateStatus();
    void setSceneUpdateStatus(bool status);

    bool getAddActorStatus();
    void setAddActorStatus(bool status);

    bool getRemoveActorStatus();
    void setRemoveActorStatus(bool status);

    bool getUpdateActorStatus();
    void setUpdateActorStatus(bool status);

    void deleteScene();

    void clearScene();

    SoSelection * selection;
    SoSeparator* scene_;

    static void made_selection( void * userdata, SoPath * path );

    void setSelectedObject(const bool selected, const std::string name);
    std::pair<bool, std::string> getSelectedObject();

public slots:

    void slotAddObjectModel(const ObjectModel& model);

    void slotRemoveObjectModel(const ObjectModel& model);

    void slotUpdateObjectModel(const ObjectModel& model);

    void slotUpdateObjectModels(const std::vector<ObjectModel>& models);
    void slotUpdateObjectModelsPlanning(const std::vector<ObjectModel>& models);

    void slotAddMotionTrajectoryModel(const MotionTrajetoryData& data);
    void slotRemoveMotionTrajectorySegment(const QString segment_name);

    void saveScene();
//    bool event(QEvent *event);
//    void wheelEvent(QWheelEvent *event);

protected:

    void addViewpoints();

    void switchCameraView(const int id);

    std::vector<SbVec3f > camera_position_views_;
    std::vector<SbRotation  > camera_orientation_views_;
    std::vector<float > camera_aspect_ratios_;
    std::vector<float > camera_far_distance_;
    std::vector<float > camera_focal_distance_;
    std::vector<float > camera_near_distance_;

    std::string instance_name_;

    SoQtExaminerViewer* viewer_;

    ObjectModels current_scene_models_;

    QMap<QString, SoTransform* > body_transform_map_;

    bool scene_updating_;
    QMutex scene_updating_mutex_;

    bool adding_actor_;
    QMutex adding_actor_mutex_;

    bool removing_actor_;
    QMutex removing_actor_mutex_;

    bool updating_actor_;
    QMutex updating_actor_mutex_;

//    qreal totalScaleFactor;

    //3D visualization
    bool transparency_wooden;
    QMutex mutex_wooden_;
    bool transparency_styrofoam;
    QMutex mutex_styrofoam_;
    bool transparency_carton;
    QMutex mutex_carton_;
    bool transparency_plastic;
    QMutex mutex_plastic_;
    bool transparency_all;

    bool view_fragile;
    QMutex mutex_fragile_;
    bool view_dangerous;
    QMutex mutex_dangerous_;
    bool view_others;
    QMutex mutex_others_;

    bool view_support_box;    
    std::string selected_actor;
    std::vector<std::string> supporting_boxes;

    bool view_texture;
    QMutex mutex_texture_;

    bool view_PalletBB;
    QMutex mutex_PalletBB_;
    bool view_BppCOM;
    QMutex mutex_BppCOM_;

    float com_x;
    float com_y;
    float com_z;

    bool object_selected_;
    std::string selected_object_name_;
    QMutex mutex_selection_;

};

#endif // SCENERENDERER3D_H
