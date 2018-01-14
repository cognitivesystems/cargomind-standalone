#ifndef BINPACKINGGUI_H
#define BINPACKINGGUI_H

#include "sginterface.h"
#include <gui_lib/scenerenderer3d.h>
#include "defs.h"
#include <thread>
#include "WorldStateGenerator.h"
#include "ui_binpackinggui.h"
#include <QStringListModel>
#include "SGJSONParser.h"
#include "BlobStoreHelper.h"
#include <bullet/Actor.h>
#include "ui_binpackinggui.h"

namespace Ui {
    class BinPackingGUI;
}

class BinPackingGUI : public SceneRenderer3D{
    Q_OBJECT

public:
    explicit BinPackingGUI(QWidget *parent = 0);
    ~BinPackingGUI();

    bool getShowDesiredPose();
    void setShowDesiredPose(bool sdp);

    bool getPlanningFinished();
    void setPlanningFinished(bool plan_flag);

    bool getPlanningRequested();
    void setPlanningRequested(bool plan_flag);

    bpp_actor::ActorVec getPlanActors();
    void setPlanActors(bpp_actor::ActorVec actors);

    void setSelectedActor(bpp_actor::Actor actor);

    EvalBppResults getBppResults();
    void setBppResults(EvalBppResults bpp_results);

    void setRppRemoveBoxes(std::string &selected_uuid, std::vector<std::string>& uuids_remove);

    static bool saveFileLocallyForce(std::string fileURL, std::string filePath);
    static bool saveFileLocally(std::string fileURL, std::string filePath, std::string unzipPath = "", bool checkCache = false);
    static bool saveFileBlobStore(std::string fileURL, std::string filePath);
    static bool cleanTmpFilesActor(bpp_actor::Actor &actor);

    static inline bool fileExists(std::string fileName) { return std::ifstream(fileName.c_str()).good(); }
    static inline std::string getFacePath(std::string folderPath, std::string uuid, std::string name){ return folderPath + uuid + "_" + name + ".jpg"; }
    static inline std::string getFaceURL(std::string uuid, std::string name) { return "file:///home/nair/data/blob_store/" + uuid + "_" + name + ".jpg"; }
    static inline std::string getThumbnailPath(std::string folderPath, std::string uuid){ return folderPath + uuid + "_" + "thumbnail.jpg"; }
    static inline std::string getThumbnailURL(std::string uuid) { return "file:///home/nair/data/blob_store/" + uuid + "_" + "thumbnail.jpg"; }
    static inline std::string getWRLPath(std::string folderPath, std::string uuid, int bodyId){ return folderPath + uuid + "_body" + std::to_string(bodyId) + ".wrl"; }
    static inline std::string getCollisionWRLPath(std::string folderPath, std::string uuid){ return folderPath + uuid + "_collision" + ".wrl"; }
    static inline std::string getMeasureDebugPath(std::string folderPath, std::string uuid){ return folderPath + uuid + "_" + "measure_debug" + ".jpg"; }
    static inline std::string getMeasureDebugURL(std::string uuid){ return "file:///home/nair/data/blob_store/" + uuid + "_" + "measure_debug" + ".jpg"; }
    static inline std::string getMeshPathZipped(std::string folderPath, std::string uuid){ return folderPath + uuid + ".zip"; }
    static inline std::string getMeshPathUnzipped(std::string folderPath, std::string uuid){ return folderPath + uuid + "/"; }
    static inline std::string getMeshURLZipped(std::string uuid){ return "file:///home/nair/data/blob_store/" + uuid + ".zip"; }
    static inline std::string getObjectPointsPath(std::string folderPath, std::string uuid){ return folderPath + uuid + ".pcd"; }
    static inline std::string getObjectPointsURL(std::string uuid){ return "file:///home/nair/data/blob_store/" + uuid + ".pcd"; }
    static inline std::string getFeaturePointsPath(std::string folderPath, std::string uuid){ return folderPath + uuid + ".dat"; }
    static inline std::string getFeaturePointsURL(std::string uuid){ return "file:///home/nair/data/blob_store/" + uuid + ".dat"; }

    ObjectModel createModel(const bpp_actor::Actor &actor);
    QMatrix4x4 poseToQMatrix4x4(geometry_msgs::Pose pose);

    void updateSceneModels();
    void updateScenePlanningModels();

    wsg::WorldStateGenerator ws_;

    Eigen::Affine3d pallet_tr_to_world;

public slots:
    void saveBoxesJSON(bpp_actor::ActorVec &actors);
    void loadBoxesJSON(bpp_actor::ActorVec &actors);
    bool loadActorsFromJSONFolder(std::string folderPath, bpp_actor::ActorVec &actors);
    void resetBppSlot();

signals:
    void requestBinPackingPlan(const std::string);

    void evaluateBinPackingPlan();

    void requestActorFromUUID(const std::string);

    void requestMALocation();

    void backupCurrentWorldStateSignal(const std::string);

    void loadWorldStateSignal(const std::string);

    void resetPPSignal();

    void requestPalletizationPlan();

    void requestRePalletization(const std::string);



protected:
    void closeEvent(QCloseEvent *event);

    virtual void timerEvent(QTimerEvent* timerEvent);

private slots:
//    void on_loadFileButton_clicked();

    void on_MASimulationButton_clicked();

    void on_showPlanedPose_clicked(bool checked);

    void on_evalButton_clicked();

    void on_listWidget_itemSelectionChanged();

    void on_checkBox_PalletBB_clicked(bool checked);

    void on_checkBox_COM_clicked(bool checked);

    void on_radioButton_space_opt_clicked(bool checked);

    void on_radioButton_mass_opt_clicked(bool checked);

    void on_radioButton_com_opt_clicked(bool checked);

    void on_radioButton_default_clicked(bool checked);

    void on_rppButton_clicked();

    void on_importBtn_clicked();

    void on_closeBtn_clicked();

    void on_planBtn_clicked();

    void on_checkbox_wooden_clicked(bool checked);

    void on_checkbox_carton_clicked(bool checked);

    void on_checkbox_styrofoam_clicked(bool checked);

    void on_checkbox_plastic_clicked(bool checked);

    void on_checkbox_dangerous_clicked(bool checked);

    void on_checkbox_fragile_clicked(bool checked);

    void on_checkbox_others_clicked(bool checked);

    void on_checkBox_support_clicked(bool checked);

    void on_clearBtn_clicked();

    void on_resetviewBtn_clicked();

    void on_defaultviewBtn_clicked();

    void on_plannedviewBtn_clicked(bool checked);

    void on_analyticviewBtn_clicked();

    void on_volumeviewBtn_clicked(bool checked);

    void on_comviewBtn_clicked(bool checked);

    void on_resetButton_clicked();

    void on_textureviewBtn_clicked(bool checked);

    void on_saveButton_clicked();



private:
    Ui::BinPackingGUI *ui;

    int timer_id_;

    bool show_desired_pose_;
    QMutex mutex_show_desired_pose_;

    bpp_actor::ActorVec plan_actors_;
    QMutex mutex_plan_actors_;

    bool planning_finished_;
    QMutex mutex_planning_finished_;

    bool planning_request_;
    QMutex mutex_planning_request_;

    EvalBppResults bpp_results_;
    QMutex mutex_bpp_results_;

    bool bpp_done_;
    QMutex mutex_bpp_done_;

    std::string bpp_opt_goal_;

    QMessageBox msgBox;

    QAction *saveSceneAction;

    QMovie *movie_loading_;

    std::thread *thread_bpp_;

    void planReceived();

    int importedBoxesCount;


    SGJSONParser parser;


};

#endif // BINPACKINGGUI_H
