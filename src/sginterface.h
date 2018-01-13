#ifndef SGINTERFACE_H_
#define SGINTERFACE_H_

#include <bullet/Actor.h>

//#include <gui_msgs/WSGUIStateAction.h>
//#include <gui_msgs/Dialog.h>
//#include <gui_msgs/Progress.h>
#include <gui_lib/scenerenderer3d.h>
#include "BppROS.h"

#include "defs.h"

#include <QThread>
#include <QMutex>
#include <algorithm>

Q_DECLARE_METATYPE(Transforms);
Q_DECLARE_METATYPE(ObjectBody);
Q_DECLARE_METATYPE(ObjectModel);
Q_DECLARE_METATYPE(std::vector<QString>);
Q_DECLARE_METATYPE(std::string);
Q_DECLARE_METATYPE(float);
Q_DECLARE_METATYPE(bpp_actor::ActorVec);
Q_DECLARE_METATYPE(bpp_actor::Actor);
//Q_DECLARE_METATYPE(gui_msgs::DialogRequest);
//Q_DECLARE_METATYPE(sensor_msgs::Image);
//Q_DECLARE_METATYPE(std::vector<bpp_actor::Actor>);
//Q_DECLARE_METATYPE(perception_msgs::PalletData);
//Q_DECLARE_METATYPE(gui_msgs::Progress);


typedef ::Eigen::Matrix< double, 4, 1 >  JointData;


namespace sginf
{
class SGInterface : public QThread
{
    Q_OBJECT

public:
    SGInterface();
    ~SGInterface();

    virtual void run();

    void stopThread();

    bool callback_update_pallet_fp();

    std::vector<bpp_actor::Actor> to_actors(const std::vector<bpp_msgs::BoxPlan>& box_plan);
    double compute_used_space(const std::vector<bpp_actor::Actor>& actors);
    double compute_total_mass(const std::vector<bpp_actor::Actor>& actors);
    Eigen::Vector3d compute_bin_com(std::vector<bpp_actor::Actor>& actors);
    bool evaluation_callback(float& used_space, float& packed_mass, geometry_msgs::Vector3& com);


signals:

    void setActors(const bpp_actor::ActorVec&);

    void finished();

    void populateBoxList(const std::vector<bpp_actor::Actor>& actors);

    void enableNextStep();

    void setProgressSignal(const gui_msgs::Progress& progress);

public slots:

    void fetchNewBoxPlan(const std::string& opt_name);

    void evaluateBinPlan();

    void updatePalletFPsWSG();

    void removeBoxReplan(const std::string &uuid);

private:
    bool run_thread_;

    std::vector<QString> node_names_;
    QMutex mutex_node_names_;

    bpp_actor::ActorVec actor_vec_;
    QMutex mutex_actor_vec_;

    void fetchNewBoxPlanPriv(const std::string &opt_name);

    std::map<std::string, std::string> wsg_backups;

    bpa_ros::BppROS bpp_ros_;
};

}

#endif /* SGINTERFACE_H_ */
