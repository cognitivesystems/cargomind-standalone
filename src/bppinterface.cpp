#include "mainwindow.h"
#include "BppROS.h"
#include "sginterface.h"
#include <thread>

namespace sginf
{

bool SGInterface::callback_update_pallet_fp()
{
    updatePalletFPsWSG();
    return true;
}

void SGInterface::fetchNewBoxPlanPriv(const std::string &opt_name)
{
    bpp_actor::ActorVec plan_actors;

    std::string plan_name="";
    if(!bpp_ros_.bin_packing_boxes_callback(plan_name, opt_name, plan_actors))
       std::cout<< "ERROR:Bin packing failed" << std::endl;

    std::cout << "Plan actors count -------------------------------------------> " << plan_actors.size() << std::endl;

    MainWindow::binPackingGUIInstance()->setPlanningFinished(true);
    MainWindow::binPackingGUIInstance()->setPlanActors(plan_actors);
    std::cout << "------------- Bpp process is Done!!\n";
}

void SGInterface::fetchNewBoxPlan(const std::string &opt_name)
{
    gui_msgs::WSGUIStateFeedback gui_state_feedback;
    gui_state_feedback.current_state = "doBinPackingPlanning";

    std::cout << "slot processed" << std::endl;
    //TODO: call bpp, get plan and update scene
    std::thread thread_bpp_ = std::thread(&SGInterface::fetchNewBoxPlanPriv, this, opt_name);
    thread_bpp_.detach();
}

void SGInterface::evaluateBinPlan()
{
    gui_msgs::WSGUIStateFeedback gui_state_feedback;
    gui_state_feedback.current_state = "doBinPackingEvaluation";

    float used_space;
    float packed_mass;
    geometry_msgs::Vector3 com;
    evaluation_callback(used_space, packed_mass, com);

    EvalBppResults bpp_results;
    bpp_results.bpp_used_space_ = used_space;
    bpp_results.bpp_packed_mass_ = packed_mass;
    bpp_results.bpp_com_ = QVector3D(com.x, com.y, com.z);
    MainWindow::binPackingGUIInstance()->setBppResults(bpp_results);
    std::cout << "------------- Evalation bpp is Done!!\n";
}

void SGInterface::removeBoxReplan(const std::string &uuid)
{
//    std::cout << "Service call to /rpp/re_palletize_planner!" << std::endl;
//    bpp_msgs::RePalletizationPlan srv;
//    srv.request.uuid_to_remove = uuid;
//    if(ros::service::call(std::string("/rpp/re_palletize_planner"), srv.request, srv.response))
//    {
//        if(!srv.response.success)
//        {
//            ROS_ERROR("/rpp/re_palletize_planner failed");
//            return;
//        }
//    }
//    else
//    {
//        ROS_ERROR("Service call to /rpp/re_palletize_planner failed!!!");
//        return;
//    }
//    std::cout << "------------- /rpp/re_palletize_planner is Done!!\n";

//    // set for depalletization execution
//    MainWindow::binPackingGUIInstance()->setRppRemoveBoxes(srv.request.uuid_to_remove, srv.response.uuids_remove);
}

void SGInterface::updatePalletFPsWSG()
{
    //compute the pallet fitting points after execution
//    bpp_actor::Actor pallet = wsgtools::WSGClientHelper::getActorFromUUID("p6ppallet");
    std::string pallet_name="p6ppallet";
    bpp_actor::Actor pallet = MainWindow::binPackingGUIInstance()->ws_.get_actor_from_uuid(pallet_name).front();

//    std::vector<bpp_actor::Actor> pallet_actors = wsgtools::WSGClientHelper::getActorsInZone("pallet");
    bpp_actor::QueryActor query;
    query.zone="pallet";
    std::vector<bpp_actor::Actor> pallet_actors;
    MainWindow::binPackingGUIInstance()->ws_.get_actor_callback(query, pallet_actors);

    //    std::cout << "=============== Packed box size = " << pallet_actors.size() << std::endl;
    //wsgtools::WSGClientHelper::computePalletFP(pallet, pallet_actors);
    bpp_ros_.computePalletFP(pallet, pallet_actors);
    //    std::cout << "Packed FPs in pallet = " << pallet.binFPs.size() << std::endl;
//    wsgtools::WSGClientHelper::updateActor(pallet);
    MainWindow::binPackingGUIInstance()->ws_.update_actor(pallet);
}

std::vector<bpp_actor::Actor> SGInterface::to_actors(const std::vector<bpp_msgs::BoxPlan>& box_plan)
{
    std::vector<bpp_actor::Actor> actors;
    for (size_t box_id = 0; box_id < box_plan.size(); box_id++)
    {
//        std::vector<bpp_actor::Actor> boxes_actors;
        bpp_actor::Actor actor=MainWindow::binPackingGUIInstance()->ws_.get_actor_from_uuid(box_plan[box_id].box_uuid).front();

//        bpp_actor::Actor actor = wsgtools::WSGClientHelper::getActorFromUUID(box_plan[box_id].box_uuid);
        actor.bbox.x = box_plan[box_id].bbox.x;
        actor.bbox.y = box_plan[box_id].bbox.y;
        actors.push_back(actor);
    }
    return actors;
}

double SGInterface::compute_used_space(const std::vector<bpp_actor::Actor>& actors)
{
    double bin_volume = 0.0;
    for(bpp_actor::Actor b : actors)
    {
        bin_volume += (b.bbox.x * b.bbox.y * b.bbox.z);
    }
    return bin_volume;
}

double SGInterface::compute_total_mass(const std::vector<bpp_actor::Actor>& actors)
{
    double bin_mass = 0.0;
    for(bpp_actor::Actor b : actors)
    {
        bin_mass += b.weight;
    }
    return bin_mass;
}

Eigen::Vector3d SGInterface::compute_bin_com(std::vector<bpp_actor::Actor>& actors)
{
    Eigen::Vector3d bin_com, box_com;
    double bin_mass = 0.0;
    for(bpp_actor::Actor b : actors)
    {
        box_com << b.desiredPoseVec[0].position.x, b.desiredPoseVec[0].position.y, b.desiredPoseVec[0].position.z;
        bin_com = (box_com * b.weight + bin_com * bin_mass) / (bin_mass + b.weight);
        bin_mass += b.weight;
    }
    return bin_com;
}

bool SGInterface::evaluation_callback(float &used_space, float &packed_mass, geometry_msgs::Vector3 &com)
{
    bool success=false;
    // This only get the current box plan. If there are mulple steps, this is not for the whole pallet!!
    std::vector<bpp_msgs::BoxPlan> box_plan;
    MainWindow::binPackingGUIInstance()->ws_.get_box_plan_callback(box_plan);
    std::vector<bpp_actor::Actor> actors = to_actors(box_plan);
    // get the already packed boxes
    bpp_actor::QueryActor query;
    query.zone="pallet";
    std::vector<bpp_actor::Actor> actors_packed;
    MainWindow::binPackingGUIInstance()->ws_.get_actor_callback(query, actors_packed);
//    std::vector<bpp_actor::Actor> actors_packed = wsgtools::WSGClientHelper::getActorsInZone("pallet");
//    actors.insert(actors.begin(), actors_packed.begin(), actors_packed.end());

    //////////////////////////////////////////////////
    // avoid repeat actors, after packed
    std::vector<std::string> actor_uuids;
    for(bpp_msgs::BoxPlan bp : box_plan)
    {
        actor_uuids.push_back(bp.box_uuid);
    }
    for(bpp_actor::Actor actor : actors_packed)
    {
        //if this actor does not exist
        if(find(actor_uuids.begin(),actor_uuids.end(),actor.uuid)==actor_uuids.end())
        {
            actors.push_back(actor);
        }
    }
    ////////////////////////////////////////////////

    used_space = compute_used_space(actors);
    packed_mass = compute_total_mass(actors);
    Eigen::Vector3d bin_com;
    bin_com = compute_bin_com(actors);
    success = true;
    if(actors.size()==0)
    {
        com.x = 0.0; // with respect to pallet center.
        com.y = 0.0;
        com.z = 0.0;
    }
    else
    {
        com.x = bin_com(0) - 2.264; // with respect to pallet center.
        com.y = bin_com(1) - 1.665;
        com.z = bin_com(2) - 0.050;
    }

    return true;
}


}
