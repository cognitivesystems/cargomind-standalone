/*
 * Frames:
 *        bpp pallet frame (left corner of pallet)
 *        world pallet frame (center of pallet)
 *        base frame (world frame)
 *
 * The frames of all actors wrt base frame are in the center of actors !-!
 *
 */
#include "BppROS.h"
#include <bullet/PhysicsBullet.h>
#include "mainwindow.h"

namespace bpa_ros
{

BppROS::BppROS()
{
    // setup parameters from a json file
    Json::Value paramJson;
    Json::Reader reader;
    std::string file_path="config/planningParams.json";
    std::cout <<"Json parameter path: --> " << file_path << std::endl;
    std::ifstream file(file_path);
    bool parsingSuccessful = reader.parse(file, paramJson);

    bbox_offset = paramJson["bbox_offset"].asDouble();
    pallet_frame_id = paramJson["pallet_frame_id"].asString();
    pallet_length = paramJson["pallet_size"]["length"].asDouble();  //2.44
    pallet_width = paramJson["pallet_size"]["width"].asDouble();    //3.18
    pallet_height = paramJson["pallet_size"]["height"].asDouble();  //3.10
    pallet_center(0) = paramJson["pallet_center_translation"]["x"].asDouble();
    pallet_center(1) = paramJson["pallet_center_translation"]["y"].asDouble();
    pallet_center(2) = paramJson["pallet_center_translation"]["z"].asDouble();

    //tranform from bpp pallet frame w.r.t world pallet frame
    pallet_tr_bpp_to_world=Eigen::Affine3d::Identity();

    pallet_tr_bpp_to_world.rotate (Eigen::AngleAxisd (0, Eigen::Vector3d::UnitZ()));
    pallet_tr_bpp_to_world.translation()[0]=-(pallet_length)/2.0;
    pallet_tr_bpp_to_world.translation()[1]=-(pallet_width)/2.0;
    pallet_tr_bpp_to_world.translation()[2]=0.0;

    std::cout << "Pallet Transform from bpp to world " << std::endl;
    std::cout << pallet_tr_bpp_to_world.matrix() << std::endl;

    //load the parameters from Json files for bpp
    parameter_file = "/bppParams.json";
    loadParamsJSON(parameter_file);
    //    std::cout << "Parameters: ==================\n W_MASS = " << bpa::Params::instance()->W_MASS <<"\n W_VOL = " << bpa::Params::instance()->W_VOL <<"\n W_MASSVOL = " << bpa::Params::instance()->W_MASSVOL << "\n W_COM = " << bpa::Params::instance()->W_COM << "\n HELT_RATE = " << bpa::Params::instance()->HELT_RATE << "\n W_SUPPORTED = " << bpa::Params::instance()->W_SUPPORTED <<"\n W_CONTACT = " << bpa::Params::instance()->W_CONTACT <<std::endl;

    // services
    // output status
    std::cout<< "Ready..." << std::endl;
}


BppROS::~BppROS()
{
}


bool BppROS::bin_packing_boxes_callback(const std::string& bin_name, const std::string& opt_name, bpp_actor::ActorVec &boxes_to_pack)
{
    //set parameters for bpp
    if(opt_name == "opt_space")
        parameter_file = "/bppParams_space.json";
    else if(opt_name == "opt_mass")
        parameter_file = "/bppParams_mass.json";
    else if(opt_name == "opt_com")
        parameter_file = "/bppParams_com.json";
    else
        parameter_file = "/bppParams.json";
    loadParamsJSON(parameter_file);

    boxes_to_pack = binPackingBoxes();

    std::cout << "Boxes to pack ---------------------------------> " << boxes_to_pack.size() << std::endl;

    return true;
}

std::vector<bpp_actor::Actor> BppROS::binPackingBoxes()
{
    boxes_pose_table.clear();
    /***************************************************************************************************
    * 1. Get current pallet state, holding area state and measurement area state
    * ************************************************************************************************/
    //    std::vector<bpp_actor::Actor> pallet_actors = wsgtools::WSGClientHelper::getActorsInZone("pallet");
    //    std::vector<bpp_actor::Actor> holding_area_actors = wsgtools::WSGClientHelper::getActorsInZone("holding_area");

    bpp_actor::QueryActor query;
    query.zone="pallet";
    std::vector<bpp_actor::Actor> pallet_actors;
    MainWindow::binPackingGUIInstance()->ws_.get_actor_callback(query, pallet_actors);
    query.zone="holding_area";
    std::vector<bpp_actor::Actor> holding_area_actors;
    MainWindow::binPackingGUIInstance()->ws_.get_actor_callback(query, holding_area_actors);

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "number of boxes is in Pallet = " << pallet_actors.size() << '\n';
    std::cout << "number of boxes is in HA is  = " << holding_area_actors.size() << '\n';

    if(holding_area_actors.size()==0)
    {
        query.zone="measurement_area";
        MainWindow::binPackingGUIInstance()->ws_.get_actor_callback(query, holding_area_actors);
        std::cout << "number of boxes is in MA is  = " << holding_area_actors.size() << '\n';
    }
    //for the all box in MA, the desiredPose = targetPose, it has bugs if don't change back the desiredPose from last bpp
    std::vector<bpp_actor::Actor> to_plan_actors;
    for(bpp_actor::Actor ac : holding_area_actors)
    {
        ac.desiredPoseVec.clear();
        ac.desiredPoseVec.resize(ac.targetPoseVec.size());
        ac.desiredPoseVec[0] = ac.targetPoseVec[0];
        to_plan_actors.push_back(ac);
        MainWindow::binPackingGUIInstance()->ws_.update_actor(ac);
    }

    // get current fps for bin
    bpp_actor::QueryActor query1;
    query1.uuid="p6ppallet";
    std::vector<bpp_actor::Actor > actors;
    MainWindow::binPackingGUIInstance()->ws_.get_actor_callback(query1, actors);
    bpp_actor::Actor p6ppallet = actors.front();

    std::cout << "Packed FPs in pallet 1= " << p6ppallet.binFPs.size() << std::endl;
    computePalletFP(p6ppallet, pallet_actors);
    // should update the pappallet.binFPs after move the boxes in pallet. = 6ppallet.fitting_points
    std::vector<bpp_actor::FittingPoint> pallet_fps_base = p6ppallet.binFPs;
    std::cout << "Packed FPs in pallet 2= " << p6ppallet.binFPs.size() << std::endl;

    // translate 'actors' to internal bin data structures (box), frames
    std::vector<bpa::Box> pallet_boxes = actorsToBoxes(pallet_actors);
    std::cout << "!!!!!!!Packed boxes in Pallet = " << pallet_boxes.size() << '\n';

    std::vector<bpa::FittingPoint> bin_fps_bpp = fpsWrtBppFrame(pallet_fps_base);

    std::vector<bpa::Box> holding_area_boxes = actorsToBoxes(to_plan_actors);

    /***************************************************************************************************
    * 2. Do bin packing
    * ************************************************************************************************/
    bpa::Bin new_pallet_config(pallet_length, pallet_width, pallet_height, bpa::Params::instance()->START_WITH_ALL_EDGES_AS_FP);

    // intial the pallet fitting points, 1 left corner fp or more
    std::vector<bpa::FittingPoint> fps = new_pallet_config.fitting_points;

    if(p6ppallet.binFPs.size()==0)
    {
        updateBinFPsToWSG(fps, true); // if its true, its the initial fps for the pallet;
    }

    // update packed boxes and bin fitting points, state before bpp
    new_pallet_config.setPackedBoxes(pallet_boxes);
    new_pallet_config.setBinFittingPoints(bin_fps_bpp);

    // build the physics world for bullet
    new_pallet_config.bulletPhysics->addBinBoundingBox();
    new_pallet_config.bulletPhysics->addNewBoxesToPhysics(new_pallet_config.packed_boxes);
    std::cout << "************************size of the  packed boxes from last bpp is :" << new_pallet_config.packed_boxes.size() <<std::endl;
    std::cout << "************************size of the fitting points from last bpp is:" << new_pallet_config.fitting_points.size() <<std::endl;

    std::cout << "\n\nBin Packing.................................................................................\n";
    bpa::BinPackingPlanner bin_packing_planner;
    new_pallet_config = bin_packing_planner.solveWithOneFunction(new_pallet_config, holding_area_boxes);

    /// get the current step packed boxes after bpp
    std::vector<bpa::Box> pack_boxes = new_pallet_config.getStepPackedBoxes();
    std::cout << "\nCurrent box plan size ==================================================" << pack_boxes.size() << std::endl;
    std::cout << "Total boxes in pallet will be = " << new_pallet_config.packed_boxes.size() << std::endl;

    for(bpa::Box b : pack_boxes)
    {
        std::cout << "Bpp UUID " << b.m_name << "  l " << b.m_length << "  w " << b.m_width << "  h " << b.m_height << "  mass " << b.m_mass <<" x " << b.position.position(0) << " y " << b.position.position(1) << " z " << b.position.position(2) << "  rotated " << b.is_rotated << "  rotation " << b.rotation << " " << b.material << "--";

        for(std::string label : b.box_labels)
        {
            std::cout <<" " << label;
        }
        std::cout << ";  " << b.is_stackable;
        std::cout <<" \n";
    }
    std::cout << "==================================================================" << std::endl;

    /***************************************************************************************************
    * 3. Update the bpp results to WSG
    * ************************************************************************************************/
    // packed boxes ---> BoxPlan, in the same frame
    std::vector<bpp_msgs::BoxPlan> boxes_to_pack = boxesToBoxPlan(pack_boxes);
    updateBoxPlanToWSG(boxes_to_pack);

    std::cout << "Boxes to pack ---------------------------------> " << boxes_to_pack.size() << std::endl;

    // boxes to actors back: Box position from inside bpp frame --> outside world base frame
    std::vector<bpp_actor::Actor> actors_to_pack = boxesToActors(pack_boxes);
    updateActorsToWSG(actors_to_pack);
    std::cout << "actors_to_pack ---------------------------------> " << actors_to_pack.size() << std::endl;

    // update the pallet fitting points
    std::vector<bpa::FittingPoint> fitting_points = new_pallet_config.fitting_points;
    updateBinFPsToWSG(fitting_points, false);

    return actors_to_pack;
}

// 1. world base frame ---> bpp pallet frame,
// 2. change the box position from corner ---> center
std::vector<bpa::Box> BppROS::actorsToBoxes(const std::vector<bpp_actor::Actor>& actors)
{
    std::vector<bpa::Box> boxes_vector;
    for (size_t actor_id = 0; actor_id < actors.size(); actor_id++)
    {
        boxes_vector.push_back(this->actorToBox(actors[actor_id]));
    }

    return boxes_vector;
}

bpa::Box BppROS::actorToBox(const bpp_actor::Actor& actor)
{
    Eigen::Vector3d box_bbox;
    geometry_msgs::Quaternion q; Eigen::Vector3d eu;
    //********************************************* REIMPLEMENT ************************************************

    /*    tf::quaternionMsgToTF(actor.desiredPoseVec[0].orientation, q);
    tf::Matrix3x3(q).getRPY(eu(0), eu(1), eu(2))*/;
    //*********************************************************************************************************



    std::cout << "re implement the commeneted conde in the file below " << std::endl;

    std::cout << __FILE__ << " " << __LINE__ << std::endl;
    throw;

    if( floatEqual(std::fabs(eu(2)), M_PI_2) && actor.zone == "pallet") //box is the rotatedBox
        box_bbox << actor.bbox.y, actor.bbox.x, actor.bbox.z;
    else
        box_bbox << actor.bbox.x, actor.bbox.y, actor.bbox.z;

    bpa::Box box(box_bbox(0)+bbox_offset, box_bbox(1)+bbox_offset, box_bbox(2), actor.weight, actor.uuid, actor.labels, actor.material, actor.robotTool);

    // change from base to world pallet frame
    geometry_msgs::Pose _pose;
    Eigen::Affine3d _pose_tr, _pose_bpp_tr;
    //    _pose = tftools::TFClientHelper::getActorPoseWrtFrame(nh_, actor.uuid, "p6ppallet"); //its box currentPose, not the desiredPose
    //    geometry_msgs::PoseStamped poseStamp;
    //    poseStamp.header.frame_id = "base";
    //    poseStamp.pose = actor.desiredPoseVec[0];
    //    geometry_msgs::PoseStamped transPose = tftools::TFClientHelper::transformPoseWrtFrame(nh_, poseStamp, pallet_frame_id);
    //    _pose= transPose.pose;

    Eigen::Affine3d box_tr_base;
    Eigen::Affine3d box_tr_pallet;
    tf::poseMsgToEigen(actor.desiredPoseVec[0], box_tr_base);
    box_tr_pallet=MainWindow::binPackingGUIInstance()->pallet_tr_to_world.inverse()*box_tr_base;
    tf::poseEigenToMsg(box_tr_pallet, _pose);

    // change from world pallet frame to bpp pallet frame
    tf::poseMsgToEigen(_pose, _pose_tr);
    _pose_bpp_tr=pallet_tr_bpp_to_world.inverse()*_pose_tr;

    box.position.position(0) = _pose_bpp_tr.translation()[0];
    box.position.position(1) = _pose_bpp_tr.translation()[1];
    box.position.position(2) = _pose_bpp_tr.translation()[2];

    //    std::cout <<actor.uuid << " state = " << actor.state  << ";  zone = " <<actor.zone<< std::endl;
    if(actor.state == "packed")
        box.is_packed = true;
    if(actor.type == "other" || actor.type == "pallet")
        box.is_stackable = false;

    //center to corner
    box.position.position(0) -= box.m_length/2.0;
    box.position.position(1) -= box.m_width/2.0;
    box.position.position(2) -= box.m_height/2.0;
    if( floatEqual(std::fabs(eu(2)), M_PI_2) && actor.zone == "pallet")  // if the box desired is rotated(90 or -90), l and w opposite
    {
        box.rotation = 90;
        box.is_rotated = true;
    }
    else
    {
        box.rotation = 0;
        box.is_rotated = false;
    }

    if(box.is_packed)
    {
        std::cout << "@@@@@@ Actor to Box " << box.m_name <<"  l " << box.m_length << "  w " << box.m_width << "  h " << box.m_height << "  mass " << box.m_mass <<"   position=" << box.position.position.transpose() <<"  rotated " << box.is_rotated << "  rotation " << box.rotation<< std::endl;
    }

    return box;
}

std::vector<bpa::FittingPoint> BppROS::fpsWrtBppFrame(const std::vector<bpp_actor::FittingPoint> &fitting_points)
{
    std::vector<bpa::FittingPoint> fps_bin;
    fps_bin.clear();

    for(size_t i=0; i < fitting_points.size(); ++i)
    {
        //change from base to world pallet frame
        geometry_msgs::Point fp_pos_pallet;
        fp_pos_pallet.x = fitting_points[i].position.x - pallet_center(0);
        fp_pos_pallet.y = fitting_points[i].position.y - pallet_center(1);
        fp_pos_pallet.z = fitting_points[i].position.z - pallet_center(2);

        //change position from world pallet frame to bpp frame
        Eigen::Vector3d fp_vector_bpp, fp_vector_pallet;
        tf::pointMsgToEigen(fp_pos_pallet, fp_vector_pallet);
        fp_vector_bpp = pallet_tr_bpp_to_world.inverse().rotation() * fp_vector_pallet + pallet_tr_bpp_to_world.inverse().translation();

        bpa::FittingPoint fp(bpa::Point(fp_vector_bpp), fitting_points[i].quadrant);
        fps_bin.push_back(fp);
    }
    return fps_bin;
}

// Box.h to BoxPlan.msg, the same frame
std::vector<bpp_msgs::BoxPlan> BppROS::boxesToBoxPlan(const std::vector<bpa::Box> &boxes)
{
    std::vector<bpp_msgs::BoxPlan> box_plan;
    for (size_t box_id = 0; box_id < boxes.size(); box_id++)
    {
        box_plan.push_back(this->boxToBoxPlan(boxes[box_id]));
    }
    return box_plan;
}

bpp_msgs::BoxPlan BppROS::boxToBoxPlan(const bpa::Box &box)
{
    bpp_msgs::BoxPlan bp;
    bp.box_uuid = box.m_name;
    bp.box_pose_in_pallet.position.x =  box.position.position(0);
    bp.box_pose_in_pallet.position.y =  box.position.position(1);
    bp.box_pose_in_pallet.position.z =  box.position.position(2);
    bp.is_rotated = box.is_rotated;
    bp.box_state = "planed";
    bp.box_mass = box.m_mass;
    bp.labels = box.box_labels;
    bp.material = box.material;
    bp.robot_tool = box.tool_name;
    if(box.is_rotated)
    {   // rotated box, the l and w is opposite wrt to the real box
        bp.bbox.x = box.m_width;
        bp.bbox.y = box.m_length;
        bp.bbox.z = box.m_height;
    }
    else
    {
        bp.bbox.x = box.m_length;
        bp.bbox.y = box.m_width;
        bp.bbox.z = box.m_height;
    }

    // pass some status to box plan
    //    bpp_actor::Actor actor= wsgtools::WSGClientHelper::getActorFromUUID(box.m_name);
    //    bp.bbox = actor.bbox;//
    //    bp.box_mass = actor.weight;
    //    bp.labels = actor.labels;
    //    bp.material = actor.material;
    return bp;
}

// box---> actor: change the frame from bpp pallet ---> world base frame
std::vector<bpp_actor::Actor> BppROS::boxesToActors(const std::vector<bpa::Box>& boxes)
{
    std::vector<bpp_actor::Actor> actors;
    for (size_t box_id = 0; box_id < boxes.size(); box_id++)
    {
        actors.push_back(this->boxToActor(boxes[box_id]));
    }
    return actors;
}

bpp_actor::Actor BppROS::boxToActor(const bpa::Box& box)
{
    /***************************************************************************************************
     * 1. Rotation for the box;
     * ************************************************************************************************/
    //    tf::Quaternion _real_world_orientation;
    //    tf::quaternionMsgToTF(boxes_pose_table[box.m_name].pose.orientation, _real_world_orientation);
    double _sim_world_yaw = (box.is_rotated == true) ? M_PI_2 : 0.0;


    //********************************************* REIMPLEMENT ************************************************

//    tf::Quaternion _sim_world_orientation = tf::createQuaternionFromYaw(_sim_world_yaw);
//    tf::Quaternion _final_orientation =  _sim_world_orientation;

    //*********************************************************************************************************

    std::cout << "re implement the commeneted conde in the file below " << std::endl;

    std::cout << __FILE__ << " " << __LINE__ << std::endl;
    throw;

    /***************************************************************************************************
     * 2. tranformation the box pose to the base frame (actor)
     * ************************************************************************************************/
    // corner ---> center
    // a. bpp pallet frame ---> world pallet frame
    // create the data
    bpp_actor::Actor actor_box;
    std::vector<bpp_actor::Actor > actors_box=MainWindow::binPackingGUIInstance()->ws_.get_actor_from_uuid(box.m_name);
    actor_box=actors_box.front();
    //    bpp_actor::Actor actor_box = wsgtools::WSGClientHelper::getActorFromUUID(box.m_name);
    actor_box.uuid = box.m_name;
    //    actor_box.zone = "pallet";
    actor_box.state = "planed";
    actor_box.robotTool = box.tool_name;
    if(actor_box.robotTool == "cranepalletfork")
    {
        actor_box.forkLeftTine = "l2";
        actor_box.forkRightTine = "r2";
    }

    geometry_msgs::Pose box_pose_wrt_bpp;
    box_pose_wrt_bpp.position.x = box.position.position(0) + box.m_length / 2.0; //don't need conside the rotation, because is the rotated box
    box_pose_wrt_bpp.position.y = box.position.position(1) + box.m_width / 2.0;
    box_pose_wrt_bpp.position.z = box.position.position(2) + box.m_height / 2.0;



    //********************************************* REIMPLEMENT ************************************************

    //tf::quaternionTFToMsg(_final_orientation, box_pose_wrt_bpp.orientation);

    //*********************************************************************************************************

    std::cout << "re implement the commeneted conde in the file below " << std::endl;

    std::cout << __FILE__ << " " << __LINE__ << std::endl;
    throw;

    Eigen::Affine3d box_tr_wrt_bpp, box_tr_wrt_world;
    tf::poseMsgToEigen(box_pose_wrt_bpp, box_tr_wrt_bpp);
    box_tr_wrt_world=pallet_tr_bpp_to_world*box_tr_wrt_bpp;
    tf::poseEigenToMsg(box_tr_wrt_world, actor_box.desiredPoseVec[0]);

    /*********************************************************************/
    //b. world pallet frame--> world base frame for box.pose
    //    geometry_msgs::PoseStamped poseStamp;
    //    poseStamp.header.frame_id = pallet_frame_id;
    //    poseStamp.pose = actor_box.desiredPoseVec[0];
    //    geometry_msgs::PoseStamped transPose = tftools::TFClientHelper::transformPoseWrtFrame(nh_, poseStamp, "base");
    //    actor_box.desiredPoseVec[0]= transPose.pose;


    Eigen::Affine3d box_tr_base;
    Eigen::Affine3d box_tr_pallet;
    tf::poseMsgToEigen(actor_box.desiredPoseVec[0], box_tr_pallet);
    box_tr_base=MainWindow::binPackingGUIInstance()->pallet_tr_to_world*box_tr_pallet;
    tf::poseEigenToMsg(box_tr_base, actor_box.desiredPoseVec[0]);
    tf::poseEigenToMsg(box_tr_base, actor_box.targetPoseVec[0]);


    //hake for tumcreate demo
    if(box.m_name == "00ae6f3f" || box.m_name == "17d6be6b" || box.m_name == "a207eac3")
        actor_box.desiredPoseVec[0].position.y += 0.20;

    /*********************************************************************/
    // fitting points from box to actor, fps for box after packed this box
    actor_box.fittingPoints.clear();
    actor_box.fittingPoints.resize(box.box_fps.size());
    for(size_t i=0; i < box.box_fps.size(); ++i)
    {
        //change position from internal bpp frame to pallet frame
        geometry_msgs::Point fp_pos_bpp;
        fp_pos_bpp.x = box.box_fps[i].coordinates.position(0);
        fp_pos_bpp.y = box.box_fps[i].coordinates.position(1);
        fp_pos_bpp.z = box.box_fps[i].coordinates.position(2);

        Eigen::Vector3d fp_vector_bpp, fp_vector_world;
        tf::pointMsgToEigen(fp_pos_bpp, fp_vector_bpp);
        fp_vector_world=pallet_tr_bpp_to_world.rotation() * fp_vector_bpp + pallet_tr_bpp_to_world.translation();
        tf::pointEigenToMsg(fp_vector_world, actor_box.fittingPoints[i].position);
        //change to world base frame
        actor_box.fittingPoints[i].position.x += pallet_center(0);
        actor_box.fittingPoints[i].position.y += pallet_center(1);
        actor_box.fittingPoints[i].position.z += pallet_center(2);

        actor_box.fittingPoints[i].quadrant = box.box_fps[i].quadrant;
    }
    // fps for pallet after packed this box
    actor_box.binFPs.clear();
    actor_box.binFPs.resize(box.bin_fps.size());
    for(size_t i=0; i < box.bin_fps.size(); ++i)
    {
        //change position from internal bpp frame to pallet frame
        geometry_msgs::Point fp_pos_bpp;
        fp_pos_bpp.x = box.bin_fps[i].coordinates.position(0);
        fp_pos_bpp.y = box.bin_fps[i].coordinates.position(1);
        fp_pos_bpp.z = box.bin_fps[i].coordinates.position(2);

        Eigen::Vector3d fp_vector_bpp, fp_vector_world;
        tf::pointMsgToEigen(fp_pos_bpp, fp_vector_bpp);
        fp_vector_world=pallet_tr_bpp_to_world.rotation() * fp_vector_bpp + pallet_tr_bpp_to_world.translation();
        tf::pointEigenToMsg(fp_vector_world, actor_box.binFPs[i].position);
        //change to world base frame
        actor_box.binFPs[i].position.x += pallet_center(0);
        actor_box.binFPs[i].position.y += pallet_center(1);
        actor_box.binFPs[i].position.z += pallet_center(2);

        actor_box.binFPs[i].quadrant = box.bin_fps[i].quadrant;
    }

    //pass supporting boxes
    actor_box.supportBoxes.clear();
    for(size_t j=0; j < box.support_boxes.size(); ++j)
    {
        bpp_actor::SupportBox sbox;
        sbox.boxID= box.support_boxes[j].uuid;
        sbox.helt = box.support_boxes[j].helt;

        actor_box.supportBoxes.push_back(sbox);
    }

    return actor_box;
}

bool BppROS::updateBinFPsToWSG(const std::vector<bpa::FittingPoint> &fitting_points, bool intial)
{
    // for pallet fitting points
    bpp_actor::QueryActor query;
    query.uuid="p6ppallet";

    //    bpp_actor::Actor pallet = wsgtools::WSGClientHelper::getActorFromUUID("p6ppallet");
    std::vector<bpp_actor::Actor > actors_pallet;
    bpp_actor::Actor pallet;
    std::string pallet_name="p6ppallet";
    actors_pallet= MainWindow::binPackingGUIInstance()->ws_.get_actor_from_uuid(pallet_name);
    pallet=actors_pallet.front();

    pallet.fittingPoints.clear();
    pallet.fittingPoints.resize(fitting_points.size());
    for(size_t i=0; i < fitting_points.size(); ++i)
    {
        geometry_msgs::Point bfp_pos_bpp;
        bfp_pos_bpp.x = fitting_points[i].coordinates.position(0);
        bfp_pos_bpp.y = fitting_points[i].coordinates.position(1);
        bfp_pos_bpp.z = fitting_points[i].coordinates.position(2);

        Eigen::Vector3d bfp_vector_bpp, bfp_vector_world;
        tf::pointMsgToEigen(bfp_pos_bpp, bfp_vector_bpp);
        bfp_vector_world=pallet_tr_bpp_to_world.rotation() * bfp_vector_bpp + pallet_tr_bpp_to_world.translation();
        tf::pointEigenToMsg(bfp_vector_world, pallet.fittingPoints[i].position);
        //change to world base frame
        pallet.fittingPoints[i].position.x += pallet_center(0);
        pallet.fittingPoints[i].position.y += pallet_center(1);
        pallet.fittingPoints[i].position.z += pallet_center(2);
        pallet.fittingPoints[i].quadrant = fitting_points[i].quadrant;
    }

    // at beginning, the binFPs = fittingPoints = 1
    if(intial)
    {
        pallet.binFPs = pallet.fittingPoints;
    }

    //    return wsgtools::WSGClientHelper::updateActor(pallet);
    return MainWindow::binPackingGUIInstance()->ws_.update_actor(pallet);
}

bool BppROS::updateActorsToWSG(const std::vector<bpp_actor::Actor>& actors)
{
    for (size_t actor_id = 0; actor_id < actors.size(); actor_id++)
    {
        //        wsgtools::WSGClientHelper::updateActor(actors[actor_id]);
        MainWindow::binPackingGUIInstance()->ws_.update_actor(actors[actor_id]);
    }
    return true;
}

void BppROS::loadParamsJSON(std::string &file_name)
{
    Json::Value paramJson;
    Json::Reader reader;

    //get the file path
    std::string file_path="config"+file_name;
    std::cout <<"Json parameter path: --> " << file_path << std::endl;
    std::ifstream file(file_path);
    bool parsingSuccessful = reader.parse( file, paramJson);

    if( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout  << "Failed to parse configuration: "<< reader.getFormattedErrorMessages();
    }
    std::cout << paramJson["W_MASS"].asDouble() << std::endl;
    bpa::Params::instance()->W_MASS = paramJson["W_MASS"].asDouble();
    bpa::Params::instance()->W_VOL = paramJson["W_VOL"].asDouble();
    bpa::Params::instance()->W_MASSVOL = paramJson["W_MASSVOL"].asDouble();

    bpa::Params::instance()->W_COM = paramJson["W_COM"].asDouble();
    bpa::Params::instance()->HELT_RATE = paramJson["HELT_RATE"].asDouble();
    bpa::Params::instance()->W_SUPPORTED = paramJson["W_SUPPORTED"].asDouble();
    bpa::Params::instance()->W_CONTACT = paramJson["W_CONTACT"].asDouble();

    bpa::Params::instance()->NEIGHBOUR_CONSTANT = paramJson["NEIGHBOUR_CONSTANT"].asDouble();
    bpa::Params::instance()->W_ASSIGNMENT = paramJson["W_ASSIGNMENT"].asDouble();
    bpa::Params::instance()->W_PLACE_NEAR = paramJson["W_PLACE_NEAR"].asDouble();
    bpa::Params::instance()->BIN_HEIGHT = paramJson["BIN_HEIGHT"].asDouble();
    bpa::Params::instance()->MIN_BOX_SIZE = paramJson["MIN_BOX_SIZE"].asDouble();

    bpa::Params::instance()->W_ITEM_IN_THE_BOTTOM_AREA = paramJson["W_ITEM_IN_THE_BOTTOM_AREA"].asDouble();
    bpa::Params::instance()->W_HIGH_ITEMS_GOOD_PLACED = paramJson["W_HIGH_ITEMS_GOOD_PLACED"].asDouble();

    bpa::Params::instance()->GENERATE_SIMULATED_BOXES = paramJson["GENERATE_SIMULATED_BOXES"].asBool();
    bpa::Params::instance()->START_WITH_ALL_EDGES_AS_FP = paramJson["START_WITH_ALL_EDGES_AS_FP"].asBool();
    bpa::Params::instance()->SEARCH_HEIGHT = paramJson["SEARCH_HEIGHT"].asInt();
    bpa::Params::instance()->SEARCH_WIDTH = paramJson["SEARCH_WIDTH"].asInt();
}

void BppROS::computePalletFP(bpp_actor::Actor &pallet, std::vector<bpp_actor::Actor> &packed_boxes)
{
    /// TODO after the real robot execution
    std::vector<bpp_actor::FittingPoint> bin_fps;
    for(bpp_actor::Actor box : packed_boxes)
    {
        for(bpp_actor::FittingPoint fp : box.fittingPoints)
        {
            //check if this fp is already exist in the vector
            //            if(find(bin_fps.begin(),bin_fps.end(),fp)==bin_fps.end())
            int id = -1;
            if(!isFPEqual(fp, bin_fps, id))
            {
                bin_fps.push_back(fp);
            }
        }
    }
    //    std::cout <<"Fps before delete are " << bin_fps.size() << std::endl;

    // Then delete the Fitting Point where the boxes are placed
    for(bpp_actor::Actor box : packed_boxes)
    {
        bpp_actor::FittingPoint boxOrigin;

        Eigen::Vector3d eu;

        //********************************************* REIMPLEMENT ************************************************

        //        tf::Quaternion q;
        //        Eigen::Vector3d eu;
        //        tf::quaternionMsgToTF(box.targetPoseVec[0].orientation, q);
        //        tf::Matrix3x3(q).getRPY(eu(0), eu(1), eu(2));

        //*********************************************************************************************************

        std::cout << "re implement the commeneted conde in the file below " << std::endl;

        std::cout << __FILE__ << " " << __LINE__ << std::endl;
        throw;

        if( floatEqual(std::fabs(eu(2)), M_PI_2))  // if the box desired is rotated(90 or -90)
        {
            boxOrigin.position.x = box.targetPoseVec[0].position.x + box.bbox.y/2.0;
            boxOrigin.position.y = box.targetPoseVec[0].position.y - box.bbox.x/2.0;
            boxOrigin.position.z = box.targetPoseVec[0].position.z - box.bbox.z/2.0;
        }
        else
        {
            boxOrigin.position.x = box.targetPoseVec[0].position.x + box.bbox.x/2.0;
            boxOrigin.position.y = box.targetPoseVec[0].position.y - box.bbox.y/2.0;
            boxOrigin.position.z = box.targetPoseVec[0].position.z - box.bbox.z/2.0;
        }
        boxOrigin.quadrant = 4;
        //        boxOrigin.quadrant = pallet.fittingPoints[0].quadrant;
        removeFittingpoint(boxOrigin, bin_fps);
    }
    //    std::cout <<"Fps after delete are " << bin_fps.size() << std::endl;


    if(bin_fps.size() > 0)
    {
        pallet.binFPs.clear();
        pallet.binFPs = bin_fps;
    }
}

//check if this fp is already exist in the vector
bool BppROS::isFPEqual(bpp_actor::FittingPoint &fp, std::vector<bpp_actor::FittingPoint> &bin_fps, int &id)
{
    for(size_t i = 0; i<bin_fps.size(); ++i)
    {
        bpp_actor::FittingPoint bin_fp = bin_fps[i];
        double errorX, errorY, errorZ;
        errorX = fp.position.x-bin_fp.position.x;
        errorY = fp.position.y-bin_fp.position.y;
        errorZ = fp.position.z-bin_fp.position.z;

        if( (fabs(errorX) < 0.001) && (fabs(errorY) < 0.001) && (fabs(errorZ) < 0.001) && (fp.quadrant==bin_fp.quadrant) )
        {
            id = i;
            return true;
        }
    }
    return false;
}

void BppROS::removeFittingpoint(bpp_actor::FittingPoint &fp, std::vector<bpp_actor::FittingPoint> &bin_fps)
{
    int id = -1;
    if(isFPEqual(fp, bin_fps, id))
    {
        bin_fps.erase(bin_fps.begin()+id);
    }
}

bool BppROS::updateBoxPlanToWSG(const std::vector<bpp_msgs::BoxPlan >& plan)
{
    MainWindow::binPackingGUIInstance()->ws_.update_box_plan_callback(plan);
}

}


