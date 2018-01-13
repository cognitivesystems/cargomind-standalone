#include "WorldStateGenerator.h"


#define CHECK_UUID_PRESENCE() do{ \
    if (request.actor.uuid.empty() && request.actor.semanticName.empty()) \
{ \
    std::cout <<"No UUID/Name available for the Actor!!!"<<std::endl; \
    response.success = false; \
    return false; \
    } \
    } while(0);

#define CHECK_UUID_PRESENCE_1() do{ \
    if (actor.uuid.empty() && actor.semanticName.empty()) \
{ \
    std::cout <<"No UUID/Name available for the Actor!!!"<<std::endl; \
    return false; \
    } \
    } while(0);


namespace wsg
{
WorldStateGenerator::WorldStateGenerator()
{
    // setup parameters in rosparam server
    use_simulation_ = false;//nh_.param<bool>("use_simulation", false);
    static_actors_file_ = "";//nh_.param<std::string>("static_actors_file", "");
    world_frame_name_ = "world";//nh_.param<std::string>("world_frame_name", "world");
    std::vector<std::string> _valid_zones{"detection_area", "measurement_area",
                                          "holding_area", "pallet", "static"};
//    valid_zones_ = nh_.param<std::vector<std::string>>("valid_zones", _valid_zones);
    valid_zones_=_valid_zones;
    std::vector<std::string> _valid_states{"to_pack", "to_palletise", "done"};
//    valid_states_ = nh_.param<std::vector<std::string>>("valid_states", _valid_states);
    valid_states_=_valid_states;
    std::vector<std::string> _valid_types{"box", "static"};
//    valid_types_ = nh_.param<std::vector<std::string>>("valid_types", _valid_types);
    valid_types_=_valid_types;
    // init the world state register
    init_world_state();
    // services

    // output status
    std::string _sim_status = use_simulation_ ? "ON" : "OFF";
    std::cout << "Using Simulation = " << _sim_status << std::endl;
    std::cout << "World Frame Name = " << world_frame_name_ << std::endl;
    std::cout <<"Ready..." << std::endl;
}


WorldStateGenerator::~WorldStateGenerator()
{
    stop_thread_flag_ = true;
}


bool WorldStateGenerator::get_actor_callback(bpp_actor::QueryActor& query, std::vector<bpp_actor::Actor >& actors)
{
    bool success=false;
    auto _world_state = get_register();
    auto _is_valid_filter = [](const std::vector<std::string> &filter_items,
            const std::string &filter_value)
    {
        return std::find(std::begin(filter_items),
                         std::end(filter_items), filter_value) != std::end(filter_items);
    };
    // All
    if (query.all)
    {
        actors = get_register_as_list(_world_state);
        success = true;
    }
    // Zone filter
    else if (!query.zone.empty())
    {
        auto _zone = query.zone;
        if (!_is_valid_filter(valid_zones_, _zone))
        {
            std::cout << "Invalid Zone. zone = " << _zone << std::endl;
            success = false;
        }
        else
        {
            actors = get_actors_in_zone(_zone);
            success = true;
        }
    }
    // State filter
    else if (!query.state.empty())
    {
        auto _state = query.state;
        if (!_is_valid_filter(valid_states_, _state))
        {
            std::cout << "Invalid State. state = "<< _state<< std::endl;
            success = false;
        }
        else
        {
            actors = get_actors_in_state(_state);
            success = true;
        }
    }
    // Type filter
    else if (!query.type.empty())
    {
        auto _type = query.type;
        if (!_is_valid_filter(valid_types_, _type))
        {
            std::cout << "Invalid Type. type = "<< _type<< std::endl;
            success = false;
        }
        else
        {
            actors = get_actors_in_type(_type);
            success = true;
        }
    }
    // UUID filter
    else if (!query.uuid.empty())
    {
        auto _uuid = query.uuid;
        if (_world_state.find(_uuid) == _world_state.end())
        {
            std::cout <<"Invalid UUID = " << _uuid << std::endl;
            success = false;
        }
        else
        {
            actors = get_actor_from_uuid(_uuid);
            success = true;
        }
    }
    // catch all
    else
    {
        std::cout <<"Bad Query to GET actors." << std::endl;
        success = false;
    }
    return true;
}


bool WorldStateGenerator::update_actors_poses_callback(std::vector<std::__cxx11::string> actor_uuids,
                                                       std::vector<geometry_msgs::PoseStamped> poses)
{
    if(actor_uuids.size() != poses.size())
    {
        std::cout <<"The sizes of actors uuids and the poses are not the same !!!!!" << std::endl;
        return false;
    }
    return update_actors_poses(actor_uuids, poses);
}

bool WorldStateGenerator::update_actors_callback(bpp_actor::ActorVec &actors)
{
    return update_actors(actors);
}

bool WorldStateGenerator::remove_actor_callback(const std::string actor_uuid)
{
    if (actor_uuid.empty())
    {
        std::cout <<"No UUID available for the Actor!!!"<< std::endl;
        return false;
    }
    return delete_actor(actor_uuid);
}

bool WorldStateGenerator::get_actor_pose_callback(const std::string actor_uuid, const std::string actor_name, const std::string pose_type, geometry_msgs::PoseStamped& pose)
{
    auto uuid = actor_uuid.empty() ? actor_name : actor_uuid;
    if (actor_uuid.empty() && actor_name.empty())
    {
        std::cout <<"No UUID/Name available for the Actor!!!"<< std::endl;
        return false;
    }
    else if (pose_type.empty())
    {
        std::cout <<"No pose_type mentioned for Actor: " << uuid<< std::endl;
        return false;
    }
    return get_actor_pose(uuid, pose_type, pose);

}


bool WorldStateGenerator::reset_callback()
{
    init_world_state();
    return true;
}

bool WorldStateGenerator::update_box_plan_callback(const std::vector<bpp_msgs::BoxPlan >& plan)
{
    box_plan_.clear();
    box_plan_ = plan;
    return true;
}

bool WorldStateGenerator::get_box_plan_callback(std::vector<bpp_msgs::BoxPlan> &plan)
{
    plan.clear();
    plan = box_plan_;
    return true;
}

//bool WorldStateGenerator::get_world_state_callback(util_msgs::GetWorldState::Request &request, util_msgs::GetWorldState::Response &response)
//{
//    /*
//    AssociationMap assc_map=get_association_map();
//    WorldStateMap world_state = get_register();
//    transform(world_state.begin(), world_state.end(), std::back_inserter(response.ws.actors.actors), [](const WorldStateMap::value_type& val){return val.second;} );
//    transform(assc_map.begin(), assc_map.end(), std::back_inserter(response.ws.assoications),
//        [](const AssociationMap::value_type& val)
//        {
//            bpp_actor::Association data;
//            data.parent_model_uuid=val.second.parent_model_uuid;
//            data.parent_body_uuid=val.second.parent_body_uuid;
//            data.child_model_uuid=val.second.child_model_uuid;
//            data.child_body_uuid=val.second.child_body_uuid;
//            data.parent_to_child_pose = val.second.parent_to_child_pose;
//            return data;
//        } );
//    response.ws.box_plan = box_plan_;
//*/
//    response.success = true;
//    return true;
//}

//bool WorldStateGenerator::set_world_state_callback(util_msgs::SetWorldState::Request &request, util_msgs::SetWorldState::Response &response)
//{
//    /*
//    WorldStateMap world_state;
//    for(bpp_actor::Actor &actor: request.ws.actors.actors)
//        world_state.insert(std::pair<std::string, bpp_actor::Actor> (actor.uuid, actor));
//    set_register(world_state);
//    AssociationMap assc_map;
//    for(bpp_actor::Association &assc: request.ws.assoications)
//    {
//        AssociationData data;
//        data.parent_model_uuid=assc.parent_model_uuid;
//        data.parent_body_uuid=assc.parent_body_uuid;
//        data.child_model_uuid=assc.child_model_uuid;
//        data.child_body_uuid=assc.child_body_uuid;
//        data.parent_to_child_pose = assc.parent_to_child_pose;
//        assc_map.insert(std::pair<std::string, AssociationData> (data.child_model_uuid, data));
//    }
//    set_association_map(assc_map);
//    box_plan_ = request.ws.box_plan;
//*/
//    response.success = true;
//    return true;
//}

void WorldStateGenerator::init_world_state()
{
    WorldStateMap _world_state;
    _world_state.clear();
    set_register(_world_state);

    /*
    AssociationMap assc_map;
    assc_map.clear();
    set_association_map(assc_map);
*/

    // init static actors if needed, exit if this step is unsuccessful.
    if (!static_actors_file_.empty())
    {
        if (!load_static_actors(static_actors_file_))
            std::cout << "Loading of STATIC actors unsuccessful!!!"<< std::endl;
    }
    else
    {
        std::cout << "No STATIC actors requested to load!!!"<< std::endl;
    }
    // clear box plan
    box_plan_.clear();
}


void WorldStateGenerator::set_register(const WorldStateMap &ws_reg)
{
    world_state_mutex_.lock();
    world_state_register_ = ws_reg;
    std::cout << stringify(world_state_register_) << std::endl;
    world_state_mutex_.unlock();
}


WorldStateMap WorldStateGenerator::get_register()
{
    world_state_mutex_.lock();
    auto _ws_reg = world_state_register_;
    world_state_mutex_.unlock();
    return _ws_reg;
}


bool WorldStateGenerator::add_actor(const bpp_actor::Actor &actor)
{
    CHECK_UUID_PRESENCE_1();

    auto _world_state = get_register();
    auto uuid = actor.uuid.empty() ? actor.semanticName : actor.uuid;
    // complain if the actor is already present...
    auto result = _world_state.find(uuid);
    if (result != _world_state.end())
    {
        std::cout << "You are trying to ADD an actor that's already present"<<std::endl;
        std::cout << "Actor UUID/Name = "<< uuid<<std::endl;
        return false;
    }
    else
    {
        _world_state.insert(std::make_pair(uuid, actor));
        set_register(_world_state);
        std::cout << "ADD actor successful."<< uuid<<std::endl;
        return true;
    }
}

bool WorldStateGenerator::uuidExists(const std::__cxx11::string &uuid)
{
    bool success=false;

    auto _world_state = get_register();
    auto _is_valid_filter = [](const std::vector<std::string> &filter_items,
            const std::string &filter_value)
    {
        return std::find(std::begin(filter_items),
                         std::end(filter_items), filter_value) != std::end(filter_items);
    };

    auto _uuid = uuid;
    if (_world_state.find(_uuid) == _world_state.end())
    {
        std::cout <<"Invalid UUID = "<< _uuid<<std::endl;
        success = false;
    }
    else
    {
        success = true;
    }

    return success;
}


bool WorldStateGenerator::delete_actor(const std::string &uuid)
{
    WorldStateMap _world_state = get_register();
    // complain if the actor is not present...
    auto result = _world_state.find(uuid);
    if (result == _world_state.end())
    {
        std::cout <<"You are trying to DELETE an actor that's not present"<< std::endl;
        std::cout <<"Actor UUID/Name = "<< uuid << std::endl;
        return false;
    }
    else
    {
        _world_state.erase(uuid);
        set_register(_world_state);
        std::cout <<"DELETE actor successful."<<std::endl;
        return true;
    }
}


bool WorldStateGenerator::update_actor(const bpp_actor::Actor &actor)
{
    CHECK_UUID_PRESENCE_1();

    auto _world_state = get_register();
    auto uuid = actor.uuid.empty() ? actor.semanticName : actor.uuid;
    // complain if the actor is not present...
    auto result = _world_state.find(uuid);
    if (result == _world_state.end())
    {
        std::cout <<"You are trying to UPDATE an actor that's not present"<<std::endl;
        std::cout <<"Actor UUID/Name = "<< uuid<<std::endl;
        return false;
    }
    //    auto past = result->second;
    // TODO Need to compute diff between past/current and update accordingly

    std::cout << "Updating actor -----> " << actor.uuid << std::endl;
    _world_state[uuid] = actor;
    set_register(_world_state);
    return true;
}

// update the current box poses
bool WorldStateGenerator::update_actors_poses(std::vector<std::string> &uuids, std::vector<geometry_msgs::PoseStamped> &poses)
{
    WorldStateMap _world_state = get_register();
    for(size_t i=0; i<uuids.size(); ++i)
    {
        auto result = _world_state.find(uuids[i]);
        if (result == _world_state.end())
        {
            std::cout <<"You are trying to UPDATE an actor that's not present"<<std::endl;
            std::cout <<"Actor UUID/Name = "<< uuids[i]<<std::endl;
            return false;
        }
        bpp_actor::Actor actor = _world_state[uuids[i]];
        //actor.header = poses[i].header;
        actor.targetPoseVec[0] = poses[i].pose;

        _world_state[uuids[i]] = actor;
    }
    set_register(_world_state);

    return true;
}

// update the current box poses
bool WorldStateGenerator::update_actors(const bpp_actor::ActorVec &actor_vec)
{
    WorldStateMap _world_state = get_register();
    for(size_t i=0; i<actor_vec.size(); ++i)
    {
        auto result = _world_state.find(actor_vec[i].uuid);
        if (result == _world_state.end())
        {
            std::cout <<"You are trying to UPDATE an actor that's not present"<<std::endl;
            std::cout <<"Actor UUID/Name = "<< actor_vec[i].uuid<<std::endl;
            return false;
        }
        _world_state[actor_vec[i].uuid] = actor_vec[i];
    }
    set_register(_world_state);
    return true;
}

bool WorldStateGenerator::get_actor_pose(const std::string &uuid, const std::string &pose_type,
                                         geometry_msgs::PoseStamped &pose)
{
    auto _world_state = get_register();
    // complain if the actor is not present...
    auto result = _world_state.find(uuid);
    if (result == _world_state.end())
    {
        std::cout <<"You are trying to GET POSE for an actor that's not present"<<std::endl;
        std::cout <<"Actor UUID/Name = "<< uuid<<std::endl;
        return false;
    }
    else
    {
        auto actor = _world_state[uuid];
//        pose.header = actor.header;
        if (pose_type == "target")
        {
            pose.pose = actor.targetPoseVec.front();
        }
        else if (pose_type == "desired")
        {
            pose.pose = actor.desiredPoseVec.front();
        }
        else if (pose_type == "place")
        {
            //orientation is the same as the box
            pose.pose = actor.desiredPoseVec.front();

            //get the grasp position wrt base frame: R*placePose.position + boxPose.position
            geometry_msgs::Pose boxPose = actor.desiredPoseVec.front();
            // actor.placePose is wrt to box center frame
            Eigen::Affine3d box_tr_wrt_base, gripper_tr_wrt_box, gripper_tr_wrt_base;
            tf::poseMsgToEigen(boxPose, box_tr_wrt_base);
            tf::poseMsgToEigen(actor.placePose, gripper_tr_wrt_box);

            gripper_tr_wrt_base = box_tr_wrt_base * gripper_tr_wrt_box;
            tf::poseEigenToMsg(gripper_tr_wrt_base, pose.pose);
        }
        else if (pose_type == "grasp")
        {
            //orientation is the same as the box
            pose.pose = actor.targetPoseVec.front();

            //get the grasp position wrt base frame: R*graspPose.position + boxPose.position
            geometry_msgs::Pose boxPose = actor.targetPoseVec.front();
            // actor.graspPose is wrt to box center frame
            Eigen::Affine3d box_tr_wrt_base, gripper_tr_wrt_box, gripper_tr_wrt_base;
            tf::poseMsgToEigen(boxPose, box_tr_wrt_base);
            tf::poseMsgToEigen(actor.graspPose, gripper_tr_wrt_box);

            gripper_tr_wrt_base = box_tr_wrt_base * gripper_tr_wrt_box;
            tf::poseEigenToMsg(gripper_tr_wrt_base, pose.pose);
        }
        else
        {
            std::cout <<"Pose type NOT VALID!!!"<<std::endl;
            std::cout <<"Pose type = "<< pose_type<<std::endl;
            return false;
        }
        return true;
    }
}


ActorVector WorldStateGenerator::get_actors_in_zone(const std::string &zone)
{
    auto _world_state = get_register();
    ActorVector actors;
    std::for_each(std::begin(_world_state),
                  std::end(_world_state),
                  [&](const WorldStateMap::value_type &item) {
        if (item.second.zone == zone)
            actors.push_back(item.second);
    } );
    return actors;
}


ActorVector WorldStateGenerator::get_actors_in_state(const std::string &state)
{
    auto _world_state = get_register();
    ActorVector actors;
    std::for_each(std::begin(_world_state),
                  std::end(_world_state),
                  [&](const WorldStateMap::value_type &item) {
        if (item.second.state == state)
            actors.push_back(item.second);
    } );
    return actors;
}


ActorVector WorldStateGenerator::get_actors_in_type(const std::string &type)
{
    auto _world_state = get_register();
    ActorVector actors;
    std::for_each(std::begin(_world_state),
                  std::end(_world_state),
                  [&](const WorldStateMap::value_type &item) {
        if (item.second.type == type)
            actors.push_back(item.second);
    } );
    return actors;
}


ActorVector WorldStateGenerator::get_actor_from_uuid(const std::string &uuid)
{
    auto _world_state = get_register();
    ActorVector actors;
    actors.push_back(_world_state[uuid]);
    return actors;
}

bool WorldStateGenerator::get_pose_from_body_uuid(const std::string &model_uuid, const std::string &body_uuid, geometry_msgs::Pose &pose)
{
    bool found=false;

    ActorVector actors=get_actor_from_uuid(model_uuid);

    if(actors.size()==1){
        for(size_t i=0;i<actors.back().bodies.size();++i){
            if(actors.back().bodies[i]==body_uuid){
                pose=actors.back().targetPoseVec[i];
                found=true;
                break;
            }
        }
    }
    else{
        std::cout <<"get_pose_from_body_uuid failed: actors.size()==1. Condition did not meet!"<<std::endl;
    }

    return found;
}


bool WorldStateGenerator::load_static_actors(const std::string& static_actors_file)
{
    TiXmlDocument doc(static_actors_file.c_str());
    // complain if file isn't loaded...
    if(!doc.LoadFile())
    {
        std::cout <<"Failed to load XML file with static actors."<<std::endl;
        std::cout <<"File = "<< static_actors_file<<std::endl;
        return false;
    }
    // complain if ROOT is something else...
    TiXmlElement *root = doc.RootElement();
    if(std::strcmp(root->Value(), "actors") != 0)
    {
        std::cout <<"ROOT value error. Allowed value is `actors`"<<std::endl;
        std::cout <<"Current ROOT = "<< root->Value()<<std::endl;
        return false;
    }
    for(TiXmlElement *actor = root->FirstChildElement(); actor; actor = actor->NextSiblingElement())
    {
        bpp_actor::Actor _actor;
        _actor.semanticName = actor->Attribute("name");
//        _actor.header.seq = 0;
//        _actor.header.stamp = ros::Time::now();
//        _actor.header.frame_id = actor->Attribute("parent");
        _actor.uuid = _actor.semanticName;
        _actor.type = "static";
        _actor.targetPoseVec.resize(1);
        _actor.targetPoseVec[0].position.x = std::atof(actor->Attribute("x"));
        _actor.targetPoseVec[0].position.y = std::atof(actor->Attribute("y"));
        _actor.targetPoseVec[0].position.z = std::atof(actor->Attribute("z"));
        _actor.targetPoseVec[0].orientation.x = std::atof(actor->Attribute("rot_x"));
        _actor.targetPoseVec[0].orientation.y = std::atof(actor->Attribute("rot_y"));
        _actor.targetPoseVec[0].orientation.z = std::atof(actor->Attribute("rot_z"));
        _actor.targetPoseVec[0].orientation.w = std::atof(actor->Attribute("rot_w"));
        _actor.desiredPoseVec.resize(_actor.targetPoseVec.size());
        _actor.desiredPoseVec[0] = _actor.targetPoseVec[0];
        _actor.bbox.x = std::atof(actor->Attribute("bbox_x"));
        _actor.bbox.y = std::atof(actor->Attribute("bbox_y"));
        _actor.bbox.z = std::atof(actor->Attribute("bbox_z"));
        if (add_actor(_actor))
        {
            std::cout <<"Successfully added STATIC actor = "<< _actor.uuid<<std::endl;
        }
        else
        {
            std::cout <<"Failed to add STATIC actor!!!"<<std::endl;
            std::cout <<"Actor = "<< _actor.uuid<<std::endl;
        }
    }
    return true;
}


};


