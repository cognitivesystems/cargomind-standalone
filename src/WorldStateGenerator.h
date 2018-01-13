#ifndef WORLD_STATE_GENERATOR_H
#define WORLD_STATE_GENERATOR_H

#include <mutex>
#include <thread>

#include <tinyxml.h>

#include "wsg_utils.h"
 
namespace wsg
{
    /**
     * \class   WorldStateGenerator
     */
    class WorldStateGenerator
    {
        public:
            /// \brief  Constructor
            WorldStateGenerator();
            /// \brief  Destructor
            ~WorldStateGenerator();
            /// \brief  Callback function for add_actor service
            bool get_actor_callback(bpp_actor::QueryActor& query, std::vector<bpp_actor::Actor >& actors);
            /// \brief  Callback function for update_actors service
            bool update_actors_callback(bpp_actor::ActorVec& actors);
            bool update_actors_poses_callback(std::vector<std::string > actor_uuids, std::vector<geometry_msgs::PoseStamped > poses);
            /// \brief  Callback function for remove_actor service
            bool remove_actor_callback(const std::string actor_uuid);
            /// \brief  Callback function for get_actor_pose service
            bool get_actor_pose_callback(const std::string actor_uuid, const std::string actor_name, const std::string pose_type, geometry_msgs::PoseStamped& pose);
            /// \brief  Callback function for reset service
            bool reset_callback();
            /// \brief  Callback function for update_box_plan service
            bool update_box_plan_callback(const std::vector<bpp_msgs::BoxPlan >& plan);
            /// \brief  Callback function for get_box_plan service
            bool get_box_plan_callback(std::vector<bpp_msgs::BoxPlan >& plan);
//            bool get_world_state_callback(util_msgs::GetWorldState::Request& request,
//                                              util_msgs::GetWorldState::Response& response);
//            bool set_world_state_callback(util_msgs::SetWorldState::Request& request,
//                                              util_msgs::SetWorldState::Response& response);
            /// \brief  Function to set world_state_register_
            void set_register(const WorldStateMap &ws_reg);
            /// \brief  Function to get world_state_register_
            WorldStateMap get_register();
            /// \brief  Function to broadcast tf in a thread
            void publish_tf();

    public:
            /// \brief  Function to initialise the world state and load the static actors
            void init_world_state();
            /// \brief  Function to add actor into the register
            /// \param  actor: An actor object
            /// \return true if successful and false otherwise
            bool add_actor(const bpp_actor::Actor &actor);

            bool uuidExists(const std::string& uuid);
            /// \brief  Function to delete actor from the register
            /// \param  actor: An actor object
            /// \return true if successful and false otherwise
            bool delete_actor(const std::string &uuid);
            /// \brief  Function to update actor in the register
            /// \param  actor: An actor object
            /// \return true if successful and false otherwise
            bool update_actor(const bpp_actor::Actor &actor);  \
            // only a set of actors together
            bool update_actors(const bpp_actor::ActorVec &actor_vec);
            // only update the actors poses
            bool update_actors_poses(std::vector<std::string> &uuids, std::vector<geometry_msgs::PoseStamped> &poses);
            /// \brief  Function to get the pose of an actor
            /// \param  uuid: UUID/Name of the actor (key to the register)
            /// \param  pose_type: Pose type - values = `target`, `grasp`
            /// \return The pose value as `geometry_msgs::PoseStamped` object
            /// \return Also true if successful and false otherwise
            /// \note   In the case of target pose this function returns the very first pose
            ///         (`front()`) in the list.
            bool get_actor_pose(const std::string &uuid, const std::string &pose_type,
                    geometry_msgs::PoseStamped &pose);
            /// \brief  Function to get actors in a particular zone
            /// \param  zone: Zone name
            /// \return `ActorVec` containing all the actors present in a particular zone
            ActorVector get_actors_in_zone(const std::string &zone);
            /// \brief  Function to get actors in a particular state
            /// \param  state: State name
            /// \return `ActorVec` containing all the actors present in a particular state
            ActorVector get_actors_in_state(const std::string &state);
            /// \brief  Function to get actors in a particular type
            /// \param  type: Type name
            /// \return `ActorVec` containing all the actors present of a particular type
            ActorVector get_actors_in_type(const std::string &type);
            /// \brief  Function to get an actor by specifying its UUID
            /// \param  UUID of the actor
            /// \return `ActorVec` containing (only one) the actor
            ActorVector get_actor_from_uuid(const std::string &uuid);
            /// \brief  Function to get an body pose of an actor
            /// \param  UUID of the actor
            /// \param  UUID of body of the actor
            /// \param `geometry_msgs::Pose` pose
            /// \return `bool` found
            bool get_pose_from_body_uuid(const std::string &model_uuid, const std::string &body_uuid, geometry_msgs::Pose& pose);
            /// \brief  Function to load static actors from a XML file and set up static
            ///         transform publisher
            /// \param  Path of the XML file to load static actors from
            /// \return true if successful and false otherwise
            bool load_static_actors(const std::string &static_actors_file);
            /*************************************/
            /// \brief  Register to maintain the current state of the world. key = UUID of
            ///         the actor.
            WorldStateMap world_state_register_;
            /// \brief  Mutex to lock and unlock operations on world_state_register_
            std::mutex world_state_mutex_;
            /// \brief  Thread control flag
            bool stop_thread_flag_ = false;
            /// \brief  Register to maintain the current palletisation plan (box plan)
            std::vector<bpp_msgs::BoxPlan> box_plan_;
            /*************************************/
            /// \brief  Parameter to indicate the list of valid zones.
            std::vector<std::string> valid_zones_;
            /// \brief  Parameter to indicate the list of valid states.
            std::vector<std::string> valid_states_;
            /// \brief  Parameter to indicate the list of valid types.
            std::vector<std::string> valid_types_;
            /// \brief  Parameter variable to indicate if to use simulation or not
            bool use_simulation_;
            /// \brief  Parameter to specify the path of XML file containing the static
            ///         actors
            std::string static_actors_file_;
            /// \brief  Parameter to specify the name of the world frame
            std::string world_frame_name_;
            /*************************************/
            /// \brief  Node handle
    };
};

#endif  // WORLD_STATE_GENERATOR_H
