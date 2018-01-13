#ifndef WSG_UTILS_H
#define WSG_UTILS_H

#ifndef Q_MOC_RUN
#include <map>
#include <string>
#include <vector>

#include <bullet/Actor.h>

#endif
 
namespace wsg
{
    /// \brief  typedef for convenience
    typedef std::map<std::string, bpp_actor::Actor> WorldStateMap;
    /// \brief struct to hold Association data
    struct AssociationData{
        std::string parent_model_uuid;
        std::string parent_body_uuid;
        std::string child_model_uuid;
        std::string child_body_uuid;
        geometry_msgs::Pose parent_to_child_pose;

    };
    /// \brief  typedef for association map
    typedef std::map<std::string, AssociationData> AssociationMap;
    typedef std::vector<bpp_actor::Actor> ActorVector;
    /// \brief  Function to get the register entities as a list (std::vector). In
    ///         other words convert from `WorldStateMap` to `ActorVec`.
    /// \param  items: All the actors from the register that should be included in
    ///                the list
    /// \return `ActorVec` containing all the actors present in the register
    inline ActorVector get_register_as_list(const WorldStateMap &items){

            ActorVector actors;
            actors.reserve(items.size());
            std::transform(std::begin(items),
                    std::end(items),
                    std::back_inserter(actors),
                    [](const WorldStateMap::value_type &item) { return item.second; } );
            return actors;
    }

    /// \brief  Function to string-ify `bpp_actor::Actor`
    /// \param  actor: an Actor
    /// \return `std::string` of the actors
    std::string stringify(const bpp_actor::Actor &actor);
    /// \brief  Function to string-ify `ActorVec`
    /// \param  actors: list containing the actors
    /// \return `std::string` of the actors
    std::string stringify(const ActorVector &actors);
    /// \brief  Function to string-ify `WorldStateMap`
    /// \param  items: All the actors from the register that should be string-ified
    /// \return `std::string` of the actors
    std::string stringify(const WorldStateMap &items);
    /// \brief  Implementation function to string-ify an Actor. NOT TO BE USED EXTERNALLY.
    inline std::string __stringify_actor__(const bpp_actor::Actor &actor)
    {
        auto _pose = actor.targetPoseVec[0];
        std::ostringstream buf;
        buf << "{"
            << "uuid:" << actor.uuid << ", "
            << "position:" << "["
            << _pose.position.x << ", "
            << _pose.position.y << ", "
            << _pose.position.z << "], "
            << "orientation:" << "["
            << _pose.orientation.x << ", "
            << _pose.orientation.y << ", "
            << _pose.orientation.z << ", "
            << _pose.orientation.w << "]"
            << "}" << std::endl;
        return buf.str();
    }
};

#endif  // WSG_UTILS_H
