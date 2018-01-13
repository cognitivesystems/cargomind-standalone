#ifndef ACTOR_H_
#define ACTOR_H_

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

using namespace std;

namespace geometry_msgs{

struct Vector3{
    double x;
    double y;
    double z;
};

struct Point{
    double x;
    double y;
    double z;
};

struct Quaternion{
    double x;
    double y;
    double z;
    double w;
};

struct Pose{
    Point position;
    Quaternion orientation;
};

struct Header{
    uint seq;
    string frame_id;
};

struct PoseStamped{
    Pose pose;
};
}

namespace bpp_msgs{

struct ActorUIDs{
    vector<string > uid;
};

struct BoxPlan{
    // UUID of the box
    string box_uuid;

    // the pose of the box in the bpp pallet frame
    geometry_msgs::Pose box_pose_in_pallet;

    // if the box is rotated around z 90 degree
    bool is_rotated;

    // bounding box of the actor
    geometry_msgs::Vector3 bbox;
    float box_mass;

    // current state, packed or pallitized
    string box_state;

    // Material of the actor (cargo package)
    // (values: carton, wooden, styrofoam, plastic.)
    string material;

    // Indicate if the actor is fragile or not
    // (values: fragile, dangerous, none, etc.)
    vector<string > labels;

    string robot_tool;
};

}

namespace bpp_actor{

struct FittingPoint{
    // position of the fp
    geometry_msgs::Point position;
    int32_t quadrant;
    double score;
    double temp_a_helt;
};

struct SupportBox{
    string boxID;
    double helt;
};

struct RectLabel{
    string label_string;
    int label;
    int x;
    int y;
    int w;
    int h;
};

typedef vector<RectLabel > RectLabelVec;

struct Actor{
    // UUID of the actor
    string uuid;

    // location of the box
    string storage_location;

    // equivalent to UUID - using either of them is enough
    string semanticName;

    // Current zone of the actor
    // (values: detection_area, measurement_area, holding_area, pallet, etc.)
    string zone;

    // Current state of the actor
    // (values: to_pack, planed, palletized, packed)
    string state;

    // To indicate the actor type eg. box
    // (values: box, static, etc.)
    string type;

    // To hold model bodies
    vector<string > bodies;
    vector<string > body_urls;
    vector<string > body_collision_urls;
    vector<string > body_texture_urls;
    vector<float > params;

    // bounding box of the actor
    geometry_msgs::Vector3 bbox;

    // Weight of the actor (cargo package)
    float weight;
    geometry_msgs::Vector3 cog;

    // Material of the actor (cargo package)
    // (values: carton, wooden, styrofoam, plastic.)
    string material;

    // Indicate if the actor is fragile or not
    // (values: fragile, dangerous, none, etc.)
    vector<string > labels;

    // supporting boxes of the actor
    vector<SupportBox > supportBoxes;

    bool fragile;

    // current pose of the actor in the world
    vector<geometry_msgs::Pose > targetPoseVec;

    // desired pose of the actor on pallet
    vector<geometry_msgs::Pose > desiredPoseVec;

    // test for dpp
    vector<string > dependency_x;
    vector<string > dependency_y;
    vector<string > dependency_z;

    // maybe change name to grasp_location
    geometry_msgs::Pose graspPose;
    geometry_msgs::Pose placePose;
    string robotTool;
    string forkLeftTine;
    string forkRightTine;
    string boxDirection;

    // TODO add some description
    // maybe make it int32[] since all actors will have a UUID
    string associatedActor;

    // TODO add some description
    string modelPath;
    bool isArticulated;
    string kinematicModelPath;

    // Add fitting points for actor (box or pallet) in bpp
    vector<FittingPoint > fittingPoints;
    // fitting points for pallet for execution
    vector<FittingPoint > binFPs;

    // Perception Requirements

    vector<string > facesPath;
    vector<RectLabelVec > rectLabelVecs;
    string featurePointsPath;
    string objectPointsPath;
    string measureDebugPath;
    string meshPath;
    string thumbnailPath;
    string markerId;
    string barcode;

    int32_t actor_robot_tool_id_offline;
};

typedef vector<Actor > ActorVec;

struct QueryActor{
    //# Set to true to return all actors in the world
    bool all;

    //# Filter according to zone
    string zone;

    //# Filter according to state
    string state;

    //# Filter according to type
    string type;

    //# Filter according to UUID
    //# return one and only one if it's present
    string uuid;
};

}

namespace tf{
 inline void 	poseMsgToEigen (const geometry_msgs::Pose &m, Eigen::Affine3d &e){

     e = Eigen::Translation3d(m.position.x,
                                   m.position.y,
                                   m.position.z) *
            Eigen::Quaterniond(m.orientation.w,
                               m.orientation.x,
                               m.orientation.y,
                               m.orientation.z);
//    std::cout << "poseMsgToEigen not yet implemented" << std::endl;

//    throw;
}

 inline void 	poseEigenToMsg (const Eigen::Affine3d &e, geometry_msgs::Pose &m){
    std::cout << "poseEigenToMsg not yet implemented" << std::endl;
    throw;
}

 inline void 	pointMsgToEigen (const geometry_msgs::Point &m, Eigen::Vector3d &e){
    std::cout << "pointMsgToEigen not yet implemented" << std::endl;
    throw;
}

 inline void 	pointEigenToMsg (const Eigen::Vector3d &e, geometry_msgs::Point &m){
    std::cout << "pointEigenToMsg not yet implemented" << std::endl;
    throw;
}

//void quaternionMsgToTF (const geometry_msgs::Quaternion &msg, Quaternion &bt){
//    std::cout << "quaternionMsgToTF not yet implemented" << std::endl;
//    throw;
//}
};

namespace gui_msgs{
struct Progress{
    int id;
    uint progress;
    string text;
};

struct WSGUIStateFeedback{
    string goal_state;

    string final_state;
    bpp_actor::Actor final_actor;

    string current_state;
    bpp_actor::Actor current_actor;
};

};

#endif /* ACTOR_H_ */

