#ifndef BPP_ROS_H
#define BPP_ROS_H


#include <iostream>
#include <iomanip>
#include <bullet/Actor.h>
#include <bpa/bpp_functions.h>
#include <bpa/pp_functions.h>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/writer.h>


namespace bpa_ros
{
    /**
     * \class   BppROS
     */
    class BppROS
    {
        public:
            // \brief  Constructor
            BppROS();
            // \brief  Destructor
            ~BppROS();

            // \brief  Callback function for add_actor service
            bool bin_packing_boxes_callback(const std::string& bin_name, const std::string& opt_name, bpp_actor::ActorVec& boxes_to_pack);

            void loadParamsJSON(std::string& file_name);

            void computePalletFP(bpp_actor::Actor &pallet, std::vector<bpp_actor::Actor> &packed_boxes);
            bool isFPEqual(bpp_actor::FittingPoint &fp, std::vector<bpp_actor::FittingPoint> &bin_fps, int &id);
            void removeFittingpoint(bpp_actor::FittingPoint &fp, std::vector<bpp_actor::FittingPoint> &bin_fps);

            bool updateBoxPlanToWSG(const std::vector<bpp_msgs::BoxPlan>& box_plan);

    private:
            /** *** member functions *****/
            // \brief  Function to palletise boxes.
            std::vector<bpp_actor::Actor> binPackingBoxes();

            // actors from outside ---> boxes
            std::vector<bpa::Box> actorsToBoxes(const std::vector<bpp_actor::Actor>& actors);
            bpa::Box actorToBox(const bpp_actor::Actor& actor);
            std::vector<bpa::FittingPoint> fpsWrtBppFrame(const std::vector<bpp_actor::FittingPoint>& fitting_points);

            //boxes ---> boxPlan for bpp frame
            std::vector<bpp_msgs::BoxPlan> boxesToBoxPlan(const std::vector<bpa::Box>& boxes);
            bpp_msgs::BoxPlan boxToBoxPlan(const bpa::Box& box);

            //boxes ---> actors for outside
            std::vector<bpp_actor::Actor> boxesToActors(const std::vector<bpa::Box>& boxes);
            bpp_actor::Actor boxToActor(const bpa::Box& box);

            // update the bpp results to wsg
            bool updateBinFPsToWSG(const std::vector<bpa::FittingPoint>& fitting_points, bool intial);
            bool updateActorsToWSG(const std::vector<bpp_actor::Actor>& actors);

            /** *** private members *****/
            std::map<std::string, geometry_msgs::PoseStamped> boxes_pose_table;
            double pallet_length;
            double pallet_width;
            double pallet_height;
            std::string pallet_frame_id;
            double bbox_offset;

            // \brief tranform from bpp pallet frame to world pallete frame
            Eigen::Affine3d pallet_tr_bpp_to_world;
            Eigen::Vector3d pallet_center;

            std::string parameter_file;


    };
}

#endif  // BPP_ROS_H
