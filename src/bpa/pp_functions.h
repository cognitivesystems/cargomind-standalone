#ifndef PP_FUNCTIONS_H_
#define PP_FUNCTIONS_H_

#include <vector>
#include <bpa/Bin.h>
#include <bpa/Box.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/writer.h>
#include <iostream>
#include <fstream>

namespace bpa
{
    class PalletizationPlanner
    {
    public:

        PalletizationPlanner();
        /*
         * To get the correct Palletization order of boxes not marked as "packed" in a bin configuration (and setting them to "packed")
         * \param abin: A Bin object with the current bin configuration
         * \return Vector of Box objects; Order: vector<box>[0] has to be packed first and then so on
         */
        std::vector<Box> getPalletizationPlan(Bin &abin, std::vector<Box> &packing_plan);

        /*
         * To remove blocking constellations in the packing order, reorder
         * \param packing_plan:  Box indicating the current packing order
         * \param abin: A Bin indicating the current Bin configuration
         * \return A bool: True: All blocking constellations could be eliminated; False: still blocking constellations are existing
         */
        bool sortBoxesRemoveBlock(std::vector<Box> &packing_plan, Bin &abin);

        /// partially ordered set: consider the boxes orders according to tool change times
        std::vector<Box> getOrderdBoxSets(std::vector<Box> &bpp_boxes, Bin &abin);

        /*
         * To decide the robot move box direction
         * First check XY, then Z direction
         * TODO: conside the "tool_changer" and "box size"
         */
        std::vector<Box> getMoveDirection(std::vector<Box> &box_plan, Bin &abin);

        std::vector<Box> getGraspPose(std::vector<Box> &pp_boxplan, Bin &abin);

        /*
         * To check if a box blocks another box
         * All directions: X, Y, Z
         * TODO: add the box feature (mass, fragile...) to decide the orders
         */
        bool isBlocking(Box abox, Box b, Bin abin);

        /// is abox blocked in the x direction move
        bool xBlocking(Box abox, Box b, Bin abin);
        bool yBlocking(Box abox, Box b, Bin abin);
        bool zBlocking(Box abox, Box b, Bin abin);

        /*
         * To get the place position for boxes
         * TODO: depend on different grasp tools
         */
        void getPlacePositionVacumm(Box &box, Bin &abin);
        void getPlacePositionFork(Box &box, Bin &abin);

        /*
         * To compare the position_z + height of boxes (For the sort() algorithm of the List)
         * \return A boolean Value if the position_z + height value is higher
         */
        bool compareBoxHeightSide(const Box &first, const Box &second);

        bool loadParamsFromFile(std::string filename);

        double vacuum_length; // 300mm, length of the Gripper
        double vacuum_width; // 300mm, length of the Gripper
        double vacuum_height;

        double fork_l;
        double fork_w1;
        double fork_w2;
        double fork_h;

        double CLEARANCE;    // 50cm;  The minimum space between the Bin height and the ground surface of the gripper at its highest position
        double current_bin_height;   // the hightest point of current bin

    };

}

#endif
