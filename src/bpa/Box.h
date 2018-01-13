/*
 * Box.h
 *
 *  Created on: August, 2015
 *      Author: Cai
 */

#ifndef BOX_H_
#define BOX_H_

#include <algorithm>
#include <iostream>
#include <queue>
#include "FittingPoint.h"
#include "Point.h"
#include <map>

#include "Parameters.h"


using namespace std;

//enum axis {NONE=0, X_AXIS=1,Y_AXIS=2,Z_AXIS=3, XY_AXIS=4, XYZ_AXIS=5};

namespace bpa
{
class Box
{
    public:

        /* \brief A Constructor with input parameters
         * \param name: a string value for the uuid of the box
         * \param frag: a boolean value to indicate if the box is fragile
         */
        Box(double length, double width, double height, double mass, std::string name, std::vector<std::string> labels, std::string material="carton", std::string tool="");

        //TODO: deprecated function, kept only to prevent API break
//        Box(double length, double width, double height, double mass, string name, bool frag);

        /*
         * \brief A Constructor
         */
        Box();

        /*
         * \brief A copy Constructor
         */
//        Box(const Box &box);

        virtual ~Box();

        /* To set the choose score of the box (to the own choose_score double variable)
         */
        void setChooseScore();

        /*
         * To get the volume of the box
         */
        double getVolume();

        /*
         * To get the ID of the box
         */
        int getId();

        /*
         * To estimate if the box equals another box
         */
        bool equalsBox(Box &abox);

        /*
         * To estimate if the passed integer value equals the Id of this box
         */
        bool equalsBox(int id);

        /* Comparision Operator.
         * Compare with the Box.id
         */
        bool operator==(const Box &other);

        /*
         * To estimate if a given box is listed as corresponding box in this box
         * \return A boolean value if the Id is listed as corresponding box in this box
         */
        bool hasCorrespondBox(int id);

        double m_length;
        double m_width;
        double m_height;
        double m_mass;
        int m_id;
        std::string m_name;   /* uuid Value of the Box */
        std::string material; /* carton(cardboard), wooden, styrofoam, plastic(drum) */

        Point position;  /* desired position of the box (with respect to the bin coordinate system)*/
        double rotation; /* Box desired rotation in radiants */
        Point center_of_mass; /* The center of mass of the Box with respect to the box coordinate system.*/

        bool is_rotated;
        bool is_simulated;

        std::vector<std::string> box_labels;
        bool is_fragile;
        bool is_dangerous;
        bool is_packed; /* if the box is already packed into the bin or just planned to give to the packing plan algorithm*/
        double choose_score;
        bool is_stackable;

        //the ids corresponding boxes (like a rotated or simulated box)
        std::vector<int> correspond_boxes;

        // Needed for the robot gripper in pplan :
        Point gripper_position;  /* Midpoint Position of the Gripper while gripping (w.r.t the box frame) */
        double gripper_orientation;
        string box_direction;
        std::string tool_name;
        double z_max;

        std::vector< FittingPoint > box_fps;  /* Fitting Points for each box */
        std::vector< FittingPoint > bin_fps;  /* Fitting Points for bin after packing this box*/

        std::vector<std::string> dependency;  /* For palletization orders: dependency are the boxes which block the way of this box*/

        std::vector<supportingBox> support_boxes; /* supporting boxes under */

        //box current pose in the world frame
        Eigen::Vector3d current_position;
        double current_rotation;
};

}

#endif /* BOX_H_ */
