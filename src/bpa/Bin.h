/*
 * Bin.h
 *
 *  Created on: August, 2015
 *      Author: Cai
 */

#ifndef BIN_H_
#define BIN_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>
#include "Box.h"
#include "FittingPoint.h"
#include "HoldingPlatform.h"
#include "SimulatedBox.h"

#include <bullet/PhysicsBullet.h>

using namespace std;

namespace bpa
{

class Bin
{
public:
    /*
	 * \brief A Constructor with input parameters
     * \modus: a boolean value for the initial modus (true = Start with all four edges as Fitting Point, false = Start only with one initial Fitting Point)
	 */
    Bin(double length, double width, double height, bool modus);

    // \brief Destructor
	virtual ~Bin();

    /*
     * To copy the data from another bin into this bin
     * \param other: A bin object from which the data should be copied
     */
    void copyData(Bin &other);

    /*
     * To add a new box into the bin
     * \param helt: the surface of the new box which is supported by other boxes or the pallet ground
     */
    void addNewBox(Box &new_box, FittingPoint fp, HoldingPlatform hold, double helt);
    /// check if the new box can be moved more toward -x, -y
    Eigen::Vector3d checkNewBoxPosition(Box abox, FittingPoint fp);
    Eigen::Vector3d pushBoxTowardBinCenter(Box abox, FittingPoint fp, double helt);

    /* There are some boxes already in the bin, then bpp next group of boxes
     * Set the all packed boxes and bin fitting points for the bin
     * \param boxes_on_pallet: packed boxes in bin
     * \param fps_on_pallet: fitting points for current bin
     */
    void setPackedBoxes(std::vector<Box> &boxes_on_bin);
    void setBinFittingPoints(std::vector<FittingPoint> &fps_on_bin);

    /* There are some boxes already in the bin, then bpp next group of boxes
     * Update the boxes for current boxes plan in bin
     * \param boxes_on_pallet: packed boxes in current configuration
	 */
    std::vector<Box> getStepPackedBoxes();
    void updateBoxesPosition(std::vector<Box> &boxes_on_pallet);

    /* actualize_fittingpoints
     * To update new Fitting Points after adding a new Box
	 * \param abox: A Box object which has been packed into the bin
	 * \param fp: A Fitting_Point object which indicates the Point where the box has been packed
	 */
    void updateFittingPoints(Box &abox,FittingPoint &fp);

    /*
     * To remove a Fitting Point from the Set of Fitting Points
     * \param fp: A Fitting_Point which has to be removed from the Fitting Point Set
     */
    void removeFittingpoint(FittingPoint &fp, std::vector<FittingPoint> &fitting_points);

    /*
	 * To get the affection for the center of mass of a box placement in x-direction
	 * \param abox: A Box object which would be packed into the Bin
	 * \param fp: A Fitting_Point object which indicates the Point where the box would be packed
     * \return the deviation of the new center of mass to the target
     * compared to the deviation of the old center of mass to the target
	 */
    double getComDiffX(Box &abox, FittingPoint &fp);
    double getComDiffY(Box &abox, FittingPoint &fp);
    double getComDiffZ(Box &abox, FittingPoint &fp);

    /*
	 * To get the average height of boxes in the bin
     * \return the average height of the boxes in the bin
	 */
    double giveAverageHeightOfBoxes();

    /*
     * To get the nearest distance from a box to borders of the bin
	 * \param abox: A Box object which would be packed into the Bin
	 * \param fp: A Fitting_Point object which indicates the Point where the box would be packed
     * \return the shortes distance from the box to the closest border of the bin
	 */
    double getNearestDistanceToEdges(Box &abox, FittingPoint &fp);

    /*
     * Get a projection point along the direction using bullet ray casting
     */
    Eigen::Vector3d getProjection(Eigen::Vector3d point, Eigen::Vector3d direction);
    Eigen::Vector3d getProjectionVector(Eigen::Vector3d point, Eigen::Vector3d direction);
    bool isColliding(bpa::Box &box);

    double bin_length;
    double bin_width;
    double bin_height;
    double bin_mass;      /* the mass of all boxes packed into the bin */
    Point bin_com;        /* the center of mass of the bin (with respect to the bin coordinate system) */
    Point target_com;     /* the target center of mass of the bin (with respect to the bin coordinate system) */
    double acc_height_of_all_boxes; /* the cumulated height of all boxes packed into the bin */

    std::vector<Box> packed_boxes;  /* all boxes packed into the bin */

    double boxes_volume;             /* the volume of boxes packed into the bin */
    double boxes_ground_surface;     /* the ground surface of boxes packed into the bin */
    double boxes_support_surface;    /* the supported ground surface of boxes packed into the bin */

    std::vector<FittingPoint> fitting_points;  /* all Fitting Points of the bin */
    std::vector<FittingPoint> packed_boxes_fps; /* The origin points of the packed boxes */

    PhysicsBullet *bulletPhysics;

};

}

#endif /* BIN_H_ */
