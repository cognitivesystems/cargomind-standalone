/*
 * FittingPoint.h
 *
 *  Created on: August, 2015
 *      Author: Cai
 */

#ifndef FITTINGPOINT_H_
#define FITTINGPOINT_H_

#include "Point.h"
#include "Parameters.h"
#include <vector>

namespace bpa
{

class FittingPoint
{
public:

     /* \brief A Constructor
     */
    FittingPoint();

    /*
	 * \brief A Constructor with input parameters
     * \param q: Quadrant of the Fitting Point
	 */
    FittingPoint(double xi, double yi, double zi, int q);

    /*
	 * \brief A Constructor with input parameters
     * \param point: Position of the Fitting Point in the bin
	 */
    FittingPoint(Point point, int q);

    /*
     * \brief A copy Constructor
     * \param fp: object to be copied
     */
//    FittingPoint(const FittingPoint &fp);

    // \brief Destructor
    virtual ~FittingPoint();

    /*
	 * To estimate if a Fitting_Point is equal to this one
	 */
    bool equals(FittingPoint &fp);

    /* Comparision Operator.
	 * \param other: a const Fitting_Point Object
	 */
    bool operator==(const FittingPoint &other);

    /*
     * To compare the own score value with the score value of another Fitting Point object
     * \param first: A const Fitting_Point object
     * \return A boolean Value if the Score of the other Fitting Point is higher
     */
    bool hasLowerScoreThan(const FittingPoint &other);


    Point coordinates;  /* Position of the Fitting Point in the Bin (w.r.t the frame of the Bin) */
    int quadrant;       /* Edge_Quadrant of the Fitting Point */
    double score;       /* Score of a Box-Fitting Point combination (only used when this Fitting Point is stored in a Box) */
    double temp_a_helt; /* The supported ground surface (used in the Deep Search Algorithms) */

    Eigen::Vector3d direction_x, direction_y, direction_z;  // For calculate three corner point, according to fp.quadrant
    Eigen::Vector3d direction_box_pos;  // Get the new box original position, according to fp.quadrant

};

}

#endif /* FITTINGPOINT_H_ */
