/*
 * Point.h
 *
 *  Created on: August, 2015
 *      Author: Cai
 */

#ifndef POINT_H_
#define POINT_H_

#include <eigen3/Eigen/Dense>
#include <iostream>

namespace bpa
{

class Point
{
public:
    /*
	 * \brief A Constructor
	 * A default constructor of Point
	 */
	Point();

    /*
	 * \brief A Constructor with input parameters
	 * \param x: a double value of the position in the x coordinate
	 * \param y: a double value of the position in the y coordinate
	 * \param z: a double value of the position in the z coordinate
	 */
    Point(double x, double y, double z);

    Point(Eigen::Vector3d pos);

    // \brief Destructor
	virtual ~Point();

    Eigen::Vector3d position;
};

}

#endif /*  POINT_H_ */
