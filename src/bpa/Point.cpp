/*
 * Point.cpp
 *
 *  Created on: August, 2015
 *      Author: Cai
 */

#include "bpa/Point.h"

namespace bpa
{

Point::Point(double x, double y, double z)
{
    this->position << x, y, z;
}

Point::Point(Eigen::Vector3d pos)
{
    this->position = pos;
}

Point::Point()
{
    this->position << 0.0, 0.0, 0.0;
}

Point::~Point()
{
}

}
