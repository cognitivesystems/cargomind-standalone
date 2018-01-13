/*
 * FittingPoint.cpp
 *
 *  Created on: August, 2015
 *      Author: Cai
 */

#include "bpa/FittingPoint.h"

namespace bpa
{

FittingPoint::FittingPoint(double xi, double yi, double zi, int q)
{
    coordinates.position << xi, yi, zi;
    quadrant = q;
	score = 0;
	temp_a_helt = 0;

    switch (q)
    {
    case 1:
        direction_x << 1,0,0;
        direction_y << 0,1,0;
        direction_z << 0,0,1;
        direction_box_pos << 0,0,0;
        break;
    case 2:
        direction_x << 1,0,0;
        direction_y << 0,-1,0;
        direction_z << 0,0,1;
        direction_box_pos << 0,-1,0;
        break;
    case 3:
        direction_x << -1,0,0;
        direction_y << 0,-1,0;
        direction_z << 0,0,1;
        direction_box_pos << -1,-1,0;
        break;
    case 4:
        direction_x << -1,0,0;
        direction_y << 0,1,0;
        direction_z << 0,0,1;
        direction_box_pos << -1,0,0;
        break;
    default:
        break;
    }
}

FittingPoint::FittingPoint(Point point, int q)
{
    coordinates = point;
	quadrant = q;
	score = 0;
	temp_a_helt = 0;

    switch (q)
    {
    case 1:
        direction_x << 1,0,0;
        direction_y << 0,1,0;
        direction_z << 0,0,1;
        direction_box_pos << 0,0,0;
        break;
    case 2:
        direction_x << 1,0,0;
        direction_y << 0,-1,0;
        direction_z << 0,0,1;
        direction_box_pos << 0,-1,0;
        break;
    case 3:
        direction_x << -1,0,0;
        direction_y << 0,-1,0;
        direction_z << 0,0,1;
        direction_box_pos << -1,-1,0;
        break;
    case 4:
        direction_x << -1,0,0;
        direction_y << 0,1,0;
        direction_z << 0,0,1;
        direction_box_pos << -1,0,0;
        break;
    default:
        break;
    }

}

//????
FittingPoint::FittingPoint()
{
    coordinates.position << 0.0, 0.0, 0.0;
    quadrant = 1;
    score = 0;
    temp_a_helt = 0;

    direction_x << 1,0,0;
    direction_y << 0,1,0;
    direction_z << 0,0,1;
    direction_box_pos << 0.0, 0.0, 0.0;
}

//FittingPoint::FittingPoint(const FittingPoint &fp)
//{
//    coordinates.pos_x = fp.coordinates.pos_x;
//    coordinates.pos_y = fp.coordinates.pos_y;
//    coordinates.pos_z = fp.coordinates.pos_z;
//    quadrant = fp.quadrant;
//    score = fp.score;
//    temp_a_helt = fp.temp_a_helt;
//}

FittingPoint::~FittingPoint()
{
}

bool FittingPoint::equals(FittingPoint &fp)
{
    return ( floatEqual(this->coordinates.position(0), fp.coordinates.position(0))
            && floatEqual(this->coordinates.position(1) , fp.coordinates.position(1))
            && floatEqual(this->coordinates.position(2) , fp.coordinates.position(2))
            && (this->quadrant == fp.quadrant) );
}

bool FittingPoint::operator==(const FittingPoint &other)
{
    return ( floatEqual(this->coordinates.position(0), other.coordinates.position(0))
            && floatEqual(this->coordinates.position(1) , other.coordinates.position(1))
            && floatEqual(this->coordinates.position(2) , other.coordinates.position(2))
            && (this->quadrant == other.quadrant) );
}

bool FittingPoint::hasLowerScoreThan(const FittingPoint &other)
{
    return this->score < other.score;
}

}
