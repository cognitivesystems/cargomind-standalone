/*
 * bin.cpp
 *
 *  Created on: August, 2015
 *      Author: Cai
 */

#include "bpa/Bin.h"
//#define DEBUG_PRINT

namespace bpa
{

Bin::Bin(double length, double width, double height, bool modus)
{
    bin_length = length;
    bin_width = width;
    bin_height = height;
    bin_mass = 0.0;
    acc_height_of_all_boxes = 0.0;
    boxes_volume = 0.0;
    boxes_ground_surface = 0.0;
    boxes_support_surface = 0.0;

    FittingPoint b(bin_length,0,0,4);
    fitting_points.push_back(b);
    if(modus)
    {
        FittingPoint a(0, 0, 0, 1);
        fitting_points.push_back(a);
        FittingPoint c(0,bin_width,0,2);
        fitting_points.push_back(c);
        FittingPoint d(bin_length,bin_width,0,3);
        fitting_points.push_back(d);
    }

    //start with center
//    FittingPoint a(bin_length/ 2.0, bin_width/ 2.0 ,0 ,1);
//    fitting_points.push_back(a);
//    FittingPoint b(bin_length/ 2.0, bin_width/ 2.0 ,0 ,2);
//    fitting_points.push_back(b);
//    FittingPoint c(bin_length/ 2.0, bin_width/ 2.0 ,0 ,3);
//    fitting_points.push_back(c);
//    FittingPoint d(bin_length/ 2.0, bin_width/ 2.0 ,0 ,4);
//    fitting_points.push_back(d);

    bin_com.position << 0.0, 0.0, 0.0;
    target_com.position << bin_length / 1.0, bin_width / 1.0, bin_height / 3.0;

    bulletPhysics = new PhysicsBullet();
}

Bin::~Bin()
{
}

void Bin::copyData(Bin &other)
{
	bin_length = other.bin_length;
	bin_width = other.bin_width;
	bin_height = other.bin_height;
    bin_mass = other.bin_mass;
	acc_height_of_all_boxes = other.acc_height_of_all_boxes;

    boxes_ground_surface = other.boxes_ground_surface;
	boxes_volume = other.boxes_volume;
    boxes_support_surface = other.boxes_support_surface;

    target_com.position = other.target_com.position;
    bin_com.position = other.bin_com.position;

	packed_boxes.clear();
    for(Box b:other.packed_boxes)
    {
		packed_boxes.push_back(b);
	}

    fitting_points.clear();
    for(FittingPoint fp:other.fitting_points)
    {
        fitting_points.push_back(fp);
    }

    packed_boxes_fps.clear();
    for(FittingPoint fp:other.packed_boxes_fps)
    {
        packed_boxes_fps.push_back(fp);
    }
}

void Bin::addNewBox(Box &new_box, FittingPoint fp, HoldingPlatform hold, double helt)
{
    if(hold.isSimulatedBox(new_box.getId()))
    {
        // Assumption: SimBoxes are always supported by 100% of its ground surface
        SIMULATED_BOX sim_box = hold.giveSimulatedBox(new_box.getId());
        Box temp(0,0,0,0,"",std::vector<std::string>());

        switch(fp.quadrant)
        {
            case 1: switch(sim_box.arrangement)
            {
                    case TOP: 	temp = hold.giveBox(sim_box.id1);
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                                fp.coordinates.position(1) = fp.coordinates.position(1) + temp.m_width;
                                temp = hold.giveBox(sim_box.id2);
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                        break;
                    case RIGHT: temp = hold.giveBox(sim_box.id1);
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                                fp.coordinates.position(0) = fp.coordinates.position(0) + temp.m_length;
                                temp = hold.giveBox(sim_box.id2);
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                        break;
                    default: cout << "Error in add simulate boxes!";
            }
            break;
            case 2: switch(sim_box.arrangement)
            {
                    case TOP: 	temp = (hold.giveBox(sim_box.id2));
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                                fp.coordinates.position(1) = fp.coordinates.position(1) - temp.m_width;
                                temp = (hold.giveBox(sim_box.id1));
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                        break;
                    case RIGHT: temp = (hold.giveBox(sim_box.id1));
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                                fp.coordinates.position(0) = fp.coordinates.position(0) + temp.m_length;
                                temp = (hold.giveBox(sim_box.id2));
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                        break;
                    default: cout << "Error in add simulate boxes!";
            }
            break;
            case 3: switch(sim_box.arrangement)
            {
                    case TOP: 	temp = (hold.giveBox(sim_box.id2));
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                                fp.coordinates.position(1) = fp.coordinates.position(1) - temp.m_width;
                                temp = (hold.giveBox(sim_box.id1));
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                        break;
                    case RIGHT: temp = (hold.giveBox(sim_box.id2));
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                                fp.coordinates.position(0) = fp.coordinates.position(0) - temp.m_length;
                                temp = (hold.giveBox(sim_box.id1));
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                        break;
                    default: cout << "Error in add simulate boxes!";
            }
            break;
            case 4: switch(sim_box.arrangement)
            {
                    case TOP: 	temp = (hold.giveBox(sim_box.id1));
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                                fp.coordinates.position(1) = fp.coordinates.position(1) + temp.m_width;
                                temp = hold.giveBox(sim_box.id2);
                                addNewBox(temp,fp,hold,temp.m_width + temp.m_length);
                        break;
                    case RIGHT: temp = (hold.giveBox(sim_box.id2));
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                                fp.coordinates.position(0) = fp.coordinates.position(0) - temp.m_length;
                                temp = (hold.giveBox(sim_box.id1));
                                addNewBox(temp,fp,hold,temp.m_width * temp.m_length);
                        break;
                     default: cout << "Error in add simulate boxes!";
            }
            break;
            default: cout << "Error in bin::add() with fp!";
		}
    }else
    {
        //new box position
        Eigen::Matrix3d box_size;
        box_size << new_box.m_length, 0, 0,
                    0, new_box.m_width,  0,
                    0, 0, new_box.m_height;

         new_box.position.position = fp.coordinates.position + box_size * fp.direction_box_pos;
        //check if the box can be push in x or y direction more, in order to reduce empty space
        // correct the new fp and new_box position
//        Eigen::Vector3d boxMove = checkNewBoxPosition(new_box,fp);
//        if(new_box.m_length > 0.35 || new_box.m_width > 0.35)
//        {
            Eigen::Vector3d boxMove = pushBoxTowardBinCenter(new_box,fp, helt);
            if(new_box.m_name == "a207eac3")
                boxMove(1) = 0.20;

            fp.coordinates.position = fp.coordinates.position + boxMove;
            new_box.position.position = fp.coordinates.position + box_size * fp.direction_box_pos;
//        }

        //check collision
//        std::cout << "=======================================================box collised ? = " << this->isColliding(new_box)<< std::endl;

        //calculate the supporting boxes and helts
        new_box.support_boxes.clear();
        if(floatEqual(fp.coordinates.position(2), 0.0))
        {
            supportingBox s_box;
            s_box.uuid = "p6ppallet";
            s_box.helt = new_box.m_width * new_box.m_length;
            new_box.support_boxes.push_back(s_box);
        }
        else
        {
            for(Box b : packed_boxes)
            {
                double overlap = bulletPhysics->getSupportArea(new_box, b);
                // new_box on top of b
                if(floatEqual(b.position.position(2) + b.m_height, new_box.position.position(2)) && floatGreaterThan(overlap, 0))
                {
                    supportingBox sup_box;
                    sup_box.uuid = b.m_name;
                    sup_box.helt = overlap;
                    new_box.support_boxes.push_back(sup_box);
                }
            }
        }

        //update new center of mass of bin
        Point com_new_box;
        com_new_box.position = fp.coordinates.position + (1 / 2.0) * box_size * (fp.direction_x+fp.direction_y+fp.direction_z);
        bin_com.position = (com_new_box.position * new_box.m_mass + bin_com.position * bin_mass) / (bin_mass + new_box.m_mass);
        //update new parameters of the bin
        bin_mass = bin_mass + new_box.m_mass;
        acc_height_of_all_boxes += new_box.m_height;
        boxes_volume += new_box.getVolume();
        boxes_support_surface += helt;
        boxes_ground_surface += new_box.m_length * new_box.m_width;

        packed_boxes_fps.push_back(fp);
        updateFittingPoints(new_box,fp);

        //update the boxes in the bin
        packed_boxes.push_back(new_box);
	}

    // add this new box to the bullet physics world
    bulletPhysics->addNewBoxToPhysicsNoRot(new_box);
}

Eigen::Vector3d Bin::checkNewBoxPosition(Box abox, FittingPoint fp)
{
    FittingPoint cp_0(Point(fp.coordinates.position + abox.m_height*fp.direction_z), fp.quadrant);
    FittingPoint cp_1(Point(fp.coordinates.position + abox.m_length*fp.direction_x), fp.quadrant);
    FittingPoint cp_2(Point(fp.coordinates.position + abox.m_width*fp.direction_y), fp.quadrant);

    //compute the distance in the -x direction, to see if it can be moved more!
    Eigen::Vector3d distanceX, distance;
    distanceX = getProjectionVector(fp.coordinates.position, -1*fp.direction_x);
    double dis_1 = distanceX(0);
    distanceX = getProjectionVector(cp_0.coordinates.position, -1*fp.direction_x);
    double dis_2 = distanceX(0);
    distance(0) = floatLessThan(std::fabs(dis_1), std::fabs(dis_2)) ? dis_1:dis_2;
    distanceX = getProjectionVector(cp_2.coordinates.position, -1*fp.direction_x);
    double dis_3 = distanceX(0);
    distance(0) = floatLessThan(std::fabs(dis_3), std::fabs( distance(0))) ? dis_3:distance(0);

    // -y direction
    Eigen::Vector3d distanceY;
    distanceY = getProjectionVector(fp.coordinates.position, -1*fp.direction_y);
    dis_1 = distanceY(1);
    distanceY = getProjectionVector(cp_0.coordinates.position, -1*fp.direction_y);
    dis_2 = distanceY(1);
    distance(1) = floatLessThan(std::fabs(dis_1), std::fabs(dis_2)) ? dis_1:dis_2;
    distanceY = getProjectionVector(cp_1.coordinates.position, -1*fp.direction_y);
    dis_3 = distanceY(1);
    distance(1) = floatLessThan(std::fabs(dis_3), std::fabs( distance(1))) ? dis_3:distance(1);

    // check1: if there is enough support area after move the box to new position
    // don't move too far in the case: the support area is not enough and not in the floor
    if( (std::fabs(distance(0)) > ((1-bpa::Params::instance()->HELT_RATE) * abox.m_length)) && !floatEqual(abox.position.position(2), 0.0) )
    {distance(0) = 0.0; }
    if( (std::fabs(distance(1)) > ((1-bpa::Params::instance()->HELT_RATE) * abox.m_width))  && !floatEqual(abox.position.position(2), 0.0) )
    {distance(1) = 0.0; }

    // check2: if this side contacts with other boxes!!
    Eigen::Vector3d centerX, centerY;
    centerX << fp.coordinates.position(0),
               fp.coordinates.position(1) + (1 / 2.0) * abox.m_width*fp.direction_y(1),
               fp.coordinates.position(2) + (1 / 2.0) * abox.m_height*fp.direction_z(2);

    centerY << fp.coordinates.position(0) + (1 / 2.0) * abox.m_length*fp.direction_x(0),
               fp.coordinates.position(1),
               fp.coordinates.position(2) + (1 / 2.0) * abox.m_height*fp.direction_z(2);

    if(bulletPhysics->isPointContact(centerX)) {distance(0) = 0.0; }
    if(bulletPhysics->isPointContact(centerY)) {distance(1) = 0.0; }

    // check3: if the box collides with others in the new place
    Box xBox = abox;     Box yBox = abox;
    xBox.position.position(0) = abox.position.position(0) + distance(0);
    if(this->isColliding(xBox)){distance(0) = 0.0;}
    yBox.position.position(1) = abox.position.position(1) + distance(1);
    if(this->isColliding(yBox)){distance(1) = 0.0;}

    distance(2) = 0.0;

    return distance;
}

Eigen::Vector3d Bin::pushBoxTowardBinCenter(Box abox, FittingPoint fp, double helt)
{
    // get the left down corner position of the box
    Eigen::Vector3d position, origin;
    origin(0) = fp.coordinates.position(0) + fp.direction_box_pos(0) * abox.m_length;
    origin(1) = fp.coordinates.position(1) + fp.direction_box_pos(1) * abox.m_width;
    origin(2) = fp.coordinates.position(2);

    //get the direction need to push
    Eigen::Vector3d dist, center, directionX, directionY, directionZ;
    center << bin_length/2.0, bin_width/2.0, origin(2);
    dist = center - origin;
    directionX << dist(0), 0, 0;
    directionX = directionX.normalized();
    directionY << 0, dist(1), 0;
    directionY = directionY.normalized();
    directionZ << 0, 0, 1;

    //compute the distance to push
    //get the corner position to push
    position = origin;
    if(directionX(0)>0) {position(0) = origin(0) + abox.m_length * directionX(0);}
    if(directionY(1)>0) {position(1) = origin(1) + abox.m_width * directionY(1);}

    Eigen::Vector3d cp_0, cp_1, cp_2;
    cp_0 = position + abox.m_height * directionZ;
    cp_1 = position - abox.m_length * directionX;  cp_1(2) += 0.1;
    cp_2 = position - abox.m_width * directionY;   cp_2(2) += 0.1;

    //compute the distance in the x direction, to see if it can be moved more!
    Eigen::Vector3d p1, p2, p3;
    p1 << position(0), position(1)-0.05*directionY(1), position(2)+0.1;
    p2 << cp_0(0), cp_0(1)-0.05*directionY(1), cp_0(2)-0.1;
    p3 << cp_2(0), cp_2(1)+0.05*directionY(1), cp_2(2);
    Eigen::Vector3d distanceX, distance;
    distanceX << 0, 0, 0;  distance << 0, 0, 0;
    distanceX = getProjectionVector(p1, directionX);
    double dis_1 = distanceX(0);
    distanceX = getProjectionVector(p2, directionX);
    double dis_2 = distanceX(0);
    distance(0) = floatLessThan(std::fabs(dis_1), std::fabs(dis_2)) ? dis_1:dis_2;
    distanceX = getProjectionVector(p3, directionX);
    double dis_3 = distanceX(0);
    distance(0) = floatLessThan(std::fabs(dis_3), std::fabs( distance(0))) ? dis_3:distance(0);

    // y direction
    p1 << position(0)-0.05*directionX(0), position(1), position(2)+0.1;
    p2 << cp_0(0)-0.05*directionX(0), cp_0(1), cp_0(2)-0.1;
    p3 << cp_1(0)+0.05*directionX(0), cp_1(1), cp_1(2);
    Eigen::Vector3d distanceY;  distanceY << 0, 0, 0;
    distanceY = getProjectionVector(p1, directionY);
    dis_1 = distanceY(1);
    distanceY = getProjectionVector(p2, directionY);
    dis_2 = distanceY(1);
    distance(1) = floatLessThan(std::fabs(dis_1), std::fabs(dis_2)) ? dis_1:dis_2;
    distanceY = getProjectionVector(p3, directionY);
    dis_3 = distanceY(1);
    distance(1) = floatLessThan(std::fabs(dis_3), std::fabs( distance(1))) ? dis_3:distance(1);

#ifdef DEBUG_PRINT
    std::cout << "Original box can move distance " << distance.transpose()<< std::endl;
#endif

    // check1: if there is enough support area after move the box to new position
    double supportArea = 0.0; //this supportArea does not include the floor
    if(floatEqual(fp.coordinates.position(2), 0.0))
        supportArea += abox.m_width * abox.m_length;
    Box xBox = abox;     Box yBox = abox;
    xBox.position.position(0) = abox.position.position(0) + distance(0);
    for(Box b : packed_boxes)
    {
        double overlap = bulletPhysics->getSupportArea(xBox, b);
        // abox on top of b
        if(floatEqual(b.position.position(2) + b.m_height, xBox.position.position(2)))
        {
            supportArea += overlap;
        }
    }
//    if(floatLessThan(supportArea, helt)) {distance(0) = 0.0;  std::cout << "x less support area\n";}
    if( (supportArea < bpa::Params::instance()->HELT_RATE * abox.m_length * abox.m_width) ) {distance(0) = 0.0;}
    if(this->isColliding(xBox)) {distance(0) = 0.0; }

    supportArea = 0.0;
    if(floatEqual(fp.coordinates.position(2), 0.0))
        supportArea += abox.m_width * abox.m_length;
    yBox.position.position(1) = abox.position.position(1) + distance(1);
    for(Box b : packed_boxes)
    {
        double overlap = bulletPhysics->getSupportArea(yBox, b);
        if(floatEqual(b.position.position(2) + b.m_height, yBox.position.position(2)))
        {
            supportArea += overlap;
        }
    }
//    if(floatLessThan(supportArea, helt)) {distance(1) = 0.0;}
    if( (supportArea < bpa::Params::instance()->HELT_RATE * abox.m_length * abox.m_width) ) {distance(1) = 0.0;}
    if(this->isColliding(yBox)) {distance(1) = 0.0;}

    //if the contact area of the new box is less than origin, then don't move
    double areaOld = 0.0;
    double areaX = 0.0;
    double areaY = 0.0;
    for(Box b : packed_boxes)
    {
        double oldArea = bulletPhysics->getContactArea(abox, b);
        areaOld += oldArea;
        double xArea = bulletPhysics->getContactArea(xBox, b);
        areaX += xArea;
        double yArea = bulletPhysics->getContactArea(yBox, b);
        areaY += yArea;
    }
    if(floatLessThan(areaX, areaOld)){distance(0) = 0.0;}
    if(floatLessThan(areaY, areaOld)){distance(1) = 0.0;}

    // check2: if this side center contacts with other boxes!!
    Eigen::Vector3d centerX, centerY;
    centerX << position(0),
               position(1) - (1 / 2.0) * abox.m_width * directionY(1),
               position(2) + (1 / 2.0) * abox.m_height;

    centerY << position(0) - (1 / 2.0) * abox.m_length * directionX(0),
               position(1),
               position(2) + (1 / 2.0) * abox.m_height;

    if(bulletPhysics->isPointContact(centerX)) {distance(0) = 0.0;}
    if(bulletPhysics->isPointContact(centerY)) {distance(1) = 0.0;}

    //check3: dont move too far, its not reasonable
    if(fabs(distance(0)) > abox.m_length){distance(0) = 0.0; }
    if(fabs(distance(1)) > abox.m_width){distance(1) = 0.0;}

    //check4: if the box is near the center
    if(origin(0) < center(0) && (origin(0)+abox.m_length > center(0)) ){distance(0) = 0.0;}
    if(origin(1) < center(1) && (origin(1)+abox.m_width > center(1)) ){distance(1) = 0.0; }

    //check5: if the box is dangerous, don't move, keep in the edge
    if(abox.is_dangerous && fabs(distance(0)) > (bpa::Params::instance()->MIN_BOX_SIZE-0.1) ){distance(0) = 0.0;}
    if(abox.is_dangerous && fabs(distance(1)) > (bpa::Params::instance()->MIN_BOX_SIZE-0.1) ){distance(1) = 0.0; }

    //check6: if move distance is too small, dont need to move
    if( std::fabs(distance(0)) < 0.03 ){distance(0) = 0.0;}
    if( std::fabs(distance(1)) < 0.03 ){distance(1) = 0.0;}

    //collisition check for both direction
    if(std::fabs(distance(0))>0 && std::fabs(distance(1))>0)
    {
        Box xyBox = abox;
        xyBox.position.position(0) = abox.position.position(0) + distance(0);
        xyBox.position.position(1) = abox.position.position(1) + distance(1);
        if(this->isColliding(xyBox)) {distance(1) = 0.0; std::cout<<"Move both x and y, collision!\n";}
    }

#ifdef DEBUG_PRINT
    std::cout << "--------------------------------------------------------------- box move distance " << distance.transpose()<< std::endl;
#endif
    return distance;
}

void Bin::removeFittingpoint(FittingPoint &fp, std::vector<FittingPoint> &fitting_points)
{
    std::vector<FittingPoint>::iterator it_fp = fitting_points.begin();
    while(it_fp != fitting_points.end())
    {
        if(it_fp->equals(fp))
        {
            fitting_points.erase(it_fp);
            break;
        }
        else
        {
            it_fp++;
        }
    }
}

void Bin::updateFittingPoints(Box &abox, FittingPoint &fp)
{
    bool USE_CORNER_POINTS = true;
    bool USE_EXTREME_POINTS = true;
    bool USE_ADDITIONAL_PROJECTIONS = false;
    bool USE_TOP_POINTS = true;

    // Add 3 corner points to fitting point
    FittingPoint cp_0(Point(fp.coordinates.position + abox.m_height*fp.direction_z), fp.quadrant);
    FittingPoint cp_1(Point(fp.coordinates.position + abox.m_length*fp.direction_x), fp.quadrant);
    FittingPoint cp_2(Point(fp.coordinates.position + abox.m_width*fp.direction_y), fp.quadrant);

    if(USE_CORNER_POINTS)
    {
        fitting_points.push_back(cp_0);
        fitting_points.push_back(cp_1);
        fitting_points.push_back(cp_2);
        abox.box_fps.push_back(cp_0);
        abox.box_fps.push_back(cp_1);
        abox.box_fps.push_back(cp_2);
    }

    // Add other 3 box top points as fp
    if(USE_TOP_POINTS)
    {
        double u_x = fp.coordinates.position(0) + fp.direction_box_pos(0) * abox.m_length;
        double u_y = fp.coordinates.position(1) + fp.direction_box_pos(1) * abox.m_width;

        if( u_x < bpa::Params::instance()->MIN_BOX_SIZE && (fp.coordinates.position(2)+abox.m_height) > 2.5)
        {
            FittingPoint tp_1(Point(fp.coordinates.position + abox.m_height*fp.direction_z + abox.m_length*fp.direction_x), 5-fp.quadrant);
            FittingPoint tp_2(Point(fp.coordinates.position + abox.m_height*fp.direction_z + abox.m_width*fp.direction_y), fp.quadrant+(fp.quadrant%2-0.5)*2);
            FittingPoint tp_3(Point(fp.coordinates.position + abox.m_height*fp.direction_z + abox.m_length*fp.direction_x + abox.m_width*fp.direction_y), fp.quadrant+fp.direction_x(0)*2);
            fitting_points.push_back(tp_1);
            fitting_points.push_back(tp_2);
            fitting_points.push_back(tp_3);
            abox.box_fps.push_back(tp_1);
            abox.box_fps.push_back(tp_2);
            abox.box_fps.push_back(tp_3);
        }
    }

    if(USE_EXTREME_POINTS)
    {
//        FittingPoint ep_0(Point(getProjection(cp_0.coordinates.position,-1*fp.direction_x)), fp.quadrant);
//        FittingPoint ep_1(Point(getProjection(cp_0.coordinates.position,-1*fp.direction_y)), fp.quadrant);

//        FittingPoint ep_0(Point(getProjection(fp.coordinates.position,-1*fp.direction_x)), fp.quadrant); //the projection of the box position
        FittingPoint ep_1(Point(getProjection(fp.coordinates.position,-1*fp.direction_y)), fp.quadrant);
        FittingPoint ep_2(Point(getProjection(cp_1.coordinates.position,-1*fp.direction_y)), fp.quadrant);
        FittingPoint ep_3(Point(getProjection(cp_1.coordinates.position,-1*fp.direction_z)), fp.quadrant);
        FittingPoint ep_4(Point(getProjection(cp_2.coordinates.position,-1*fp.direction_x)), fp.quadrant);
        FittingPoint ep_5(Point(getProjection(cp_2.coordinates.position,-1*fp.direction_z)), fp.quadrant);
//        fitting_points.push_back(ep_0);
        fitting_points.push_back(ep_1);
        fitting_points.push_back(ep_2);
        fitting_points.push_back(ep_3);
        fitting_points.push_back(ep_4);
        fitting_points.push_back(ep_5);
//        abox.box_fps.push_back(ep_0);
        abox.box_fps.push_back(ep_1);
        abox.box_fps.push_back(ep_2);
        abox.box_fps.push_back(ep_3);
        abox.box_fps.push_back(ep_4);
        abox.box_fps.push_back(ep_5);

        if(USE_ADDITIONAL_PROJECTIONS)
        {
            //quadrant projected along x-axis
            FittingPoint fp_0(Point(getProjection(cp_1.coordinates.position,-1*fp.direction_x)), fp.quadrant+(fp.quadrant%2-0.5)*2);
            //quadrant projected along y-axis
            FittingPoint fp_1(Point(getProjection(cp_2.coordinates.position,-1*fp.direction_y)), 5-fp.quadrant);
            fitting_points.push_back(fp_0);
            fitting_points.push_back(fp_1);
            abox.box_fps.push_back(fp_0);
            abox.box_fps.push_back(fp_1);
        }
    }

    // Finally delete all Fitting Points which are listed more than one time
    std::vector<FittingPoint> remove_doubles;
    for(FittingPoint fp : fitting_points)
    {
        // not find
        if(find(remove_doubles.begin(),remove_doubles.end(),fp)==remove_doubles.end())
        {
            remove_doubles.push_back(fp);
        }
    }

    // Remove the Fitting Point where the new Box is placed
    removeFittingpoint(fp, remove_doubles);

    // Remove all fps where all packed boxes are placed
    for(FittingPoint origin : packed_boxes_fps)
    {
        removeFittingpoint(origin, remove_doubles);
    }

    fitting_points.clear();
    for(FittingPoint fp : remove_doubles)
    {
        fitting_points.push_back(fp);
        abox.bin_fps.push_back(fp);
    }
}

Eigen::Vector3d Bin::getProjection(Eigen::Vector3d point, Eigen::Vector3d direction)
{
    return (bulletPhysics->castRays(point, direction) );
}

Eigen::Vector3d Bin::getProjectionVector(Eigen::Vector3d point, Eigen::Vector3d direction)
{
    Eigen::Vector3d projectPoint = (bulletPhysics->castRays(point, direction) );

    Eigen::Vector3d distance = projectPoint - point;
    for(size_t i = 0; i<3; ++i)
    {
        if( floatEqual(distance(i), 0.0))
        {
          distance(i) = 0.0;
        }
    }
    return distance;
}

bool Bin::isColliding(Box &box)
{
    return bulletPhysics->isColliding(box);
}

double Bin::getComDiffX(Box &abox, FittingPoint &fp)
{
    double com_new_x = 0.0;
    com_new_x = fp.coordinates.position(0) + fp.direction_x.transpose()*abox.center_of_mass.position; //center_of_mass.position(0)
    com_new_x = (com_new_x * abox.m_mass + bin_mass * bin_com.position(0)) / (bin_mass + abox.m_mass);

    return abs(bin_com.position(0)-target_com.position(0)) - abs(com_new_x-target_com.position(0));
}

double Bin::getComDiffY(Box &abox, FittingPoint &fp)
{
	double com_new_y = 0.0;
    com_new_y = fp.coordinates.position(1) + fp.direction_y.transpose()*abox.center_of_mass.position; //center_of_mass.position(1)
    com_new_y = (com_new_y * abox.m_mass + bin_mass * bin_com.position(1)) / (bin_mass + abox.m_mass);

    return abs(bin_com.position(1)-target_com.position(1)) - abs(com_new_y-target_com.position(1));
}

double Bin::getComDiffZ(Box &abox, FittingPoint &fp)
{
	double com_new_z = 0.0;
    com_new_z = fp.coordinates.position(2) + abox.center_of_mass.position(2);
    com_new_z = (com_new_z * abox.m_mass + bin_mass * bin_com.position(2)) / (bin_mass + abox.m_mass);

    return abs(bin_com.position(2)-target_com.position(2)) - abs(com_new_z-target_com.position(2));
}

double Bin::giveAverageHeightOfBoxes()
{
	return acc_height_of_all_boxes / ((double) packed_boxes.size());
}

double Bin::getNearestDistanceToEdges(Box &abox, FittingPoint &fp)
{
	double dis = bin_length;

    if(fp.coordinates.position(0) < dis)
    {
        dis = fp.coordinates.position(0);
	}
    if(fp.coordinates.position(1) < dis)
    {
        dis = fp.coordinates.position(1);
	}

    if(bin_width - fp.coordinates.position(1) - abox.m_width < dis)
    {
        dis = bin_width - fp.coordinates.position(1) - abox.m_width;
	}

    if(bin_length - fp.coordinates.position(0) - abox.m_length < dis)
    {
        dis = bin_length - fp.coordinates.position(0) - abox.m_length;
	}

	return dis;
}

//get current step packed boxes of bin
std::vector<Box> Bin::getStepPackedBoxes()
{
    // Get the std::vector of Boxes which have to be packed (waiting on the holding platform)
    // and mark them as packed
    std::vector<Box> packing_plan;
    for(Box box : packed_boxes)
    {
        if(box.is_packed == false)
        {
            packing_plan.push_back(box);
//            box.is_packed = true;  //changed in the actor.state after packed
        }
    }

    return packing_plan;
}

//set all packed boxes on bin, which are already packed, and update the pallet parameters
void Bin::setPackedBoxes(std::vector<Box> &boxes_on_bin)
{
    packed_boxes.clear();
    for(Box box : boxes_on_bin)
    {
        //update new parameters of the bin
        Point com_box;
        com_box.position = box.position.position + box.center_of_mass.position;
        bin_com.position = (com_box.position * box.m_mass + bin_com.position * bin_mass) / (bin_mass + box.m_mass);
        // update new center of mass of bin
        bin_mass = bin_mass + box.m_mass;
        acc_height_of_all_boxes += box.m_height;
        boxes_volume += box.getVolume();
        boxes_ground_surface += box.m_length * box.m_width;

        box.is_packed = true;
        this->packed_boxes.push_back(box);

        // set the packed box fps
        Eigen::Matrix3d box_size;
        box_size << box.m_length, 0, 0,
                    0, box.m_width,  0,
                    0, 0, box.m_height;
        FittingPoint fp = fitting_points[0];
        FittingPoint fp_packed(Point(box.position.position - box_size * fp.direction_box_pos), fp.quadrant);
        packed_boxes_fps.push_back(fp_packed);
    }
}

void Bin::setBinFittingPoints(std::vector<FittingPoint> &fps_on_bin)
{
    if(fps_on_bin.size()!=0)
    {
        this->fitting_points.clear();
    }
    for(FittingPoint fp : fps_on_bin)
    {
        this->fitting_points.push_back(fp);
    }
}

void Bin::updateBoxesPosition(std::vector<Box> &boxes_on_pallet)
{
    // update new position for some packed boxes
    std::vector<Box>::iterator it;
    for(Box box : boxes_on_pallet)
    {
        it = packed_boxes.begin();
        while(it->m_name != box.m_name && it != packed_boxes.end())  // find the box, which is packed
        {
            it++;
        }

        if(it == packed_boxes.end())
        {
            cout << "ERROR in actualize_positions: Could not find packed Box with UUID " << box.m_name << endl;
        }
        else
        {
            it->position.position = box.position.position;  // update the box position
        }
     }
}

}

