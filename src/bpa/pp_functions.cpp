#include "bpa/pp_functions.h"

namespace bpa
{

PalletizationPlanner::PalletizationPlanner()
{
//    vacuum_length = 0.5; // condisder the max parts
//    vacuum_width = 0.5;
//    vacuum_height = 0.38;
//    fork_l = 1.1;
//    fork_w1 = 0.46; //0.6
//    fork_w2 = 1.03;
//    fork_h = 2.03; // 1.26;

    CLEARANCE = 0.5;
    current_bin_height = 0.0;
}

std::vector<Box> PalletizationPlanner::getPalletizationPlan(Bin &abin, std::vector<Box>& packing_plan)
{
    // Sort the std::vector according to the high
//    std::sort(packing_plan.begin(), packing_plan.end(), compareBoxHeightSide);

    // Proof that no item is blocked by another:
//    sortBoxesRemoveBlock(packing_plan, abin);
    std::vector<Box> palletized_boxes = getOrderdBoxSets(packing_plan, abin);

    //get the move direction for boxes
    std::vector<Box> palletized_plan = getMoveDirection(palletized_boxes, abin);

    return palletized_plan;
}

bool PalletizationPlanner::sortBoxesRemoveBlock(std::vector<Box> &packing_plan, Bin &abin)
{
    //fill the *it one by one
    for (std::vector<Box>::iterator it = packing_plan.begin(); it != packing_plan.end(); ++it)
    {
        //get the *it box which not blocks any of other boxes
        std::vector<Box>::iterator it_in = it+1;
        while(it_in != packing_plan.end())
        {
            if(isBlocking(*it_in, *it, abin))
            {
                std::iter_swap(it, it_in);
                it_in = it+1;
            }
            else
                it_in++;
        }
    }
    return true;
}

std::vector<Box> PalletizationPlanner::getOrderdBoxSets(std::vector<Box> &bpp_boxes, Bin &abin)
{
    std::vector<Box> palletized_boxes;
    /// 1. get the dependency for every boxes
//    std::cout <<"=========================================================\n";
    for(std::vector<Box>::iterator it = bpp_boxes.begin(); it != bpp_boxes.end(); ++it)
    {
        for(std::vector<Box>::iterator it_in = bpp_boxes.begin(); it_in != bpp_boxes.end(); ++it_in)
        {
            if( (it_in != it) && (isBlocking(*it_in, *it, abin)) )
            {
                // if *it blocks box *it_in
                it->dependency.push_back(it_in->m_name);
            }
        }
    }

    /// 2. get the orders
    while(bpp_boxes.size() != 0)
    {
        // find the boxes whose dependency size=0,  can be multiple boxes
        std::vector<Box>::iterator it_selected = bpp_boxes.end();
        std::vector<std::vector<Box>::iterator> filtered_list;
        for(std::vector<Box>::iterator it = bpp_boxes.begin(); it != bpp_boxes.end(); ++it)
        {
            if(it->dependency.size() == 0)
            {
                filtered_list.push_back(it);
            }
        }
//        std::cout << "****************** Found zero boxes number = " << filtered_list.size() << std::endl;
        if(filtered_list.size() != 0)
        {
            it_selected = filtered_list[0];
            //if there is more than one, select the box according to the tool change
            if(palletized_boxes.size() > 0)
            {
                for(size_t i=0; i<filtered_list.size(); i++)
                {
                    if( filtered_list[i]->tool_name == palletized_boxes.back().tool_name)
                    {
                       it_selected = filtered_list[i];
                       break;
                    }
                }
            }
            std::string box_uuid = it_selected->m_name;
            palletized_boxes.push_back(*it_selected);
            bpp_boxes.erase(it_selected);

            // remove the selected box from other boxes' dependency
            for(std::vector<Box>::iterator it = bpp_boxes.begin(); it != bpp_boxes.end(); ++it)
            {
                std::vector<std::string>::iterator it_remove = std::find_if(it->dependency.begin(), it->dependency.end(), [box_uuid](std::string const &depend_name){return (depend_name == box_uuid);});

                if(it_remove != it->dependency.end())
                    it->dependency.erase(it_remove);
//                std::cout << "Box "<< it->m_name << " left dependency = " << it->dependency.size() << std::endl;
            }
        }
        else
        {
            std::cout << "Errrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr!!!\n";
            break;
        }
    }

    return palletized_boxes;
}

//x, y direction first, then consider z
std::vector<Box> PalletizationPlanner::getMoveDirection(std::vector<Box> &box_plan, Bin &abin)
{
    std::vector<Box> boxes;
    //add the packed boxes in pallet
    for(Box packed_box : abin.packed_boxes)
    {
        boxes.push_back(packed_box);
    }

    std::vector<Box> pp_boxplan;
    for(Box box : box_plan)
    {
        bool flagX = true;
        bool flagY = true;
        bool flagZ = true;

        current_bin_height = 0.0;
        //x direction
        for(size_t i=0; i<boxes.size(); i++)
        {
            if(xBlocking(box, boxes[i], abin))
            {
               flagX = false;
               current_bin_height = boxes[i].position.position(2) + boxes[i].m_height;
               std::cout << "Box " << box.m_name <<" is blocked in X direction!\n";
            }
        }
        //y direction
        for(size_t i=0; i<boxes.size(); i++)
        {
            if(yBlocking(box, boxes[i], abin))
            {
               flagY = false;
               if(boxes[i].position.position(2) + boxes[i].m_height > current_bin_height)
               {
                  current_bin_height = boxes[i].position.position(2) + boxes[i].m_height;
               }
               std::cout << "Box " << box.m_name <<" is blocked in Y direction!\n";
            }
        }
        // z direction, should be always be true;
        for(size_t i=0; i<boxes.size(); i++)
        {
            if(zBlocking(box, boxes[i], abin))
            {
               flagZ = false;
               std::cout << "Box " << box.m_name <<" is blocked in Z direction!\n";
            }
            if ( (current_bin_height + box.m_height) > (abin.bin_height + CLEARANCE) )
            {
               flagZ = false;
               std::cout << "Box " << box.m_name <<" is can not move in Z direction, its too high!\n";
            }
        }

        if(flagX==true && flagY==true && flagZ==true) {box.box_direction = "XYZ_AXIS";}
        else if(flagX==true && flagY==true) {box.box_direction = "XY_AXIS";}
        else if(flagX==true) {box.box_direction = "X_AXIS";}
        else if(flagY==true) {box.box_direction = "Y_AXIS";}
        else if (flagZ==true)
        {
           box.box_direction = "Z_AXIS";
        }
        else
        {
           std::cout << current_bin_height << "  errorrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr !\n";
        }

        if(box.tool_name == "schmalzgripper")
            getPlacePositionVacumm(box, abin);
        else
            getPlacePositionFork(box, abin);

        boxes.push_back(box);
        pp_boxplan.push_back(box);

        // add this new box to the bullet physics world
        abin.bulletPhysics->addNewBoxToPhysics(box);
    }

    return pp_boxplan;
}

std::vector<Box> PalletizationPlanner::getGraspPose(std::vector<Box> &pp_boxplan, Bin &abin)
{
    std::vector<Box> grasp_boxplan;
    std::cout << "================================================================Grasp planner:";
    for(Box box : pp_boxplan)
    {
        std::cout << "\n--------------------------------------------------------"<< box.m_name << ";     graspTool=" << box.tool_name << std::endl;
        if(box.tool_name == "schmalzgripper")
            getPlacePositionVacumm(box, abin);
        else
            getPlacePositionFork(box, abin);

        grasp_boxplan.push_back(box);
        // add this new box to the bullet physics world
        abin.bulletPhysics->addNewBoxToPhysics(box);
    }
    return grasp_boxplan;
}

void PalletizationPlanner::getPlacePositionVacumm(Box &box, Bin &abin)
{
    //gripper_position is wrt to box center frame
    /// vacumm gripper
    box.gripper_position.position << 0, 0, box.m_height/2.0 + vacuum_height;

    //check the collision
    bool is_collided = false;
    Eigen::Vector3d offset, vacuum_center, box_center;
    Box vacuum_bbox(vacuum_length, vacuum_width, 0.3, 10, "vacuum_bbox", std::vector<std::string>());
    offset << 0, 0, box.m_height/2.0 + 0.08;  //center wrt box center
    box_center << box.position.position(0)+box.m_length/2.0, box.position.position(1)+box.m_width/2.0, box.position.position(2)+box.m_height/2.0;
    vacuum_center = box_center + offset;
    vacuum_bbox.position.position << vacuum_center(0)-vacuum_bbox.m_length/2.0, vacuum_center(1)-vacuum_bbox.m_width/2.0, vacuum_center(2);
    is_collided = abin.bulletPhysics->isColliding(vacuum_bbox);

    // If the box is too small. make some offset in x and y, not in the box center
    if(box.is_rotated)   // if the box is rotated, here length and width is opposite
    {
        if( box.m_length < vacuum_length && is_collided)
            box.gripper_position.position(0) = vacuum_length/2.0 - box.m_length/2.0 + 0.02;//+x
        if( box.m_width < vacuum_width && is_collided)
            box.gripper_position.position(1) = vacuum_width/2.0 - box.m_width/2.0 + 0.02; //+y
    }else
    {
        if( box.m_length < vacuum_length && is_collided)
            box.gripper_position.position(0) = -(vacuum_length/2.0 - box.m_length/2.0 + 0.02);//-x
        if( box.m_width < vacuum_width && is_collided)
            box.gripper_position.position(1) = vacuum_width/2.0 - box.m_width/2.0 + 0.02; //+y
    }
    box.gripper_orientation = 0.0;

    std::cout << box.m_name << "  current rotation = " << box.current_rotation << ";  is_collised = " << is_collided << std::endl;
    std::cout << "The vacuum gripper_position = " << box.gripper_position.position.transpose() << ";  gripper_orientation = " << box.gripper_orientation << std::endl;
}

void PalletizationPlanner::getPlacePositionFork(Box &box, Bin &abin)
{
    box.gripper_position.position << 0, 0, (fork_h - box.m_height/2.0 + 0.07);
    // offset in y
    if(box.m_width <= fork_w2) //small box
        box.gripper_position.position(1) = (fork_w2-fork_w1)-box.m_width/2.0;
    else
        box.gripper_position.position(1) = box.m_width/2.0 - fork_w1;

    //for safety between the side of the box, add 10cm offset in y
    box.gripper_position.position(1) = box.gripper_position.position(1) + 0.10;

    // offset in x, check the collision first
    ///TODO: check the gjk collision
    bool is_collided = false;
    Eigen::Vector3d offset, fork_center, box_center;
    if(!box.is_rotated)
    {
        Box fork_bar(fork_l, 0.1, 0.1, 10, "fork_bar", std::vector<std::string>());
//        offset << 0, fork_w2-box.m_width/2.0+0.05, 0;  //center wrt box center(x,y direction)
        offset << 0, fork_w1+std::fabs(box.gripper_position.position(1))+0.05, 0;
        box_center << box.position.position(0)+box.m_length/2.0, box.position.position(1)+box.m_width/2.0, box.position.position(2);
        fork_center = box_center + offset;
        fork_bar.position.position << fork_center(0)-fork_bar.m_length/2.0, fork_center(1)-fork_bar.m_width/2.0, fork_center(2);
        is_collided = abin.bulletPhysics->isColliding(fork_bar);
    }
    else  // the box lengh and width is opposite
    {
        Box fork_bar(0.1, fork_l, 0.1, 10, "fork_bar", std::vector<std::string>());
//        offset << -(fork_w2-box.m_width/2.0+0.05), 0, 0;
        offset << -(fork_w1+std::fabs(box.gripper_position.position(1))+0.05), 0, 0;
        box_center << box.position.position(0)+box.m_width/2.0, box.position.position(1)+box.m_length/2.0, box.position.position(2);
        fork_center = box_center + offset;
        fork_bar.position.position << fork_center(0)-fork_bar.m_length/2.0, fork_center(1)-fork_bar.m_width/2.0, fork_center(2);
        is_collided = abin.bulletPhysics->isColliding(fork_bar);
    }
    // offset in x
    if( box.m_length <= fork_l && is_collided)
    {
        if(!box.is_rotated)
            box.gripper_position.position(0) = -(fork_l/2.0 - box.m_length/2.0); //-x
        else
            box.gripper_position.position(0) = (fork_l/2.0 - box.m_length/2.0);  //+x
        std::cout << box.m_name << " is_collised = " << is_collided << std::endl;
    }

    box.gripper_orientation = M_PI_2;

    //Case 2
//    std::fabs(-90 - box.rotation) < 45 || std::fabs(box.rotation) < 45
    if(box.current_rotation > 45 || box.current_rotation < -135 )
    {
        box.gripper_position.position(0) = -box.gripper_position.position(0);
        box.gripper_position.position(1) = -box.gripper_position.position(1);
        box.gripper_orientation = -box.gripper_orientation;
        std::cout << "------------------------------------------------------------Case 2" << std::endl;
    }

    std::cout << box.m_name << "  current rotation = " << box.current_rotation << ";  is_collised = " << is_collided << std::endl;
    std::cout << "The fork gripper_position = " << box.gripper_position.position.transpose() << ";  gripper_orientation = " << box.gripper_orientation << std::endl;
}

// abox is current box, if abox is blocked by b, move abox first
bool PalletizationPlanner::isBlocking(Box abox, Box b, Bin abin)
{   // first check if abox can move in x and y direction
    bool flag = false;

    if(xBlocking(abox, b, abin) && (abox.position.position(2) < b.position.position(2)+b.m_height) )
    { flag = true; }
    if(yBlocking(abox, b, abin) && (abox.position.position(2) < b.position.position(2)+b.m_height) )
    { flag = true; }
    if(zBlocking(abox, b, abin))
    { flag = true; }

    return flag;
}

//abox is blocked by b, abox should move first!
bool PalletizationPlanner::xBlocking(Box abox, Box b, Bin abin)
{
    //check if direction +x is blocking, 2D
    if(abox.position.position(0) >= b.position.position(0))
    {
        abox.position.position(0) = b.position.position(0);
        if(abin.bulletPhysics->isCollidingBox(abox,b))
        {
            return true;
        }
    }
    return false;
}

bool PalletizationPlanner::yBlocking(Box abox, Box b, Bin abin)
{
    //check if direction -y is blocking, 2D
    if(abox.position.position(1) <= b.position.position(1))
    {
        abox.position.position(1) = b.position.position(1);
        if(abin.bulletPhysics->isCollidingBox(abox,b))
        {
            return true;
        }
    }
    return false;
}

bool PalletizationPlanner::zBlocking(Box abox, Box b, Bin abin)
{
    //check if direction +z is blocking
    if( (abox.position.position(2)+abox.m_height) <= b.position.position(2))
    {
        abox.position.position(2) = b.position.position(2);
        if(abin.bulletPhysics->isCollidingBox(abox,b))
        {
            return true;
        }
    }
    return false;
}

bool PalletizationPlanner::compareBoxHeightSide(const Box &first,const Box &second)
{
    return first.position.position(2) + first.m_height < second.position.position(2) + second.m_height;
}

bool PalletizationPlanner::loadParamsFromFile(string filename)
{
    Json::Value paramJson;
    Json::Reader reader;
    std::ifstream file(filename);
    bool parsingSuccessful = reader.parse(file, paramJson);
    if(!parsingSuccessful)
        return false;
    vacuum_length = paramJson["tool_params"]["vacuum"]["length"].asDouble();
    vacuum_width = paramJson["tool_params"]["vacuum"]["width"].asDouble();
    vacuum_height = paramJson["tool_params"]["vacuum"]["height"].asDouble();

    fork_l = paramJson["tool_params"]["cranefork"]["length"].asDouble();
    fork_w1 = paramJson["tool_params"]["cranefork"]["width1"].asDouble();
    fork_w2 = paramJson["tool_params"]["cranefork"]["width2"].asDouble();
    fork_h = paramJson["tool_params"]["cranefork"]["height"].asDouble();

    return true;
}

}

