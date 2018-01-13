/*
 * Box.cpp
 *
 *  Created on: August, 2015
 *      Author: Cai
 */

#include "bpa/Box.h"

namespace bpa
{

//Box::Box(double length, double width, double height, double mass, string name, bool frag)
//{
//    m_length = length;
//    m_width = width;
//    m_height = height;
//    m_mass = mass;
//    m_id = 0; //1;
//    m_name = name;
//	is_rotated = false;
//    rotation = 0;

//    position.position << 0.0, 0.0, 0.0;
//    center_of_mass.position << length/2.0, width/2.0, height/2.0;

//    choose_score = 0;
//    is_packed = false;

//    gripper_position.position << 0.0, 0.0, height/2.0;
//    tool_name = "schmalzgripper"; //cranepalletfork

//    setChooseScore();
//    box_direction = "";

//    is_fragile = frag;
//    is_dangerous = false;
//    material = "carton";
//}

Box::Box(double length, double width, double height, double mass, std::string name, std::vector<std::string> labels, std::string material, std::string tool):
    box_labels(labels),
    material(material),
    tool_name(tool)
{
    m_length = length;
    m_width = width;
    m_height = height;
    m_mass = mass;
    m_id = 0; //1;
    m_name = name;
    is_rotated = false;
    rotation = 0;
    is_simulated = false;

    position.position << 0.0, 0.0, 0.0;
    center_of_mass.position << length/2.0, width/2.0, height/2.0;

    choose_score = 0;
    is_packed = false;

    gripper_position.position << 0.0, 0.0, height/2.0;
    gripper_orientation = 0.0;
//    if(mass < 30 && (material=="Carton" || material=="carton"))
//    {
//        tool_name = "schmalzgripper";
//        z_max = 4.304 - 0.093 - 0.382;
//    }
    if(tool_name == "")
    {
        if( (mass < 50 || m_length < 0.75) && (material=="Carton" || material=="carton"))
        {
            tool_name = "offsetgripper";
            z_max = 4.304 - 0.093 - 0.487;
        }
        else if(mass < 60 && (material=="Styrofoam" || material=="styrofoam" || material=="Wooden" || material=="wooden"))
        {
            tool_name = "foamgripper";
            z_max = 4.304 - 0.093 - 0.466;
        }
        else
        {
            tool_name = "cranepalletfork";
            z_max = 4.304 - 0.093 - 1.34;
        }
    }

    setChooseScore();
    box_direction = "";

    is_fragile = false;
    is_dangerous = false;
    is_stackable = true;
    for(std::string label : box_labels)
    {
        if(label=="fragile")
        {
            is_fragile = true;
        }
        else if(label=="dangerous" || label=="flammable" || label=="flammableLiquid")
        {
            is_dangerous = true;
        }
        else if(label=="nostacking")
        {
            is_stackable = false;
        }
    }
//    material = "carton";
}

Box::Box()
{
    m_length = 0;
    m_width = 0;
    m_height = 0;
    m_mass = 0;
    m_id = 0;
    m_name = "";
    is_rotated = false;
    rotation = 0;

    position.position << 0.0, 0.0, 0.0;
    center_of_mass.position << 0.0, 0.0, 0.0;

    choose_score = 0;
    is_packed = false;

    gripper_position.position << 0.0, 0.0, 0.0;
    gripper_orientation = 0.0;
    tool_name = "offsetgripper";

    setChooseScore();
    box_direction = "";

    is_fragile = false;
    is_dangerous = false;
    is_simulated = false;
    is_stackable = true;
    box_labels = std::vector<std::string>();
    material = "carton";
}

Box::~Box()
{
}

double Box::getVolume()
{
    return m_length * m_width * m_height;
}

bool Box::equalsBox(int id)
{
    return m_id == id;
}

bool Box::equalsBox(Box &abox)
{
    return (floatEqual(this->m_length, abox.m_length) && floatEqual(this->m_width, abox.m_width)
            && floatEqual(this->m_height, abox.m_height) && floatEqual(this->m_mass, abox.m_mass)
            && floatEqual(this->position.position(0), abox.position.position(0))
            && floatEqual(this->position.position(1), abox.position.position(1))
            && floatEqual(this->position.position(2), abox.position.position(2)) );
}

int Box::getId()
{
    return m_id;
}

bool Box::hasCorrespondBox(int id)
{
    return find(correspond_boxes.begin(), correspond_boxes.end(), id)!= correspond_boxes.end();
}

bool Box::operator==(const Box &other)
{
    return this->m_id == other.m_id;
}

void Box::setChooseScore()
{
    double max_mass = 300.0;  //600.0
    double mass = 0.0;
    if(m_mass > max_mass)
    {
        mass = max_mass;
    }
    else
        mass = m_mass;

	// Todo adapt the values for the normalization to the real values
    choose_score = 	bpa::Params::instance()->W_MASS * mass / max_mass
                  + 100.0 * bpa::Params::instance()->W_VOL * (m_length / 100.0 + m_width / 100.0 + m_height / 70.0) / 3.0
                  + bpa::Params::instance()->W_MASSVOL * (mass / max_mass) * 100.0 * (m_length / 100.0 + m_width / 100.0 + m_height / 70.0) / 3.0;
}

}
