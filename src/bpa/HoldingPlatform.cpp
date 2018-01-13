/*
 * holdingplatform.cpp
 *
 *  Created on: August, 2015
 *      Author: Cai
 */

#include <iostream>
#include <fstream>

#include "bpa/HoldingPlatform.h"

using namespace std;

namespace bpa
{
HoldingPlatform::HoldingPlatform(double bin_l, double bin_w, bool GENERATE_SIMULATED_BOXES, std::vector<Box> boxes_on_holdingplatform, int startid)
{
	boxes_to_pack = boxes_on_holdingplatform;
	int count = startid + 1;

    for(std::vector<Box>::iterator it = boxes_to_pack.begin(); it != boxes_to_pack.end(); ++it)
    {
        it->m_id = count;
        count++;
    }

	bin_length = bin_l;
	bin_width = bin_w;

	generateRotationBoxes();

    if(GENERATE_SIMULATED_BOXES)
    {
		generateSimulatedBoxes();
	}
}

HoldingPlatform::~HoldingPlatform()
{
}

void HoldingPlatform::copyData(HoldingPlatform &other)
{
	bin_width = other.bin_width;
	bin_length = other.bin_length;

	boxes_to_pack.clear();
    for(Box b:other.boxes_to_pack)
    {
		boxes_to_pack.push_back(b);
	}

	existing_boxes.clear();
    for(Box b:other.existing_boxes)
    {
		existing_boxes.push_back(b);
	}

	simulated_boxes.clear();
    for(SIMULATED_BOX sim:other.simulated_boxes)
    {
		simulated_boxes.push_back(sim);
	}

    if(other.existing_boxes.size()>0)
    {
		existing_boxes_iterator = existing_boxes.begin();
        std::vector<Box>::iterator temp = other.existing_boxes.begin();
        while(temp != other.existing_boxes_iterator)
        {
			temp++;
			existing_boxes_iterator++;
		}
	}
}

bool HoldingPlatform::newBoxesComing(int amount, bool GENERATE_SIMULATED_BOXES)
{
    if(existing_boxes_iterator == existing_boxes.end())
    {
        return false;
    }else
    {
        // First erase all Simulated Boxes and Rotated Boxes in the actual boxes_to_pack
        it_btp = boxes_to_pack.begin();
        bool inc = true;
        while(it_btp!=boxes_to_pack.end())
        {
            inc = true;
            if((*it_btp).correspond_boxes.size()!=0)
            {
                inc = false;
                boxes_to_pack.erase(it_btp);
            }
            if(inc)
            {
                it_btp++;
            }
        }
        int boxes_before = boxes_to_pack.size();

        simulated_boxes.clear();

        for(int i = 0;i < amount-boxes_before; ++i)
        {
            if(existing_boxes_iterator == existing_boxes.end())
            {
                break;
            }else
            {
                boxes_to_pack.push_back(*existing_boxes_iterator);
                existing_boxes_iterator++;
            }
        }

        generateRotationBoxes();

        if(GENERATE_SIMULATED_BOXES)
        {
            generateSimulatedBoxes();
        }

        return true;
    }
}

void HoldingPlatform::moveBox(int id_rem)
{
    // If a Box has to be deleted from the holding platform it has to be proven if it is a simulated
    // Box. In that case all
    // corresponding boxes have to be deleted, too. Even if it is not a Simulated Box - the rotated
    // version and every Simulated
	// Box containing the regarded Box have to be deleted anyway.

    remove_ids.clear();
	addRemoveIds(id_rem,true);
    removeBoxes();
}

void HoldingPlatform::addRemoveIds(int i, bool both_directions)
{
    if(std::find(remove_ids.begin(),remove_ids.end(),i)==remove_ids.end()) //not found
    {
        remove_ids.push_back(i);
        for(Box b:boxes_to_pack)
        {
            //rotated box has corresponding box real box
            if(b.hasCorrespondBox(i))
            {
				addRemoveIds(b.getId(),false);
			}
		}
        if(both_directions)
        {
			Box i_box = giveBox(i);
            for(int core : i_box.correspond_boxes)
            {
                addRemoveIds(core, true);
			}
		}
	}
}

void HoldingPlatform::removeBoxes()
{
    std::unique(remove_ids.begin(), remove_ids.end());
    std::sort(remove_ids.begin(), remove_ids.end());
//    remove_ids.sort();
//    remove_ids.unique();
	bool inc = true;
	it_btp = boxes_to_pack.begin();

    while(it_btp != boxes_to_pack.end())
    {
		inc = true;
        if(std::find(remove_ids.begin(),remove_ids.end(),(*it_btp).getId()) != remove_ids.end())
        {
            boxes_to_pack.erase(it_btp);
			inc = false;
		}
        if(inc)
        {
			it_btp++;
		}
	}
}

bool HoldingPlatform::hasCorrespondBoxes(Box &b1, Box &b2)
{
    if(!(b1.hasCorrespondBox(b2.getId())))
    {
        for(int i_sub_b2 : b2.correspond_boxes)
        {
            Box temp = giveBox(i_sub_b2);
            if(hasCorrespondBoxes(b1,temp))
            {
                return true;
            }
        }
        for(int i_sub : b1.correspond_boxes)
        {
            Box temp = giveBox(i_sub);
            if(hasCorrespondBoxes(temp,b2))
            {
                return true;
            }
        }
        return false;
    }else
    {
        return true;
    }
}

void HoldingPlatform::generateRotationBoxes()
{
    // To enable also all possible rotations of the boxes there are new boxes with the rotated sizes of the original boxes
    // added to the holding platform. Those new boxes have the original boxes as "correspond_box" attribute
    // - which simply means if one of the boxes (the original or the rotated one) will be added to the bin, then the other will
    // also automatically be deleted from the holding platform

	int highest_old_id = 0;
    for(Box b:boxes_to_pack)
    {
        if(b.m_id > highest_old_id)
        {
			highest_old_id = b.m_id;
		}
	}

	int count = 1;

    std::cout <<"-----------------------------------------------------------------" << std::endl;
    std::cout << "Before generateRotationBoxes for MA is = " << boxes_to_pack.size() << endl;
    std::vector<Box> rotation_boxes;
    for(Box b : boxes_to_pack)
    {
        if(b.correspond_boxes.size() == 0)
        {
            Box a(b.m_width, b.m_length, b.m_height, b.m_mass, b.m_name, b.box_labels, b.material, b.tool_name);
            a.m_id = highest_old_id + count;
            a.is_rotated = true;
            a.rotation = 90;
            a.correspond_boxes.push_back(b.m_id);
            rotation_boxes.push_back(a);
            count++;
		}
	}
    boxes_to_pack.insert(boxes_to_pack.end(), rotation_boxes.begin(), rotation_boxes.end());
    std::cout << "After generateRotationBoxes, boxes = " << boxes_to_pack.size() << endl;
}

void HoldingPlatform::generateSimulatedBoxes()
{
    // This concept was introduced to raise the possibilty that items which fit exactly next to each
    // other are packed together:
    // For those items "Simulated Boxes" are generated - which simply means that new boxes are added
    // to the holding platform
    // which have the sizes of the original boxes packed together; if one of the original boxes is
    // added to the bin the
    // Simulated Box will also automatically be deleted from the holding platform - and also the
    // other way around if a
    // Simulated Box will be packed (then in fact the original boxes will be packed in the order how
    // they are simulated in the
	// Simulated Box)

	bool added_new_box = false;

    std::vector<Box> simulate_boxes;
    int highest_old_id = 0;
    for(Box b:boxes_to_pack)
    {
        if(b.m_id > highest_old_id)
        {
            highest_old_id = b.m_id;
        }
    }

    for(Box abox:boxes_to_pack)
    {
        for(Box bbox:boxes_to_pack)
        {
//            if(!isSimulatedBox(abox.m_id) && !isSimulatedBox(bbox.m_id))  // why???
            if(!isSimulatedBox(abox.m_id) && !isSimulatedBox(bbox.m_id) && abox.box_labels.size()==bbox.box_labels.size())  // why???
            {
                if( floatEqual(abox.m_length, bbox.m_length) && floatEqual(abox.m_height, bbox.m_height) && floatLessThan(abox.m_width + bbox.m_width, bin_width))
                {
                    if(!abox.equalsBox(bbox.getId()) && !hasCorrespondBoxes(abox,bbox) && !hasCorrespondBoxes(bbox,abox) && !hasSimulatedBox(bbox.getId(),abox.getId()))
                    {
                        Box a(abox.m_length,abox.m_width+bbox.m_width,abox.m_height,abox.m_mass + bbox.m_mass,"Sim "+abox.m_name + "+"+bbox.m_name,abox.box_labels);
                        a.is_simulated = true;
						a.m_id = highest_old_id+1;
                        a.center_of_mass.position(1) = calculateCenterOfMass(abox.center_of_mass.position(1),bbox.center_of_mass.position(1)+abox.m_width,abox.m_mass,bbox.m_mass);
                        a.center_of_mass.position(2) = calculateCenterOfMass(abox.center_of_mass.position(2),bbox.center_of_mass.position(2),abox.m_mass,bbox.m_mass);

						a.correspond_boxes.push_back(abox.getId());
						a.correspond_boxes.push_back(bbox.getId());
                        simulate_boxes.push_back(a);

						SIMULATED_BOX sim_box;
						sim_box.sim_id = a.getId();
                        sim_box.arrangement = TOP;
                        //ccx: down is id1
                        if(abox.position.position(1) < bbox.position.position(1))
                        {
                            sim_box.id1 = abox.getId();
                            sim_box.id2 = bbox.getId();
                        }
                        else
                        {
                            sim_box.id2 = abox.getId();
                            sim_box.id1 = bbox.getId();
                        }

						simulated_boxes.push_back(sim_box);

						added_new_box = true;
                        highest_old_id++;
					}
                }
                else if( floatEqual(abox.m_width, bbox.m_width) && floatEqual(abox.m_height, bbox.m_height) && floatLessEqual(abox.m_length + bbox.m_length, bin_length))
                {
                    if(!abox.equalsBox(bbox.getId()) && !hasCorrespondBoxes(abox,bbox) && !hasCorrespondBoxes(bbox,abox) && !hasSimulatedBox(bbox.getId(),abox.getId()))
                    {
                        Box a(abox.m_length+bbox.m_length,abox.m_width,abox.m_height,abox.m_mass + bbox.m_mass,"Sim "+abox.m_name + "+"+bbox.m_name,abox.box_labels);
                        a.is_simulated = true;
						a.m_id = highest_old_id+1;
                        a.center_of_mass.position(0) = calculateCenterOfMass(abox.center_of_mass.position(0),bbox.center_of_mass.position(0) + abox.m_length,abox.m_mass,bbox.m_mass);
                        a.center_of_mass.position(2) = calculateCenterOfMass(abox.center_of_mass.position(2),bbox.center_of_mass.position(2),abox.m_mass,bbox.m_mass);
						a.correspond_boxes.push_back(abox.getId());
						a.correspond_boxes.push_back(bbox.getId());
                        simulate_boxes.push_back(a);

						SIMULATED_BOX sim_box;
						sim_box.sim_id = a.getId();
                        sim_box.arrangement = RIGHT;
                        //ccx: left is id1
                        if(abox.position.position(0) < bbox.position.position(0))
                        {
                            sim_box.id1 = abox.getId();
                            sim_box.id2 = bbox.getId();
                        }
                        else
                        {
                            sim_box.id2 = abox.getId();
                            sim_box.id1 = bbox.getId();
                        }

						simulated_boxes.push_back(sim_box);

						added_new_box = true;
                        highest_old_id++;
					}
				}
			}
		}
	}

    boxes_to_pack.insert(boxes_to_pack.end(), simulate_boxes.begin(), simulate_boxes.end());
    std::cout << "After generateSimulatedBoxes, boxes = " << boxes_to_pack.size() << endl;

//    if(added_new_box /* && Mode = Level 2*/)
//    {
//		generateSimulatedBoxes();
//	}
}

SIMULATED_BOX HoldingPlatform::giveSimulatedBox(int i)
{
    for(SIMULATED_BOX s: simulated_boxes)
    {
        if(s.sim_id == i)
        {
            return s;
        }
    }
    SIMULATED_BOX null;
    return null;
}

bool HoldingPlatform::isSimulatedBox(int i)
{
    for(SIMULATED_BOX s : simulated_boxes)
    {
        if(s.sim_id == i)
        {
			return true;
		}
	}
	return false;
}

bool HoldingPlatform::hasSimulatedBox(int i1, int i2)
{
    for(SIMULATED_BOX s:simulated_boxes)
    {
        if((s.id1 == i1 && s.id2 == i2) || (s.id1 == i2 && s.id2 == i1))
        {
            return true;
        }
    }
    return false;
}

Box HoldingPlatform::giveBox(int i)
{
    for(Box b:boxes_to_pack)
    {
        if(b.equalsBox(i))
        {
			return b;
		}
	}
    Box null(0,0,0,0,"",std::vector<std::string>());
	return null;
}

bool HoldingPlatform::hasBox(int i)
{
	for(Box b : boxes_to_pack){
        if(b.equalsBox(i))
        {
			return true;
		}
	}
	return false;
}

double HoldingPlatform::calculateCenterOfMass(double a, double b, double mass_a, double mass_b)
{
    return (a * mass_a + b * mass_b) / (mass_a + mass_b);
}

}
