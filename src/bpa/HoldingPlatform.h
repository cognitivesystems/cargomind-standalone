/*
 * holdingplatform.h
 *
 *  Created on: August, 2015
 *      Author: Cai
 */

#ifndef HOLDINGPLATFORM_H_
#define HOLDINGPLATFORM_H_

#include <stdlib.h>
#include <queue>
#include "SimulatedBox.h"
#include "Box.h"

using namespace std;

namespace bpa
{

class HoldingPlatform
{
public:
    /*
	 * \param boxes_on_holdingplatform: A List of Boxes which are currently on the real Holding	Platform
     * \param startid: An integer value indicating the highest ID of a box currently packed in the bin;
	 */
    HoldingPlatform(double bin_l, double bin_w, bool GENERATE_SIMULATED_BOXES, std::vector<Box> boxes_on_holdingplatform, int startid);

    // \brief Destructor
    virtual ~HoldingPlatform();

    /*
	 * To refill the Holding Platform
     * \param amount: An integer value indicating the number of boxes which should be added to the Holding Platform
	 */
    bool newBoxesComing(int amount, bool GENERATE_SIMULATED_BOXES);

    /*
	 * To remove a Box from the Holding Platform (And all its corresponding boxes)
     * \param i: the Id of the box which should be removed
	 */
    void moveBox(int i);

    // SubRoutine from moveBox() to manage the removing of all corresponding boxes
    void removeBoxes();

    /*
	 * Function to get all ids from coresponding boxes for the removing process of a box
     * \param i: the Id of the box which should be removed
     * \param both_directions: if the algorithm should search in
	 * 	both directions; False: Only the coresponding boxes of the box with Id i are added
	 *	to the rem list; False: Additionaly the boxes which have the box with Id i as corresponding box are added to the list 
	 */
    void addRemoveIds(int i, bool both_directions);

    /*
     * To generate Rotations
     */
    void generateRotationBoxes();

    /*
	 * To generate Simulated Boxes
	 */
    void generateSimulatedBoxes();

    /*
	 * To figure out if a box is a Simulated Box or not
	 * \param i: An integer value indicating the Id of a box
	 * \return A boolean value: True: i is a Simulated Box; False: i is not a Simulated Box
	 */
    bool isSimulatedBox(int i);

    /*
     * To figure out if a Simulated Box consisting of two regarded Boxes already exists
     * \param i1: the Id of a box
     * \param i2: the Id of a box
     * \return A boolean value: True: A Simulated Box containing the box with Id i1 and a box with Id i2 already exists; False: No such Simulated Box already exists
     */
    bool hasSimulatedBox(int i1, int i2);

    /*
     * To get the SIMULATED_BOX object with a given Id
     * \param i: An integer value indicating the Id of the SIMULATED BOX
     * \return The SIMULATED_BOX object with Id i
     */
    SIMULATED_BOX giveSimulatedBox(int i);

    /*
     * To get the Box with a given Id
     * \param i: An integer value indicating the Id of the Box
     * \return The Box object with Id i
     */
    Box giveBox(int i);

    /*
     * To prove if a box with a given Id is represented on the Holding Platform
     * \param i: An integer value indicating the Id of a box
     * \return A boolean value: True: The box does exists on the Holding Platform;
     * False: The Box is not represented on the Holding Platform
     */
    bool hasBox(int i);

    /*
	 * To figure out if A Box b2 or a corresponding box of the Box b2 is a corresponding box of another Box b1
	 * \param b1: A Box object
	 * \param b2: A Box object
     * \return A boolean value: True: b1 has b2 as corresponding box (or another box which is a corresponding box of b2);
     *  False: b2 and any corresponding boxes of b2 are not corresponding boxes of b1
	 */
    bool hasCorrespondBoxes(Box &b1, Box &b2);

    /*
     * To calculate the new center of mass
     * \return: the position of the new center of mass
     */
    double calculateCenterOfMass(double a, double b, double mass_a, double mass_b);

    /*
     * To copy the data from another HoldingPlatform
     */
    void copyData(HoldingPlatform &other);

    std::vector<Box> boxes_to_pack;             /* Boxes waiting on the Holding Platform */
    std::vector<Box>::iterator it_btp;
    std::vector<Box>::iterator it_choosing;
    std::vector<int> remove_ids;                /* Box Ids which should be removed from the Holding Platform */

    std::vector<Box> existing_boxes;            /* Boxes listed in the box configuration file */
    std::vector<Box>::iterator existing_boxes_iterator;

    std::vector<SIMULATED_BOX> simulated_boxes;  /* Simulated Boxes of the Holding Platform */

    double bin_length;
    double bin_width;
};

}

#endif /* HOLDINGPLATFORM_H_ */
