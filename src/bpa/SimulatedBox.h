/*
 * Simulatedbox.h
 *
 *  Created on: August, 2015
 *      Author: Cai
 */


#ifndef SIMULATEDBOX_H_
#define SIMULATEDBOX_H_

enum Arrangement {TOP, RIGHT};
//TOP in Y Direction
//RIGHT in X Direction

namespace bpa
{

struct SIMULATED_BOX
{
    int sim_id;  /* Id of the Simulated Box */
    int id1;     /* Id of the First Box in the Simulated Box */
    int id2;     /* Id of the Second Box in the Simulated Box */

    Arrangement arrangement; /* Indicating how the Boxes are arranged:
		TOP means the id2-box is placed over the id1-box (in positive Y-direction)
		RIGHT means the id2-box is placed right to the id1-box (in positive X-direction)*/
};

}

#endif /* SIMULATEDBOX_H_ */
