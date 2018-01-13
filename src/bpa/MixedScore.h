/*
 * MixedScore.h
 *
 *  Created on: August, 2015
 *      Author: Cai
 */

#ifndef MIXEDSCORE_H_
#define MIXEDSCORE_H_

#include "Box.h"
#include "FittingPoint.h"

namespace bpa
{

class Mixed_Score
{
public:

    /* \brief A Constructor
	 */
	Mixed_Score();

    // \brief Destructor
	virtual ~Mixed_Score();

    double score;        /*The mixed_score */
    Box box_to_pack;
    FittingPoint point_to_pack;

    double temp_a_helt;  /* Supported surface of the Box in the regarded Box-Fitting Point combination */
};

}

#endif /* MIXEDSCORE_H_ */
