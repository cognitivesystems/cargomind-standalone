#ifndef BPP_FUNCTIONS_H_
#define BPP_FUNCTIONS_H_

#include <iostream>
#include <limits>
#include <iostream>
#include <fstream>
#include <string>
#include <bpa/Box.h>
#include <bpa/Bin.h>
#include <bpa/HoldingPlatform.h>
#include <bpa/FittingPoint.h>
#include <bpa/MixedScore.h>
#include <bpa/Parameters.h>

using namespace std;

namespace bpa
{

    class BinPackingPlanner
    {
    public:
        BinPackingPlanner();
        /*
        * To pack a subset of boxes of the boxes on the Holding Platform into the bin
        * \param abin: A Bin Object with the current configuration of the bin
        * \param boxes_on_holdingplatform: Boxes representing the current boxes waiting on the Holding Platform to be packed
        * \return A Bin Object with the new packing configuration
        */
        Bin solveWithOneFunction(Bin &abin, std::vector<Box> boxes_on_holdingplatform);

        Bin solveWithTwoFunctions(Bin &abin, std::vector<Box> boxes_on_holdingplatform);

        /*
        * To pack a subset of boxes of the boxes on the Holding Platform into the bin with the Deep Search Algorithm
        * \param curr_best_solution: A Bin Object with the current configuration of the bin
        * \param boxes_on_holdingplatform: Boxes representing the current boxes waiting on the Holding Platform to be packed
        * \return A Bin Object with the new packing configuration
        */
        Bin solveWithDeepSearchOneFunction(Bin &curr_best_solution, std::vector<Box> boxes_on_holdingplatform);

        Bin solveWithDeepSearchTwoFunctions(Bin &curr_best_solution, std::vector<Box> boxes_on_holdingplatform);

        /**
        * To get the relative position of one Box in the bin to another one
        * \param abox: A Box Object which would be packed into the bin
        * \param fp: A Fitting_Pointindicating where abox would be packed
        * \param b: A Box representing a packed box in the bin
        * \param abin: The current Configuration of the Bin
        * \return An integer value indicating how both boxes are arranged to each other
        */
        int givePosition(Box &abox, FittingPoint &fp, Box &b, double &count_neighbours, double &distance_x, double &distance_y);

        /*
        * To get the Score (For the Two Functions Algorithm)
        * \param abox: A Box which would be packed into the bin
        * \param fp: A Fitting_Point indicating where abox would be packed
        * \param abin: The current Configuration of the Bin
        * \param hold: A Holding_Platform representing the current state of the Holding Platform
        * \return A double value indicating the Score of the Box-Fitting Point Combination
        */
        double giveScore(Box &abox, FittingPoint &fp, Bin &abin, double &helt);

        // To get the Score (For the One Function Algorithm)
        double giveMixedScore(Box &abox, FittingPoint &fp, Bin &abin/*, HoldingPlatform hold*/, double &helt);

        /*
        * To add a next box to the packing configuration (Internal Function)
        * \param abin: A Bin indicating the current Bin Configuration
        * \param holding: A Holding_Platform indicating the current state of the Holding Platform
        * \param cum_score: A pointer to the double value where the cumulated score is stored
        * \return A boolean value if the function could add a new box to the configuration
        */
        bool addNextBoxToPackingConfigurationTwoFunctions(Bin &abin,HoldingPlatform &holding, double* cum_score);

        bool addNexBoxToPackingConfigurationOneFunction(Bin &abin, HoldingPlatform &holding, double* cum_score);

    };

    /*
    * To compare Fitting Points Scores (For The Sort() Function)
    * \param first: A const Fitting_Point Object
    * \param second: A const Fitting_Point Object
    * \return A boolean Value if the Fitting Point Score of first is higher than second
    */
    bool compareFittingpointScores(const FittingPoint &first, const FittingPoint &second);

    /*
    * To compare Choosing Values (For The Sort() Function of the List)
    */
    bool compareChoosingValues(const Box &first, const Box &second);

    /*
    * To compare Mixed Scores (For The Sort() Function of the List)
    */
    bool compareMixedScores(const Mixed_Score &first, const Mixed_Score &second);


}

#endif
