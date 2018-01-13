#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <iostream>
#include <cmath>
#include <limits>

namespace bpa
{

class Params
{
    static Params *singleton_;
    Params()
    {
//        singleton_ = this;
    }
public:
    /*< --------------------------------------------- */
    /*< Weighting Parameters of the Scoring Functions */
    double W_MASS;
    double W_VOL;
    double W_MASSVOL;

    double W_COM;
    double HELT_RATE; /* Minimum Percentage of ground surface of a Box which has to be supported by other boxes or the pallet underneath */
    double W_SUPPORTED; /* The box support area */
    double W_CONTACT; /* The box contact areas with surounding */

    double NEIGHBOUR_CONSTANT;
    double W_ASSIGNMENT;
    double W_PLACE_NEAR;
    double BIN_HEIGHT;  /* The box reaches the bin max height. */
    double MIN_BOX_SIZE; /* The min box size(width). */

    double W_ITEM_IN_THE_BOTTOM_AREA;
    double W_HIGH_ITEMS_GOOD_PLACED;
    /*< --------------------------------------------- */

    bool GENERATE_SIMULATED_BOXES; /* Indicating if Simulated Boxes should be generated*/
    bool START_WITH_ALL_EDGES_AS_FP; /* Indicating if all four edges should be used asinitial Fitting Points*/

    int SEARCH_HEIGHT; /* Indicating the Search height of the Deep Search Algorithms*/
    int SEARCH_WIDTH; /* Indicating the Search width of the Deep Search Algorithms*/

    static Params *instance()
    {
        if(NULL == bpa::Params::singleton_)
        {
            bpa::Params::singleton_ = new Params;
        }
        return bpa::Params::singleton_;
    }

};

//const PARAMETERS params =
//{
//    0.3,                  // W_MASS:   0.3
//    0.6,                  // W_VOL : 0.6 / 0.8
//    0.1,                  // W_MASSVOL: 0.3 / 0.1

//    0.2,                  // W_COM      0.0/0.2
//    1.0,                  // HELT_RATE: 0.7 / 0.9
//    0.1,                  // W_SUPPORTED
//    0.1,                  // W_CONTACT   (with this look better!)

//    0.0,                  // NEIGHBOUR_CONSTANT  0.1 (8.png)/0.0(5.png):  0.1, looks better for new data
//    0.3,                  // W_ASSIGNMENT        0.4/0.3
//    0.8,                  // W_PLACE_NEAR        0.6/0.0/0.8 ??
//    0.02,                 // If the top box is reach the max height
//    0.3,                  // The min box size(width).

//    0.3,                  // W_ITEM_IN_THE_BOTTOM_AREA /0.3
//    0.3,                  // W_HIGH_ITEMS_GOOD_PLACED

//    false,                // GENERATE_SIMULATED_BOXES
//    false,                // START_WITH_ALL_EDGES_AS_FP
//    10,                   // SEARCH_HEIGHT
//    10                   // SEARCH_WIDTH
//};

struct supportingBox {
  std::string uuid;
  double helt;
};

}

//#define FLOAT_EPS std::numeric_limits<float>::epsilon()
#define FLOAT_EPS 0.001

inline bool floatEqual(double a, double b)
{
     return std::fabs(a - b) < FLOAT_EPS;
}

inline bool floatLessThan(double a, double b)
{
     return ((std::fabs(a - b) > FLOAT_EPS) && ((a - b) < FLOAT_EPS));
}

inline bool floatGreaterThan(double a, double b)
{
     return ((std::fabs(a - b) > FLOAT_EPS) && ((a - b) > FLOAT_EPS));
}

inline bool floatGreaterEqual(double a, double b)
{
     return (floatEqual(a, b) || floatGreaterThan(a, b));
}

inline bool floatLessEqual(double a, double b)
{
     return (floatEqual(a, b) || floatLessThan(a, b));
}

#endif
