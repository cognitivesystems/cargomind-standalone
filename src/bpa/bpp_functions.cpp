#include "bpa/bpp_functions.h"
#include <omp.h>
//#define DEBUG_PRINT

namespace bpa
{

bool compareFittingpointScores(const FittingPoint &first, const FittingPoint &second)
{
    return first.score > second.score;
}

bool compareChoosingValues(const Box &first,const Box &second)
{
    return first.choose_score > second.choose_score;
}

bool compareMixedScores(const Mixed_Score &first,const Mixed_Score &second)
{
    return first.score > second.score;
}


BinPackingPlanner::BinPackingPlanner()
{
}

Bin BinPackingPlanner::solveWithOneFunction(Bin &abin, std::vector<Box> boxes_on_holdingplatform)
{
    std::vector<bpa::Box>::iterator pack_it = abin.packed_boxes.begin();
    int highest_id = 0; //1;
    while(pack_it != abin.packed_boxes.end())
    {
        if(pack_it->m_id > highest_id)
        {
            highest_id = pack_it->m_id;
        }
        if(pack_it->is_packed == false)
        {
            pack_it->is_packed = true;
        }
        pack_it++;
    }

    // Initializing the Holding Platform
    HoldingPlatform holding(abin.bin_length, abin.bin_width, bpa::Params::instance()->GENERATE_SIMULATED_BOXES, boxes_on_holdingplatform, highest_id);
//    for(bpa::Box b : holding.boxes_to_pack)
//    {
//        std::cout << "HA UUID " <<b.m_id <<" "<< b.m_name << "  l " << b.m_length << "  w " << b.m_width << "  h " << b.m_height << "  mass " << b.m_mass <<" x " << b.position.position(0) << " y " << b.position.position(1) << " z " << b.position.position(2) << "  rotated " << b.is_rotated << "  rotation " << b.rotation<< " " << b.material << ", " << b.box_labels[0] << std::endl;
//    }

    // Initializing the iteration counter
    int iter = 0;
    double score = 0;
    while(addNexBoxToPackingConfigurationOneFunction(abin,holding,&score) && iter < 10000)
    {
        iter++;
    }
    return abin;
}

Bin BinPackingPlanner::solveWithTwoFunctions(Bin &abin, std::vector<Box> boxes_on_holdingplatform)
{
    std::vector<bpa::Box>::iterator pack_it = abin.packed_boxes.begin();
    int highest_id = 0; //1;
    while(pack_it != abin.packed_boxes.end())
    {
        if(pack_it->m_id > highest_id)
        {
            highest_id = pack_it->m_id;
        }
        if(pack_it->is_packed == false)
        {
            pack_it->is_packed = true;
            cout << "Not packed - ID " << pack_it->m_id << endl;
        }
        pack_it++;
    }

    HoldingPlatform holding(abin.bin_length,abin.bin_width,bpa::Params::instance()->GENERATE_SIMULATED_BOXES,boxes_on_holdingplatform,highest_id);

    //choose the box which has the highest choose_score.
    std::sort(holding.boxes_to_pack.begin(), holding.boxes_to_pack.end(), compareChoosingValues);
    //	holding.boxes_to_pack.sort(compareChoosingValues);
    holding.it_choosing = holding.boxes_to_pack.begin();

    int iter = 0;
    double score = 0;

    /*cout << "CURRENT STATE OF THE HOLDING PLATFORM ---------------------------" << endl;
    for(Box b : holding.boxes_to_pack){
        cout << "Box Name " << b.name << " with core boxes ";
        for(int p : b.correspond_boxes){
            cout << holding.giveBox(p).name << " and ";
        }
        cout << endl;
    }
    cout <<" -----------------------------------------------------------------" << endl;*/

    while(addNextBoxToPackingConfigurationTwoFunctions(abin,holding,&score) && iter < 10000)
    {
        iter++;
    }
    return abin;
}

Bin BinPackingPlanner::solveWithDeepSearchOneFunction(Bin &curr_best_solution, std::vector<Box> boxes_on_holdingplatform)
{
    std::vector<bpa::Box>::iterator pack_it = curr_best_solution.packed_boxes.begin();
    int highest_id = 0; //1;
    while(pack_it != curr_best_solution.packed_boxes.end())
    {
        if(pack_it->m_id > highest_id){
            highest_id = pack_it->m_id;
        }
        if(pack_it->is_packed == false){
            pack_it->is_packed = true;
        }
        pack_it++;
    }

    // Initializing the bins:
    Bin curr_testing(curr_best_solution.bin_length,curr_best_solution.bin_width,curr_best_solution.bin_height,bpa::Params::instance()->START_WITH_ALL_EDGES_AS_FP);
    Bin bin_before_testing(curr_best_solution.bin_length,curr_best_solution.bin_width,curr_best_solution.bin_height,bpa::Params::instance()->START_WITH_ALL_EDGES_AS_FP);

    // Initializing the holding platforms:
    HoldingPlatform holding_curr_best_solution(curr_best_solution.bin_length,curr_best_solution.bin_width,bpa::Params::instance()->GENERATE_SIMULATED_BOXES,boxes_on_holdingplatform,highest_id);
    HoldingPlatform holding_curr_testing(curr_best_solution.bin_length,curr_best_solution.bin_width,bpa::Params::instance()->GENERATE_SIMULATED_BOXES,boxes_on_holdingplatform,highest_id);
    HoldingPlatform holding_before_testing(curr_best_solution.bin_length,curr_best_solution.bin_width,bpa::Params::instance()->GENERATE_SIMULATED_BOXES,boxes_on_holdingplatform,highest_id);

    // Initializing some variables
    bool solved = false;
    int iter = 0;
    //    FittingPoint point_to_pack;
    double curr_max_cum_score;
    double curr_cum_score;
    std::vector<Mixed_Score> mixed_scores;
    Mixed_Score temp_mixed_score;
    std::vector<Mixed_Score>::iterator position_iterator;
    double helt = 0;

    while(!solved && iter < 10000)
    {
        if(holding_curr_best_solution.boxes_to_pack.size() == 0)
        {
            return curr_best_solution;
        }else
        {
            bin_before_testing.copyData(curr_best_solution);
            holding_before_testing.copyData(holding_curr_best_solution);

            curr_max_cum_score = 0;

            mixed_scores.clear();
            for(Box b : holding_curr_best_solution.boxes_to_pack)
            {
                for(FittingPoint fp : curr_best_solution.fitting_points)
                {
                    temp_mixed_score.box_to_pack = b;
                    temp_mixed_score.point_to_pack = fp;
                    temp_mixed_score.score = giveMixedScore(b,fp,curr_best_solution/*,holding_curr_best_solution*/, helt);
                    temp_mixed_score.temp_a_helt = helt;
                    mixed_scores.push_back(temp_mixed_score);
                }
            }
            std::sort(mixed_scores.begin(), mixed_scores.end(), compareMixedScores);
            //			mixed_scores.sort(compareMixedScores);
            position_iterator = mixed_scores.begin();

            if(position_iterator->score >= 0)
            {
                for(int i=0;i<bpa::Params::instance()->SEARCH_WIDTH && position_iterator != mixed_scores.end();++i)
                {
                    if(position_iterator->score >= 0)
                    {
                        curr_cum_score = position_iterator->score;
                        holding_curr_testing.copyData(holding_before_testing);
                        curr_testing.copyData(bin_before_testing);
                        curr_testing.addNewBox(position_iterator->box_to_pack, position_iterator->point_to_pack,holding_curr_testing,position_iterator->temp_a_helt);
                        holding_curr_testing.moveBox(position_iterator->box_to_pack.m_id);
                        bool height_search = true;
                        for(int i = 0;i<bpa::Params::instance()->SEARCH_HEIGHT && height_search;++i){
                            height_search = addNexBoxToPackingConfigurationOneFunction(curr_testing,holding_curr_testing,&curr_cum_score);
                        }
                        if(curr_cum_score > curr_max_cum_score)
                        {
                            curr_max_cum_score = curr_cum_score;
                            curr_best_solution.copyData(bin_before_testing);
                            holding_curr_best_solution.copyData(holding_before_testing);
                            curr_best_solution.addNewBox(position_iterator->box_to_pack, position_iterator->point_to_pack,holding_curr_best_solution,position_iterator->temp_a_helt);
                            holding_curr_best_solution.moveBox(position_iterator->box_to_pack.getId());
                        }
                    }
                    position_iterator++;
                }
            }else{
                return curr_best_solution;
            }
        }

        iter++;
    }

    if(!solved){
        cout << "Terminated by max_iter" << endl;
    }
    return curr_best_solution;
}

Bin BinPackingPlanner::solveWithDeepSearchTwoFunctions(Bin &curr_best_solution, std::vector<Box> boxes_on_holdingplatform)
{
    std::vector<bpa::Box>::iterator pack_it = curr_best_solution.packed_boxes.begin();
    int highest_id = 0; //1;
    while(pack_it != curr_best_solution.packed_boxes.end())
    {
        if(pack_it->m_id > highest_id){
            highest_id = pack_it->m_id;
        }
        if(pack_it->is_packed == false){
            pack_it->is_packed = true;
        }
        pack_it++;
    }

    // Initializing the bins:
    Bin curr_testing(curr_best_solution.bin_length,curr_best_solution.bin_width,curr_best_solution.bin_height,bpa::Params::instance()->START_WITH_ALL_EDGES_AS_FP);
    Bin bin_before_testing(curr_best_solution.bin_length,curr_best_solution.bin_width,curr_best_solution.bin_height,bpa::Params::instance()->START_WITH_ALL_EDGES_AS_FP);

    // Initializing the holding platforms:
    HoldingPlatform holding_curr_best_solution(curr_best_solution.bin_length,curr_best_solution.bin_width,bpa::Params::instance()->GENERATE_SIMULATED_BOXES,boxes_on_holdingplatform,highest_id);
    HoldingPlatform holding_curr_testing(curr_best_solution.bin_length,curr_best_solution.bin_width,bpa::Params::instance()->GENERATE_SIMULATED_BOXES,boxes_on_holdingplatform,highest_id);
    HoldingPlatform holding_before_testing(curr_best_solution.bin_length,curr_best_solution.bin_width,bpa::Params::instance()->GENERATE_SIMULATED_BOXES,boxes_on_holdingplatform,highest_id);

    // Initializing of the Iterator
    std::sort(holding_curr_best_solution.boxes_to_pack.begin(), holding_curr_best_solution.boxes_to_pack.end(), compareChoosingValues);
    //	holding_curr_best_solution.boxes_to_pack.sort(compareChoosingValues);
    holding_curr_best_solution.it_choosing = holding_curr_best_solution.boxes_to_pack.begin();

    // Initializing some variables
    Box box_to_pack(0,0,0,0,"",std::vector<std::string>());
    bool solved = false;
    int iter = 0;
    FittingPoint point_to_pack;
    double curr_max_cum_score;
    double curr_cum_score;
    std::vector<FittingPoint>::iterator fp_scores;
    double helt = 0;

    while(!solved && iter < 10000)
    {
        if(holding_curr_best_solution.boxes_to_pack.size() == 0 || holding_curr_best_solution.it_choosing == holding_curr_best_solution.boxes_to_pack.end())
        {
            return curr_best_solution;
        }else
        {
            bin_before_testing.copyData(curr_best_solution);
            holding_before_testing.copyData(holding_curr_best_solution);

            curr_max_cum_score = 0;

            box_to_pack = (*holding_curr_best_solution.it_choosing);

            fp_scores = bin_before_testing.fitting_points.begin();
            while(fp_scores != bin_before_testing.fitting_points.end())
            {
                point_to_pack = *fp_scores;
                fp_scores->score = giveScore(box_to_pack, point_to_pack,bin_before_testing/*,holding_before_testing*/, helt);
                fp_scores->temp_a_helt = helt;
                fp_scores++;
            }
            std::sort(bin_before_testing.fitting_points.begin(), bin_before_testing.fitting_points.end(), compareFittingpointScores);
            //			bin_before_testing.fitting_points.sort(compareFittingpointScores);

            fp_scores = bin_before_testing.fitting_points.begin();


            if(fp_scores->score >= 0)
            {
                for(int i=0;i<bpa::Params::instance()->SEARCH_WIDTH && fp_scores != bin_before_testing.fitting_points.end();++i)
                {
                    if(fp_scores->score >= 0)
                    {
                        curr_cum_score = fp_scores->score;
                        point_to_pack = *fp_scores;
                        holding_curr_testing.copyData(holding_before_testing);
                        curr_testing.copyData(bin_before_testing);
                        curr_testing.addNewBox(box_to_pack,point_to_pack,holding_curr_testing,point_to_pack.temp_a_helt);
                        holding_curr_testing.moveBox(box_to_pack.m_id);
                        holding_curr_testing.it_choosing = holding_curr_testing.boxes_to_pack.begin();
                        bool height_search = true;
                        for(int i = 0;i<bpa::Params::instance()->SEARCH_HEIGHT && height_search;++i){
                            height_search = addNextBoxToPackingConfigurationTwoFunctions(curr_testing,holding_curr_testing,&curr_cum_score);
                        }
                        if(curr_cum_score > curr_max_cum_score)
                        {
                            curr_max_cum_score = curr_cum_score;
                            curr_best_solution.copyData(bin_before_testing);
                            holding_curr_best_solution.copyData(holding_before_testing);
                            curr_best_solution.addNewBox(box_to_pack,point_to_pack,holding_curr_best_solution,point_to_pack.temp_a_helt);
                            holding_curr_best_solution.moveBox(box_to_pack.getId());
                            holding_curr_best_solution.it_choosing = holding_curr_best_solution.boxes_to_pack.begin();
                        }
                    }
                    fp_scores++;
                }
            }else{
                holding_curr_best_solution.it_choosing++;
            }
        }
        iter++;
    }

    if(!solved){
        cout << "Terminated by max_iter" << endl;
    }
    return curr_best_solution;
}

int BinPackingPlanner::givePosition(Box& abox, FittingPoint &fp, Box &b, double &count_neighbours, double &distance_x, double &distance_y)
{
    double u_x = fp.coordinates.position(0);
    double u_y = fp.coordinates.position(1);

    switch(fp.quadrant){
        case 1: break;
        case 2: u_y -= abox.m_width; break;
        case 3: u_x -= abox.m_length; u_y -= abox.m_width; break;
        case 4: u_x -= abox.m_length; break;
        default: cout << "error in givePosition()" << endl;
    }

    int position = 0;

    // ccx 5: abox is not on top of bbox
    if(b.position.position(2) <= fp.coordinates.position(2) && b.position.position(2) + b.m_height > fp.coordinates.position(2))
    {
        // ccx: abox is in the right side of b
//        if(b.position.position(0) + b.m_length == u_x)
        if(floatEqual(b.position.position(0) + b.m_length, u_x))
        {
            if(b.position.position(1) <= u_y && b.position.position(1) + b.m_width > u_y && b.position.position(1) + b.m_width <= u_y + abox.m_width){
                position = 17;
            }

            if(b.position.position(1) + b.m_width >= u_y + abox.m_width && b.position.position(1) >= u_y && b.position.position(1) < u_y + abox.m_width){
                position = 19;
            }

            if(b.position.position(1) > u_y && b.position.position(1) + b.m_width < u_y + abox.m_width){
                position = 18;
            }

            if(b.position.position(1) < u_y && b.position.position(1) + b.m_width > u_y + abox.m_width){
                position = 20;
            }
        }
        // ccx: abox is in the left side
//        if(b.position.position(0) == u_x + abox.m_length)
        if(floatEqual(b.position.position(0), u_x + abox.m_length))
        {
            if(b.position.position(1) <= u_y && b.position.position(1) + b.m_width > u_y && b.position.position(1) + b.m_width <= u_y + abox.m_width){
                position = 21;
            }

            if(b.position.position(1) + b.m_width >= u_y + abox.m_width && b.position.position(1) >= u_y && b.position.position(1) < u_y + abox.m_width){
                position = 23;
            }

            if(b.position.position(1) > u_y && b.position.position(1) + b.m_width < u_y + abox.m_width){
                position = 22;
            }

            if(b.position.position(1) < u_y && b.position.position(1) + b.m_width > u_y + abox.m_width){
                position = 24;
            }
        }
        // ccx: abox is in the up side (2D, y direction)
//        if(b.position.position(1) + b.m_width == u_y)
        if(floatEqual(b.position.position(1) + b.m_width, u_y))
        {
            if(b.position.position(0) <= u_x && b.position.position(0) + b.m_length > u_x && b.position.position(0) + b.m_length <= u_x + abox.m_length){
                position = 25;
            }

            if(b.position.position(0) + b.m_length >= u_x + abox.m_length && b.position.position(0) >= u_x && b.position.position(0) < u_x + abox.m_length){
                position = 27;
            }

            if(b.position.position(0) > u_x && b.position.position(0) + b.m_length < u_x + abox.m_length){
                position = 26;
            }

            if(b.position.position(0) < u_x && b.position.position(0) + b.m_length > u_x + abox.m_length){
                position = 28;
            }
        }
        // ccx: abox is in the down side
//        if(b.position.position(1) == u_y + abox.m_width)
        if(floatEqual(b.position.position(1), u_y + abox.m_width))
        {
            if(b.position.position(0) <= u_x && b.position.position(0) + b.m_length > u_x && b.position.position(0) + b.m_length <= u_x + abox.m_length){
                position = 29;
            }

            if(b.position.position(0) + b.m_length >= u_x + abox.m_length && b.position.position(0) >= u_x && b.position.position(0) < u_x + abox.m_length){
                position = 31;
            }

            if(b.position.position(0) > u_x && b.position.position(0) + b.m_length < u_x + abox.m_length){
                position = 30;
            }

            if(b.position.position(0) < u_x && b.position.position(0) + b.m_length > u_x + abox.m_length){
                position = 32;
            }
        }

        // ccx: ?? distances which do not have the intersection with the fp
        switch(fp.quadrant){
            case 1: if(b.position.position(0) >= fp.coordinates.position(0) + abox.m_length){
                        if(b.position.position(1) <= u_y && b.position.position(1) + b.m_width > u_y){
                            if(b.position.position(0) - u_x - abox.m_length < distance_x){
                                distance_x = b.position.position(0) - u_x - abox.m_length;
                            }
                        }
                    } //case 21,24
                    if(b.position.position(1) >= fp.coordinates.position(1) + abox.m_width){
                        if(b.position.position(0) <= u_x && b.position.position(0) + b.m_length > u_x){
                            if(b.position.position(1) - u_y - abox.m_width < distance_y){
                                distance_y = b.position.position(1) - u_y - abox.m_width;
                            }
                        }
                    } //case 29,32
                break;
            case 2: if(b.position.position(0) >= fp.coordinates.position(0) + abox.m_length){
                        if(b.position.position(1) <= u_y && b.position.position(1) + b.m_width > u_y){
                            if(b.position.position(0) - u_x - abox.m_length < distance_x){
                                distance_x = b.position.position(0) - u_x - abox.m_length;
                            }
                        }
                    } //case 21,24
                    if(b.position.position(1) + b.m_width <= fp.coordinates.position(1) - abox.m_width){
                        if(b.position.position(0) <= u_x && b.position.position(0) + b.m_length > u_x){
                            if(u_y - (b.position.position(1) + b.m_width) < distance_y){
                                distance_y = u_y - (b.position.position(1) + b.m_width);
                            }
                        }
                    } //case 25, 28
                break;
            case 3: if(b.position.position(0) + b.m_length <= fp.coordinates.position(0) - abox.m_length){
                        if(b.position.position(1) <= u_y && b.position.position(1) + b.m_width > u_y){
                            if(u_x - (b.position.position(0) + b.m_length) < distance_x){
                                distance_x = u_x - (b.position.position(0) + b.m_length);
                            }
                        }
                    } //case 17, 20
                    if(b.position.position(1) + b.m_width <= fp.coordinates.position(1) - abox.m_width){
                        if(b.position.position(0) <= u_x && b.position.position(0) + b.m_length > u_x){
                            if(u_y - (b.position.position(1) + b.m_width) < distance_y){
                                distance_y = u_y - (b.position.position(1) + b.m_width);
                            }
                        }
                    }  //case 25, 28
                break;
            case 4: if(floatLessEqual(b.position.position(0) + b.m_length, fp.coordinates.position(0) - abox.m_length))
//                    if(b.position.position(0) + b.m_length <= fp.coordinates.position(0) - abox.m_length)
                    {
                        if( floatLessEqual(b.position.position(1), u_y) && floatGreaterThan(b.position.position(1) + b.m_width, u_y) )
                        {
                            if( floatLessThan(u_x - (b.position.position(0) + b.m_length), distance_x))
                            {
                                distance_x = u_x - (b.position.position(0) + b.m_length);
                            }
                        }
                    } //case 17, 20
                    if( floatGreaterEqual(b.position.position(1), fp.coordinates.position(1) + abox.m_width))
                    {
                        if( floatLessEqual(b.position.position(0), u_x) && floatGreaterThan(b.position.position(0) + b.m_length, u_x) )
                        {
                            if( floatLessThan(b.position.position(1) - u_y - abox.m_width, distance_y))
                            {
                                distance_y = b.position.position(1) - u_y - abox.m_width;
                            }
                        }
                    } //case 29,32
                break;
            default: cout << "error in givePosition MARK:A" << endl;
        }
    }

    // ??ccx: After place the abox, its the same high, for the cases 17-32
    // The count_neighbours = lying_next when the same high
    if( floatEqual(b.position.position(2) + b.m_height, fp.coordinates.position(2) + abox.m_height) )
    {
        // case: 17,19,18,20
        if( floatEqual(b.position.position(0) + b.m_length, u_x))
        {
            if(b.position.position(1) <= u_y && b.position.position(1) + b.m_width > u_y && b.position.position(1) + b.m_width <= u_y + abox.m_width){
                count_neighbours += b.position.position(1) + b.m_width - u_y;
            }

            if(b.position.position(1) + b.m_width >= u_y + abox.m_width && b.position.position(1) >= u_y && b.position.position(1) < u_y + abox.m_width){
                count_neighbours += u_y + abox.m_width - b.position.position(1);
            }

            if(b.position.position(1) > u_y && b.position.position(1) + b.m_width < u_y + abox.m_width){
                count_neighbours += b.m_width;
            }

            if(b.position.position(1) < u_y && b.position.position(1) + b.m_width > u_y + abox.m_width){
                count_neighbours += abox.m_width;
            }
        }
        // case: 21,23,22,24
//        if(b.position.position(0) == u_x + abox.m_length)
        if( floatEqual(b.position.position(0), u_x + abox.m_length) )
        {
            if(b.position.position(1) <= u_y && b.position.position(1) + b.m_width > u_y && b.position.position(1) + b.m_width <= u_y + abox.m_width){
                count_neighbours += b.position.position(1) + b.m_width - u_y;
            }

            if(b.position.position(1) + b.m_width >= u_y + abox.m_width && b.position.position(1) >= u_y && b.position.position(1) < u_y + abox.m_width){
                count_neighbours += u_y + abox.m_width - b.position.position(1);
            }

            if(b.position.position(1) > u_y && b.position.position(1) + b.m_width < u_y + abox.m_width){
                count_neighbours += b.m_width;
            }

            if(b.position.position(1) < u_y && b.position.position(1) + b.m_width > u_y + abox.m_width){
                count_neighbours += abox.m_width;
            }
        }
        // case: 25,27,26,28
//        if(b.position.position(1) + b.m_width == u_y)
        if( floatEqual(b.position.position(1) + b.m_width, u_y) )
        {
            if(b.position.position(0) <= u_x && b.position.position(0) + b.m_length > u_x && b.position.position(0) + b.m_length <= u_x + abox.m_length){
                count_neighbours += b.position.position(0) + b.m_length - u_x;
            }

            if(b.position.position(0) + b.m_length >= u_x + abox.m_length && b.position.position(0) >= u_x && b.position.position(0) < u_x + abox.m_length){
                count_neighbours += u_x + abox.m_length - b.position.position(0);
            }

            if(b.position.position(0) > u_x && b.position.position(0) + b.m_length < u_x + abox.m_length){
                count_neighbours += b.m_length;
            }

            if(b.position.position(0) < u_x && b.position.position(0) + b.m_length > u_x + abox.m_length){
                count_neighbours += abox.m_length;
            }
        }
        // case: 29, 31, 30, 32
//        if(b.position.position(1) == u_y + abox.m_width)
        if( floatEqual(b.position.position(1), u_y + abox.m_width) )
        {
            if(b.position.position(0) <= u_x && b.position.position(0) + b.m_length > u_x && b.position.position(0) + b.m_length <= u_x + abox.m_length){
                count_neighbours += b.position.position(0) + b.m_length - u_x;
            }

            if(b.position.position(0) + b.m_length >= u_x + abox.m_length && b.position.position(0) >= u_x && b.position.position(0) < u_x + abox.m_length){
                count_neighbours += u_x + abox.m_length - b.position.position(0);
            }

            if(b.position.position(0) > u_x && b.position.position(0) + b.m_length < u_x + abox.m_length){
                count_neighbours += b.m_length;
            }

            if(b.position.position(0) < u_x && b.position.position(0) + b.m_length > u_x + abox.m_length){
                count_neighbours += abox.m_length;
            }
        }
    }

    return position;
}

// need get variables: helt, weight_div_temp, lying_next
// count_neighbours, distance_x, distance_y
double BinPackingPlanner::giveScore(Box &abox, FittingPoint &fp, Bin &abin, double &helt)
{
    helt = 0.0;
    double contact_area = 0.0;
    double score = 0.0;

    double lying_next = 0;
    double distance_x = abin.bin_length;
    double distance_y = abin.bin_width;
    double count_neighbours= 0.0;   /* neighbours with same height */

    int position = 0;
    double overlap = 0.0;

    // get the left down corner position of the box
    double u_x = fp.coordinates.position(0) + fp.direction_box_pos(0) * abox.m_length;
    double u_y = fp.coordinates.position(1) + fp.direction_box_pos(1) * abox.m_width;

    if(floatEqual(fp.coordinates.position(2), 0.0))
    {
        helt = abox.m_width * abox.m_length;
    }else{
        helt = 0.0;
    }

    // limit in the bin range
//    if(u_x < 0 || u_x+abox.m_length > abin.bin_length || u_y < 0 || u_y + abox.m_width > abin.bin_width
//            || fp.coordinates.position(2) < 0 || fp.coordinates.position(2) + abox.m_height > abin.bin_height)
    if(floatLessThan(u_x, 0) || floatGreaterThan(u_x+abox.m_length, abin.bin_length) ||
       floatLessThan(u_y, 0) || floatGreaterThan(u_y + abox.m_width,abin.bin_width) ||
       floatLessThan(fp.coordinates.position(2),0) || floatGreaterThan(fp.coordinates.position(2) + abox.m_height,abin.bin_height))
    {
        return -1;
    }

    //if the box is dangerous, it has to be placed in the edge
    if(abox.is_dangerous)
    {
        //min box length is 30cm
        if( u_x < bpa::Params::instance()->MIN_BOX_SIZE || (abin.bin_length-u_x-abox.m_length < bpa::Params::instance()->MIN_BOX_SIZE) || u_y < bpa::Params::instance()->MIN_BOX_SIZE || (abin.bin_width-u_y-abox.m_width < bpa::Params::instance()->MIN_BOX_SIZE))
        {}
        else{return -1;}
    }

    // return -1 if Box using fork and put high: z joint limit
    if( (fp.coordinates.position(2) > 2.25) && abox.tool_name == "cranepalletfork")
        return -1;

//    // return -1 if Box is too high
//    if(abox.m_height > 1.43)
//    {
//        return -1;
//    }
    /////////////////////////////////////////////////////////////////material
//    if( (abox.material == "drum" || abox.material == "Plastic") && (fp.coordinates.position(2) + abox.m_height) < 2.8) {
//        return -1;
//    }
//    if(abox.material == "styrofoam" && floatLessEqual(abox.m_mass, 50) && (fp.coordinates.position(2) + abox.m_height) < 2.0) {
//        return -1;
//    }

    //if it is small styrofoam box, we only can grasp in 90 degree because of the straps
    if(abox.m_length < 0.76)
    {
        if((abox.material == "styrofoam" || abox.material == "Styrofoam") && abox.tool_name == "foamgripper" && (!abox.is_rotated))
        {
            return -1;
        }
    }

    //hake for 0.8 boxes
    if((abox.material == "styrofoam" || abox.material == "Styrofoam") && abox.tool_name == "cranepalletfork" && (!abox.is_rotated))
    {
        return -1;
    }

    //hake for 0.8 wooden boxes, for tumcreate demo
    if(abox.m_name == "724114a3" && (!abox.is_rotated))
    {
        return -1;
    }
    /////////////////////////////////////////////////////////////////

    //    // return -1 if Box is a fragile item and is placed too low
    //    if(abox.is_fragile && (fp.coordinates.position(2) + abox.m_height) < 2.5)
    //    {
    //        return -1;
    //    }

    // return -1 if Box can not be stacked
    if( (abox.is_stackable==false) && (abin.bin_height - (fp.coordinates.position(2) + abox.m_height)) > 0.4 )
    {
//        std::cout << abox.m_name << " is not stackable!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
        return -1;
    }

    //get the helt
    Eigen::Matrix3d box_size;
    box_size << abox.m_length, 0, 0,
                0, abox.m_width,  0,
                0, 0, abox.m_height;
    Box new_box(abox.m_length, abox.m_width, abox.m_height, abox.m_mass, std::string("tempCollisionBox"), abox.box_labels);
    new_box.position.position = fp.coordinates.position + box_size * fp.direction_box_pos;

    // Return -1 if it collide with packed boxes
    if( abin.bulletPhysics->isColliding(new_box))
    {
       return -1;
    }

    for(Box b:abin.packed_boxes)
    {
        overlap = abin.bulletPhysics->getSupportArea(new_box, b);

        double area = abin.bulletPhysics->getContactArea(new_box, b);
        contact_area += area;

        // new_box on top of b
        if(floatEqual(b.position.position(2) + b.m_height, new_box.position.position(2)))
        {
            if(floatLessThan(overlap/(b.m_width * b.m_length), bpa::Params::instance()->HELT_RATE))
            {
                if (overlap != 0 && (b.material == "styrofoam" || b.material == "Styrofoam") && new_box.m_mass > 70)
                    return -1;
                if (overlap != 0 && (b.material == "carton" || b.material == "Carton") && new_box.m_mass > 70)
                    return -1;
            }
            helt += overlap;
        }

        position = givePosition(abox,fp,b,count_neighbours, distance_x, distance_y);  //get the count_neighbours, should be removed
        switch(position)
        {
            case 17: lying_next += b.position.position(1) + b.m_width - u_y;
                break;
            case 18: lying_next += b.m_width;
                break;
            case 19: lying_next += u_y + abox.m_width - b.position.position(1);
                break;
            case 20: lying_next += abox.m_width;
                break;
            case 21: lying_next += b.position.position(1) + b.m_width - u_y;
                break;
            case 22: lying_next += b.m_width;
                break;
            case 23: lying_next += u_y + abox.m_width - b.position.position(1);
                break;
            case 24: lying_next += abox.m_width;
                break;
            case 25: lying_next += b.position.position(0) + b.m_length - u_x;
                break;
            case 26: lying_next += b.m_length;
                break;
            case 27: lying_next += u_x + abox.m_length - b.position.position(0);
                break;
            case 28: lying_next += abox.m_length;
                break;
            case 29: lying_next += b.position.position(0) + b.m_length - u_x;
                break;
            case 30: lying_next += b.m_length;
                break;
            case 31: lying_next += u_x + abox.m_length - b.position.position(0);
                break;
            case 32: lying_next += abox.m_length;
                break;
            default: lying_next +=  0.0;
        }
    }

    // Check for border
    if( floatEqual(u_x, 0.0)){
        lying_next += abox.m_width;
    }
    if( floatEqual(u_y, 0.0)){
        lying_next += abox.m_length;
    }
    if( floatEqual(u_x + abox.m_length, abin.bin_length)){
        lying_next += abox.m_width;
    }

    if( floatEqual(u_y + abox.m_width, abin.bin_width) ){
        lying_next += abox.m_length;
    }

    //if comment this, packing results looks good, but the packed mass and used space is not so good.
    if(lying_next > abox.m_width + abox.m_length)
    {
        lying_next = abox.m_width + abox.m_length;
    }

    // Return -1 if not enough ground surface of the Box is supported
    if(floatLessThan( helt/(abox.m_width * abox.m_length), bpa::Params::instance()->HELT_RATE) )
    {
        return -1;
    }

    switch(fp.quadrant)
    {
        case 1: if(abin.bin_length - fp.coordinates.position(0) - abox.m_length < distance_x){
                    distance_x = abin.bin_length - fp.coordinates.position(0) - abox.m_length;
                }
                if(abin.bin_width - fp.coordinates.position(1) - abox.m_width < distance_y){
                    distance_y = abin.bin_width - fp.coordinates.position(1) - abox.m_width;
                }
            break;
        case 2: if(abin.bin_length - fp.coordinates.position(0) - abox.m_length < distance_x){
                    distance_x = abin.bin_length - fp.coordinates.position(0) - abox.m_length;
                }
                if(fp.coordinates.position(1) - abox.m_width < distance_y){
                    distance_y = fp.coordinates.position(1) - abox.m_width;
                }
            break;
        case 3: if(fp.coordinates.position(0) - abox.m_length < distance_x){
                    distance_x = fp.coordinates.position(0) - abox.m_length;
                }
                if(fp.coordinates.position(1) - abox.m_width < distance_y){
                    distance_y = fp.coordinates.position(1) - abox.m_width;
                }
            break;
        case 4: if(fp.coordinates.position(0) - abox.m_length < distance_x){
                    distance_x = fp.coordinates.position(0) - abox.m_length;
                }
                if(abin.bin_width - fp.coordinates.position(1) - abox.m_width < distance_y){
                    distance_y = abin.bin_width - fp.coordinates.position(1) - abox.m_width;
                }
            break;
    }
    // Increase Score if the Box would be placed near to another Box/border
    if(distance_x < abox.m_length / 4.0)
    {
        score += bpa::Params::instance()->W_PLACE_NEAR * (abox.m_length / 4.0 - distance_x) / (abox.m_length / 4.0);
    }
    // In Y-Direction:
    if(distance_y < abox.m_width / 4.0)
    {
        score += bpa::Params::instance()->W_PLACE_NEAR * (abox.m_width / 4.0 - distance_y) / (abox.m_width / 4.0);
    }

    // Increase Score if the Box will be placed in a bottom area
    score += bpa::Params::instance()->W_ITEM_IN_THE_BOTTOM_AREA * (1.0 / pow(abin.bin_height,2) * pow(fp.coordinates.position(2),2) - (2.0 / abin.bin_height * fp.coordinates.position(2))  + 1.0);

    // Increase Score if it helps to set the center of mass in the right position (only if mass > COM_MIN_MASS * average_mass)
    double tmpx = 0.0;
    double tmpy = 0.0;
    double tmpz = 0.0;
    tmpx = abin.getComDiffX(abox,fp);
    if(tmpx > 0){
        score += bpa::Params::instance()->W_COM * tmpx / abin.target_com.position(0);
    }
    tmpy = abin.getComDiffY(abox,fp);
    if(tmpy > 0){
        score += bpa::Params::instance()->W_COM * tmpy / abin.target_com.position(1);
    }
    tmpz = abin.getComDiffZ(abox,fp);
    if(tmpz > 0){
        score += bpa::Params::instance()->W_COM * tmpz / abin.target_com.position(2);
    }

    //Increasing score if Item is high and will be placed near to a border of the bin
    if(abox.m_height > abin.giveAverageHeightOfBoxes()){
        if(abin.getNearestDistanceToEdges(abox,fp) < 0.15){
            score += bpa::Params::instance()->W_HIGH_ITEMS_GOOD_PLACED;
        }
    }

    if(count_neighbours > abox.m_width + abox.m_length){
        count_neighbours = abox.m_width + abox.m_length;
    }
    // Increasing score if neighbours have the same height
    score += bpa::Params::instance()->NEIGHBOUR_CONSTANT * count_neighbours / (abox.m_width + abox.m_length);

    // Increasing score if Item is lying next to other boxes
    score += bpa::Params::instance()->W_ASSIGNMENT * lying_next / (abox.m_width + abox.m_length);

    // Increase Score if much of the ground surface of the Box is supported
    score += bpa::Params::instance()->W_SUPPORTED * helt/ (bpa::Params::instance()->HELT_RATE * (abox.m_width * abox.m_length));

    // Increase Score if much of the area (without helt) of the Box are contacted by other boxes
    if(floatEqual(u_x,0) || floatEqual(u_x+abox.m_length, abin.bin_length)) {contact_area += abox.m_width * abox.m_height;} //on the bin edges
    if(floatEqual(u_y,0) || floatEqual(u_y+abox.m_width, abin.bin_width)) {contact_area += abox.m_length * abox.m_height;}
    score += bpa::Params::instance()->W_CONTACT * contact_area/(2*(abox.m_width * abox.m_height) + 2*(abox.m_length * abox.m_height));

    // Increase Score if the box is reach the height of the bin limit
    if(floatEqual(fp.coordinates.position(2) + abox.m_height, abin.bin_height) )
    {
       score += bpa::Params::instance()->BIN_HEIGHT;
    }

    //hake for the small boxes
//    if( (abox.m_length < 0.35) && (abox.m_width < 0.35) && (fp.coordinates.position(2) + abox.m_height) < 2.6) { return -1;}
//    if( (abox.m_height < 0.43) && (fp.coordinates.position(2)) < 0.45) { return -1;}

//    if((abox.material == "wooden" || abox.material == "Wooden")) {score += 0.2;}

    return score;
}

double BinPackingPlanner::giveMixedScore(Box &abox, FittingPoint &fp, Bin &abin, double &helt)
{
    double score = giveScore(abox,fp,abin,helt);
    if(score == -1){
        return -1;
    }else
    {
        abox.setChooseScore();
        score += abox.choose_score;

        return score;
    }
}

bool BinPackingPlanner::addNextBoxToPackingConfigurationTwoFunctions(Bin& abin, HoldingPlatform &holding, double* cum_score)
{
    // Initializing some Variables
    Box box_to_pack(0,0,0,0,"",std::vector<std::string>());
    FittingPoint point_to_pack(0,0,0,0);
    double box_helt = 0.0;
    double max_score = 0; // Max score of the FPs
    double score = 0;
    bool solved = false;
    int iter = 0;
    double helt = 0.0;

    while(!solved)
    {
        if(holding.boxes_to_pack.size() == 0 || holding.it_choosing == holding.boxes_to_pack.end())
        {
            return false;
        }else
        {
            max_score = -1;

            box_to_pack = *(holding.it_choosing);

            // Calculate the score for each of the usuable FPs and select the FP with the highest score
            // Scores can not be stored because they depend on the boxes already packed into the bin
            for(FittingPoint fp : abin.fitting_points)
            {
                score = giveScore(box_to_pack,fp,abin/*,holding*/, helt);
                if(score>=max_score){
                    max_score = score;
                    point_to_pack = fp;
                    box_helt = helt;
                }
            }

            if(max_score >= 0)
            {
                // Look if the Box does fit into the chosen FP
                // If it fits add it to the bin, remove it from the holding platform and calculate the new FPs
                abin.addNewBox(box_to_pack,point_to_pack,holding,box_helt);
                holding.moveBox(box_to_pack.getId());
                holding.it_choosing = holding.boxes_to_pack.begin();
                *cum_score += max_score;
                return true;
            }else{
                holding.it_choosing++;
            }
        }

        if(iter>5000){
            cout << "Terminated by max_iter" << endl;
            return false;
        }
        iter++;
    }

    return false;
}

bool BinPackingPlanner::addNexBoxToPackingConfigurationOneFunction(Bin& abin, HoldingPlatform &holding, double* cum_score)
{
    // Initializing some Variables
    Box box_to_pack(0,0,0,0,"",std::vector<std::string>());
    FittingPoint point_to_pack(0,0,0,1);
    double box_helt = 0.0;
    double max_score;
    int max_score_fp_id = -1;
    int max_score_hbox_id = -1;

//    std::cout << "holding.boxes_to_pack = " << holding.boxes_to_pack.size() <<std::endl;

    if(holding.boxes_to_pack.size() == 0)
    {
        std::cout << "There is no boxes in Measurement Area to pack !!!\n";
        return false;
    }
    else
    {
        max_score = -1;
        // Calculate the maximal mixed score:
        //		holding.it_btp = holding.boxes_to_pack.begin();
        std::vector<double> scores;
        std::vector<double> helts;
        scores.resize(holding.boxes_to_pack.size()*abin.fitting_points.size());
        helts.resize(holding.boxes_to_pack.size()*abin.fitting_points.size());
#pragma omp parallel for collapse (2) shared(scores, helts)
        for(size_t holding_box_id = 0; holding_box_id < holding.boxes_to_pack.size();++holding_box_id)
        {
            for(size_t fitting_point_id = 0;fitting_point_id < abin.fitting_points.size();++fitting_point_id)
            {
                FittingPoint fp = abin.fitting_points[fitting_point_id];
                double helt_output = 0.0;
                scores[holding_box_id*abin.fitting_points.size()+fitting_point_id] = giveMixedScore(holding.boxes_to_pack[holding_box_id],fp,abin/*,holding*/, helt_output);
                helts[holding_box_id*abin.fitting_points.size()+fitting_point_id] = helt_output;
            }
        }
        for(size_t holding_box_id = 0; holding_box_id < holding.boxes_to_pack.size();holding_box_id++)
        {
            for(size_t fitting_point_id = 0;fitting_point_id < abin.fitting_points.size();fitting_point_id++)
            {
                if(scores[holding_box_id*abin.fitting_points.size()+fitting_point_id]>max_score)
                {
                    max_score = scores[holding_box_id*abin.fitting_points.size()+fitting_point_id];
                    max_score_fp_id = fitting_point_id;
                    max_score_hbox_id = holding_box_id;
                }
            }
        }

        // Only proceed if there is a positive Mixed_Score
        if(max_score >= 0)
        {
            point_to_pack = abin.fitting_points[max_score_fp_id];
            box_to_pack = holding.boxes_to_pack[max_score_hbox_id];
            box_helt = helts[max_score_hbox_id*abin.fitting_points.size()+max_score_fp_id];
#ifdef DEBUG_PRINT
            std::cout << "Start ---------------------------ID "<< box_to_pack.m_id << " "<< box_to_pack.m_name << "  L= "<< box_to_pack.m_length << "  W="<< box_to_pack.m_width << "  H="<< box_to_pack.m_height << "  mass= "<< box_to_pack.m_mass <<" at position " << point_to_pack.coordinates.position.transpose() <<  " rotated="<< box_to_pack.is_rotated <<std::endl;
            std::cout << "****** support helt =  " << box_helt << ";   score = " << max_score << std::endl;
#endif
            // Look if the Box does fit into the chosen FP
            // If it fits add it to the bin, remove it from the holding platform and calculate the new FPs
            abin.addNewBox(box_to_pack,point_to_pack,holding,box_helt);
            holding.moveBox(box_to_pack.getId());
            *cum_score += max_score;

#ifdef DEBUG_PRINT
            std::cout << box_to_pack.m_name << " has supported boxes = " << box_to_pack.support_boxes.size() << std::endl;
            for(int c=0; c < box_to_pack.support_boxes.size(); c++)
            {
                supportingBox sBox;
                sBox = box_to_pack.support_boxes[c];
                std::cout << c << ". supporting by " << sBox.uuid << " with helt = " << sBox.helt << std::endl;
            }
#endif

//            std::cout << "--------------------------------------------------------The pallet has fitting points: " << abin.fitting_points.size() << std::endl;
            return true;
        }else
        {
            std::cout << "No boxes' score > 0 !!!\n";
            std::cout << "Pallet packed boxes = " << abin.packed_boxes.size() << ", fp = " <<abin.fitting_points.size() <<std::endl;
            return false;
        }
    }
}


}
