#include <algorithm>
#include <numeric>
#include <sstream>

#include "wsg_utils.h"


namespace wsg
{

    std::string stringify(const bpp_actor::Actor &actor)
    {
        return __stringify_actor__(actor);
    }


    std::string stringify(const ActorVector &actors)
    {
        // get string of each actor
        std::vector<std::string> str_of_actors;
        str_of_actors.reserve(actors.size());
        std::transform(std::begin(actors),
                std::end(actors),
                std::back_inserter(str_of_actors),
                __stringify_actor__);
        // concatenate to one string and return
        return std::accumulate(std::begin(str_of_actors), std::end(str_of_actors), std::string(""));
    }


    std::string stringify(const WorldStateMap &items)
    {
        return stringify(get_register_as_list(items));
    }

};


