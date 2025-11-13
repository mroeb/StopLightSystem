//
// Created by ROEBFI on 11/13/2025.
//

#ifndef STOPLIGHTSYSTEM_CROSSOVER_HPP
#define STOPLIGHTSYSTEM_CROSSOVER_HPP
#include "StopLight.hpp"


class CrossOver {

public:
    CrossOver();
    ~CrossOver() = default;

    void addStopLight(StopLight* light);


private:
    std::vector<StopLight*> m_Lights{};

};


#endif //STOPLIGHTSYSTEM_CROSSOVER_HPP
