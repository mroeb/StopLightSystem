//
// Created by ROEBFI on 11/13/2025.
//

#include "CrossOver.hpp"

CrossOver::CrossOver() : m_Lights(4) {

}

void CrossOver::addStopLight(StopLight *light) {
    m_Lights.emplace_back(light);
}

