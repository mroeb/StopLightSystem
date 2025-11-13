//
// Created by ROEBFI on 11/13/2025.
//

#include "StopLight.hpp"

#include <bits/this_thread_sleep.h>

#include "Pilo.hpp"

StopLight::StopLight() : m_Gpio("/dev/gpiochip0", "StioLightSystem") {
    m_Gpio.add_lines<Pilo::Direction::Output, 17, 27, 22>();
}


void StopLight::addColorPin(PinColor pin_color) {
    m_PinColorMap.insert({pin_color.color, pin_color.pin});
}

void StopLight::triggerNextPhase() {
    m_CurrentPhase = m_PhaseQueue.back();
    m_PhaseQueue.pop();
}

constexpr bool checkIfContains(const Phase currentPhase, const Phase phase) {
    return (currentPhase & phase) == phase;
}
void StopLight::showCurrent() {

    using namespace std::chrono_literals;
    m_Gpio.write<17>(true);
    m_Gpio.write<22>(true);
    m_Gpio.write<27>(true);

}



void StopLight::queuePhase(const Phase phase) {
    m_PhaseQueue.push(phase);
}

