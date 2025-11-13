//
// Created by ROEBFI on 11/13/2025.
//

#include "StopLight.hpp"

#include <bits/this_thread_sleep.h>

#include "Pilo.hpp"

StopLight::StopLight() : m_Gpio("/dev/gpiochip0", "StioLightSystem") {
    m_Gpio.add_lines<Pilo::Direction::Output, 17>();
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
    while (true) {
        std::this_thread::sleep_for(1s);
        m_Gpio.write(17, true);
        std::this_thread::sleep_for(1s);
        m_Gpio.write(17, false);

    }

    if (checkIfContains(m_CurrentPhase, Phase::Stop))
        if (m_PinColorMap.contains(Phase::Stop));


    if (checkIfContains(m_CurrentPhase, Phase::PrepareGo));
    if (checkIfContains(m_CurrentPhase, Phase::PrepareStop));
    if (checkIfContains(m_CurrentPhase, Phase::Go));

}



void StopLight::queuePhase(const Phase phase) {
    m_PhaseQueue.push(phase);
}

