//
// Created by ROEBFI on 11/13/2025.
//

#ifndef STOPLIGHTSYSTEM_STOPLIGHT_HPP
#define STOPLIGHTSYSTEM_STOPLIGHT_HPP
#include <cstdint>
#include <map>
#include <queue>

#include "Pilo.hpp"


enum class Phase {
    Red = 1 << 0,
    Yellow = 1 << 1,
    Green = 1 << 2,

    Stop = Red,
    PrepareGo = Red | Yellow,
    PrepareStop = Yellow,
    Go = Green
};

constexpr Phase operator|(Phase lhs, Phase rhs) {
    using T = std::underlying_type_t<Phase>;
    return static_cast<Phase>(static_cast<T>(lhs) | static_cast<T>(rhs));
}
constexpr Phase operator&(Phase lhs, Phase rhs) {
    using T = std::underlying_type_t<Phase>;
    return static_cast<Phase>(static_cast<T>(lhs) & static_cast<T>(rhs));
}

constexpr Phase &operator|=(Phase &lhs, const Phase rhs) {
    lhs = lhs | rhs;
    return lhs;
}

struct PinColor {
    Phase color{};
    uint8_t pin{};
};
class StopLight {

public:
    StopLight();
    ~StopLight() = default;

    void addColorPin(PinColor pin_color);

    void triggerNextPhase();

    void showCurrent();

    void queuePhase(Phase phase);

private:

    std::queue<Phase> m_PhaseQueue{};
    std::map<Phase, uint8_t> m_PinColorMap{};

    Phase m_CurrentPhase{};

    Pilo::GPIO m_Gpio;

};




#endif //STOPLIGHTSYSTEM_STOPLIGHT_HPP
