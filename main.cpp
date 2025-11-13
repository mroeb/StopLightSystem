#include <iostream>

#include "src/core/CrossOver.hpp"

int main() {

    CrossOver cross_over{};

    StopLight stop_light1{};

    cross_over.addStopLight(&stop_light1);

    stop_light1.showCurrent();



    return 0;
}
