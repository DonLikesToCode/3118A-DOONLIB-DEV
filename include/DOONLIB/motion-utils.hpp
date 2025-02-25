#pragma once

#include "vex.h"

namespace doon_utils {


    template <typename cmd>

    double sign(cmd given) { return (cmd(0) < given) - (given < cmd(0)); }

    template <typename cmd>

    double clamp(cmd given, cmd min, cmd max) {

        if (given > max) { return max; }
        else if (given < min) { return min; }
        else { return given; }

    }

    template <typename degrees>

    double degToRad(degrees given) { return M_PI/180; }

    template <typename radians>

    double radToDeg(radians given) { return 180/M_PI; } 

}