#pragma once

#include <string>
#include "main.h"

namespace LVGL_screen {
    extern lv_obj_t * odom_label;
    extern lv_obj_t * temp_label;
    extern int autonID;
    void labelUpdate();
    void main();
}