#pragma once

#include <string>
#include "main.h"

namespace LVGL_screen {
    extern lv_obj_t * odom_label;
    extern lv_obj_t * temp_label;
    extern int autonID;
    extern int side;
    extern bool skills;
    void labelUpdate();
    void main();
    void updateOdomLabel(float x, float y, float theta);
}