#include "liblvgl/extra/widgets/tabview/lv_tabview.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_style.h"
#include "liblvgl/widgets/lv_btnmatrix.h"
#include "liblvgl/widgets/lv_img.h"
#include "liblvgl/widgets/lv_label.h"
#include "main.h"
#include "liblvgl/lvgl.h"
#include "pros/misc.hpp"
#include "screen.h"

LV_IMG_DECLARE(z2);
LV_IMG_DECLARE(z2p);

namespace LVGL_screen {
    lemlib::Pose pose = lemlib::Pose(0,0,0);

    static lv_style_t btn_theme;
    static lv_style_t tab_theme;
    static lv_style_t label_theme;

    lv_obj_t * odom_label;
    lv_obj_t * temp_label;
    lv_obj_t * battery_label;
    lv_obj_t * auton_btnm;
    lv_obj_t * auton_btnm_label;

    const char * odom = "X: 0 Y: 0";

    int autonID = 0;
    int side = 1;
    bool skills = false;

    int m1 = 0;
    int m2 = 0;
    int m3 = 0;
    int m4 = 0;
    int m5 = 0;
    int m6 = 0;

    lv_coord_t tab = 60;

    const char * auton_map[] = {"Right", "Left", "None", NULL};

    static void auton_handler(lv_event_t * e){
        autonID = lv_btnmatrix_get_selected_btn(lv_event_get_target(e)) + 1; //Negative is red, positive is blue
        if (autonID >= 65535){
            autonID = 0; 
        }
    }

    static void auton_run_handler(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        lv_obj_t * obj = lv_event_get_target(e);
        if(code == LV_EVENT_VALUE_CHANGED) {
            LV_UNUSED(obj);
            side = (lv_obj_has_state(obj, LV_STATE_CHECKED)) ? 1 : -1; //Not Checked Red, Checked Blue
        }
    }

    static void skills_run_handler(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        lv_obj_t * obj = lv_event_get_target(e);
        if(code == LV_EVENT_VALUE_CHANGED) {
            LV_UNUSED(obj);
            skills = (lv_obj_has_state(obj, LV_STATE_CHECKED)) ? true : false; //Not Checked Red, Checked Blue
        }
    }

    void updateOdomLabel(float x, float y, float theta){
        lv_label_set_text_fmt(odom_label, "X: %d, Y: %d, Theta: %d", x, y, theta);
        pros::delay(50);
    }

    void labelUpdate(){
        while(true){
            lv_label_set_text_fmt(temp_label, "Motor Temps: %d, %d, %d, %d, %d, %d", m1, m2, m3, m4, m5, m6);
            lv_label_set_text_fmt(battery_label, "Battery Cap: %d%, Curr: %dmA, Temp: %dc, Volt: %dmV", (int)pros::battery::get_capacity(), pros::battery::get_current(), (int)pros::battery::get_temperature(), pros::battery::get_voltage());
            lv_label_set_text_fmt(auton_btnm_label, "Auton: %d", autonID * side);
            pros::Task::delay(50);
        }
    }

    void main(){
        lv_obj_t * tabView = lv_tabview_create(lv_scr_act(), LV_DIR_LEFT, tab);
        lv_obj_t * auton_tab = lv_tabview_add_tab(tabView, "Auton");
        lv_obj_t * info_tab = lv_tabview_add_tab(tabView, "Info");

        lv_obj_t * tab_btns = lv_tabview_get_tab_btns(tabView);
        lv_obj_set_style_bg_color(tab_btns, lv_palette_darken(LV_PALETTE_GREY, 3), 0);
        lv_obj_set_style_text_color(tab_btns, lv_palette_lighten(LV_PALETTE_GREY, 5), 0);
        lv_obj_set_style_border_side(tab_btns, LV_BORDER_SIDE_RIGHT, LV_PART_ITEMS | LV_STATE_CHECKED);
        
        lv_style_init(&tab_theme);
        lv_style_set_bg_color(&tab_theme, lv_color_make(0,0,0));

        lv_style_init(&label_theme);
        lv_style_set_text_color(&label_theme, lv_color_make(255,255,0));
        lv_style_set_text_font(&label_theme, &lv_font_montserrat_12);
        
        lv_style_init(&btn_theme);
        lv_style_set_bg_color(&btn_theme, lv_color_make(10,10,10));
        lv_style_set_border_color(&btn_theme, lv_color_make(255,255,0));
        lv_style_set_border_width(&btn_theme, 2);
        lv_style_set_radius(&btn_theme, 10);
        lv_style_set_text_color(&btn_theme, lv_color_make(255,255,0));

        auton_btnm = lv_btnmatrix_create(auton_tab);
        lv_obj_align(auton_btnm, LV_ALIGN_LEFT_MID, 0, 0);
        lv_obj_add_style(auton_btnm, &btn_theme, 0);
        lv_obj_add_event_cb(auton_btnm, auton_handler, LV_EVENT_ALL, NULL);
        lv_btnmatrix_set_map(auton_btnm, auton_map);

        lv_obj_t * auton_switch = lv_switch_create(auton_tab);
        lv_obj_align(auton_switch, LV_ALIGN_RIGHT_MID, 0, 0);
        lv_obj_set_size(auton_switch, 120, 60);
        lv_obj_set_style_bg_color(auton_switch, lv_color_make(255, 0, 0), LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(auton_switch, lv_color_make(0, 0, 255), LV_STATE_DISABLED);
        lv_obj_add_event_cb(auton_switch, auton_run_handler, LV_EVENT_ALL, NULL);

        lv_obj_t * skills_switch = lv_switch_create(auton_tab);
        lv_obj_align(skills_switch, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_size(skills_switch, 60, 30);
        lv_obj_set_style_bg_color(skills_switch, lv_color_make(100, 100, 100), LV_STATE_DISABLED);
        lv_obj_set_style_bg_color(skills_switch, lv_color_make(0, 255, 0), LV_STATE_DEFAULT);
        lv_obj_add_event_cb(skills_switch, skills_run_handler, LV_EVENT_ALL, NULL);

        auton_btnm_label = lv_label_create(auton_tab);
        lv_obj_add_style(auton_btnm_label, &label_theme, 0);
        lv_obj_align(auton_btnm_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);
        lv_label_set_text(auton_btnm_label, "");

        // lv_obj_t * image_obj = lv_img_create(info_tab);
        // lv_obj_set_size(image_obj, 120, 120); // {250,207 z2} {120,120 z2p}
        // lv_obj_align(image_obj, LV_ALIGN_CENTER, 0, 0);
        // lv_img_set_src(image_obj, &z2);

        odom_label = lv_label_create(info_tab);
        lv_obj_add_style(odom_label, &label_theme, 0);
        lv_obj_align(odom_label, LV_ALIGN_TOP_LEFT, 0, 0);
        
        temp_label = lv_label_create(info_tab);
        lv_obj_add_style(temp_label, &label_theme, 0);
        lv_obj_align(temp_label, LV_ALIGN_TOP_LEFT, 0, -15);

        battery_label = lv_label_create(info_tab);
        lv_obj_align(battery_label, LV_ALIGN_TOP_LEFT, 0, 30);
        lv_obj_add_style(battery_label, &label_theme, 0);

        pros::Task labelTask(labelUpdate, "Label");
    }
}