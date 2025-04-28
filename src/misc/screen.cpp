#include "liblvgl/extra/widgets/tabview/lv_tabview.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_style.h"
#include "liblvgl/widgets/lv_btnmatrix.h"
#include "liblvgl/widgets/lv_img.h"
#include "liblvgl/widgets/lv_label.h"
#include "lemlib/api.hpp"
#include "liblvgl/lvgl.h"
#include "pros/misc.hpp"
#include "screen.h"

// LV_IMG_DECLARE(z2);
LV_IMG_DECLARE(z2p);
LV_IMG_DECLARE(oiia);

namespace LVGL_screen {
    lemlib::Pose pose = lemlib::Pose(0,0,0);

    static lv_style_t btn_theme;
    static lv_style_t tab_theme;
    static lv_style_t label_theme;

    lv_obj_t * odom_label;
    lv_obj_t * temp_label;
    lv_obj_t * battery_label;

    lv_obj_t * blue_auton_btnm;
    lv_obj_t * red_auton_btnm;

    lv_obj_t * auton_btnm_label;

    const char * odom = "X: 0 Y: 0";

    int autonID = 1;

    int m1 = 0;
    int m2 = 0;
    int m3 = 0;
    int m4 = 0;
    int m5 = 0;
    int m6 = 0;

    lv_coord_t tab = 60;

    const char * blue_auton_map[] = {"Blue\nGoal\nRush", "Blue 4\nRing\nMiddle", "Blue 4\nRing\nSweep", "4", "5", NULL}; // FIX ME
    const char * red_auton_map[] = {"Red\nGoal\nRush", "Red 4\nRing\nMiddle", "Red 4\nRing\nSweep", "4", "5", NULL};

    static void auton_blue_handler(lv_event_t * e){
        autonID = lv_btnmatrix_get_selected_btn(lv_event_get_target(e)) + 1; //Negative is red, positive is blue
        if (autonID >= 65535){
            autonID = 1; 
        }
    }

    static void auton_red_handler(lv_event_t * e){
        autonID = -1*lv_btnmatrix_get_selected_btn(lv_event_get_target(e)) - 1; //Negative is red, positive is blue
        if (autonID <= -65535){
            autonID = -1; 
        }
    }

    static void skills_run_handler(lv_event_t * e)
    {
        autonID = 0;
    }

    void updateOdomLabel(float x, float y, float theta){
        lv_label_set_text_fmt(odom_label, "X: %d, Y: %d, Theta: %d", x, y, theta);
        pros::delay(50);
    }

    void labelUpdate(){
        while(true){
            lv_label_set_text_fmt(temp_label, "Motor Temps: %d, %d, %d, %d, %d, %d", m1, m2, m3, m4, m5, m6);
            lv_label_set_text_fmt(battery_label, "Battery Cap: %d%, Curr: %dmA, Temp: %dc, Volt: %dmV", (int)pros::battery::get_capacity(), pros::battery::get_current(), (int)pros::battery::get_temperature(), pros::battery::get_voltage());
            lv_label_set_text_fmt(auton_btnm_label, "Auton: %d", autonID);
            pros::Task::delay(50);
        }
    }

    void main(){
        lv_obj_t * tabView = lv_tabview_create(lv_scr_act(), LV_DIR_LEFT, tab);
        lv_obj_t * blue_auton_tab = lv_tabview_add_tab(tabView, "Blue Autons");
        lv_obj_t * red_auton_tab = lv_tabview_add_tab(tabView, "Red Autons");
        lv_obj_t * skills_tab = lv_tabview_add_tab(tabView, "Skills");
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

        blue_auton_btnm = lv_btnmatrix_create(blue_auton_tab);
        lv_obj_align(blue_auton_btnm, LV_ALIGN_LEFT_MID, 0, 0);
        lv_obj_add_style(blue_auton_btnm, &btn_theme, 0);
        lv_obj_add_event_cb(blue_auton_btnm, auton_blue_handler, LV_EVENT_ALL, NULL);
        lv_btnmatrix_set_map(blue_auton_btnm, blue_auton_map);
        lv_obj_set_size(blue_auton_btnm, 400, 200);

        red_auton_btnm = lv_btnmatrix_create(red_auton_tab);
        lv_obj_align(red_auton_btnm, LV_ALIGN_LEFT_MID, 0, 0);
        lv_obj_add_style(red_auton_btnm, &btn_theme, 0);
        lv_obj_add_event_cb(red_auton_btnm, auton_red_handler, LV_EVENT_ALL, NULL);
        lv_btnmatrix_set_map(red_auton_btnm, red_auton_map);
        lv_obj_set_size(red_auton_btnm, 400, 200);

        lv_obj_t * skills_btn = lv_btn_create(skills_tab);
        lv_obj_align(skills_btn, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_size(skills_btn, 100, 100);
        lv_obj_add_style(skills_btn, &btn_theme, 0);
        lv_obj_add_event_cb(skills_btn, skills_run_handler, LV_EVENT_ALL, NULL);

        lv_obj_t * image_obj = lv_img_create(skills_btn);
        lv_obj_set_size(image_obj, 64, 62); // {250,207 z2} {120,120 z2p}
        lv_obj_align(image_obj, LV_ALIGN_CENTER, 0, 0);
        lv_img_set_src(image_obj, &oiia);

        auton_btnm_label = lv_label_create(lv_scr_act());
        lv_obj_add_style(auton_btnm_label, &label_theme, 0);
        lv_obj_align(auton_btnm_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);
        lv_label_set_text(auton_btnm_label, "");

        // lv_obj_t * image_obj2 = lv_img_create(info_tab);
        // lv_obj_set_size(image_obj2, 250, 207); // {250,207 z2} {120,120 z2p}
        // lv_obj_align(image_obj2, LV_ALIGN_CENTER, 0, 0);
        // lv_img_set_src(image_obj2, &z2);

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