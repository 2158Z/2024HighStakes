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
#include "main.h"
#include "screen.h"

LV_IMG_DECLARE(z2);
LV_IMG_DECLARE(z2p);

namespace screen {
    double l_p = 1;
    double l_i = 2;
    double l_d = 3;

    double a_p = 1;
    double a_i = 2;
    double a_d = 3;

    static lv_style_t btn_theme;
    static lv_style_t tab_theme;
    static lv_style_t label_theme;

    lv_obj_t * odom_label;
    lv_obj_t * temp_label;
    lv_obj_t * battery_label;
    lv_obj_t * auton_btnm;
    lv_obj_t * auton_btnm_label;

    static lv_obj_t * lp_spinbox;
    static lv_obj_t * li_spinbox;
    static lv_obj_t * ld_spinbox;
    static lv_obj_t * ap_spinbox;
    static lv_obj_t * ai_spinbox;
    static lv_obj_t * ad_spinbox;

    const char * odom = "X: 0 Y: 0";

    int autonID = 0;

    int m1 = 0;
    int m2 = 0;
    int m3 = 0;
    int m4 = 0;
    int m5 = 0;
    int m6 = 0;

    lv_coord_t tab = 60;

    const char * auton_map[] = {"Preload", "Auton2", "Auton3", NULL};

    static void auton_handler(lv_event_t * e){
        autonID = lv_btnmatrix_get_selected_btn(lv_event_get_target(e));
        if (autonID == 65535){
            autonID = 0;
        }
    }

    static void auton_run_handler(lv_event_t * e){
        autonomous();
    }

    void labelUpdate(){
        autonID = 0.0;
        while(true){
            lv_label_set_text_fmt(temp_label, "Motor Temps: %d, %d, %d, %d, %d, %d", m1, m2, m3, m4, m5, m6);
            lv_label_set_text(odom_label, odom);
            lv_label_set_text_fmt(battery_label, "Battery Cap: %d%, Curr: %dmA, Temp: %dc, Volt: %dmV", (int)pros::battery::get_capacity(), pros::battery::get_current(), (int)pros::battery::get_temperature(), pros::battery::get_voltage());
            lv_label_set_text_fmt(auton_btnm_label, "Auton: %d", autonID + 1);
            pros::Task::delay(50);
        }
    }

    void pidUpdateTask(){
        while(true){
            l_p = lv_spinbox_get_value(lp_spinbox);
            l_i = lv_spinbox_get_value(li_spinbox);
            l_d = lv_spinbox_get_value(ld_spinbox);

            a_p = lv_spinbox_get_value(ap_spinbox);
            a_i = lv_spinbox_get_value(ai_spinbox);
            a_d = lv_spinbox_get_value(ad_spinbox);
            pros::Task::delay(50);
        }
    }

    
    static void lv_spinbox_increment_event_cb_p(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
            lv_spinbox_increment(lp_spinbox);
        }
    }

    static void lv_spinbox_decrement_event_cb_p(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
            lv_spinbox_decrement(lp_spinbox);
        }
    }

    static void lv_spinbox_increment_event_cb_i(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
            lv_spinbox_increment(li_spinbox);
        }
    }

    static void lv_spinbox_decrement_event_cb_i(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
            lv_spinbox_decrement(li_spinbox);
        }
    }

        static void lv_spinbox_increment_event_cb_d(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
            lv_spinbox_increment(ld_spinbox);
        }
    }

    static void lv_spinbox_decrement_event_cb_d(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
            lv_spinbox_decrement(ld_spinbox);
        }
    }

    void main(){
        lv_obj_t * tabView = lv_tabview_create(lv_scr_act(), LV_DIR_LEFT, tab);
        lv_obj_t * auton_tab = lv_tabview_add_tab(tabView, "Auton");
        lv_obj_t * info_tab = lv_tabview_add_tab(tabView, "Info");
        lv_obj_t * lpid_tab = lv_tabview_add_tab(tabView, "Lateral PID");
        lv_obj_t * apid_tab = lv_tabview_add_tab(tabView, "Angular PID");

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

        lv_obj_t * auton_btn = lv_btn_create(auton_tab);
        lv_obj_align(auton_btn, LV_ALIGN_RIGHT_MID, 0, 0);
        lv_obj_add_style(auton_btn, &btn_theme, 0);
        lv_obj_add_event_cb(auton_btn, auton_run_handler, LV_EVENT_ALL, NULL);

        lv_obj_t * auton_btn_label = lv_label_create(auton_btn);
        lv_obj_add_style(auton_btn_label, &label_theme, 0);
        lv_label_set_text(auton_btn_label, "Run Auton");

        auton_btnm_label = lv_label_create(auton_tab);
        lv_obj_add_style(auton_btnm_label, &label_theme, 0);
        lv_obj_align(auton_btnm_label, LV_ALIGN_BOTTOM_MID, 0, 0);
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

        lp_spinbox = lv_spinbox_create(lpid_tab);
        lv_spinbox_set_range(lp_spinbox, -10000, 20000);
        lv_spinbox_step_prev(lp_spinbox);
        lv_spinbox_set_digit_format(lp_spinbox, 5, 2);
        lv_obj_set_width(lp_spinbox, 100);
        lv_obj_align(lp_spinbox, LV_ALIGN_TOP_MID, -20, 0);

        lv_coord_t h = lv_obj_get_height(lp_spinbox);

        lv_obj_t * lp_btn = lv_btn_create(lpid_tab);
        lv_obj_set_size(lp_btn, h, h);
        lv_obj_align_to(lp_btn, lp_spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
        lv_obj_set_style_bg_img_src(lp_btn, LV_SYMBOL_PLUS, 0);
        lv_obj_add_event_cb(lp_btn, lv_spinbox_increment_event_cb_p, LV_EVENT_ALL,  NULL);

        lp_btn = lv_btn_create(lpid_tab);
        lv_obj_set_size(lp_btn, h, h);
        lv_obj_align_to(lp_btn, lp_spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        lv_obj_set_style_bg_img_src(lp_btn, LV_SYMBOL_MINUS, 0);
        lv_obj_add_event_cb(lp_btn, lv_spinbox_decrement_event_cb_p, LV_EVENT_ALL, NULL);

        lv_obj_t * lp_label = lv_label_create(lpid_tab);
        lv_obj_align_to(lp_label, lp_btn, LV_ALIGN_OUT_LEFT_MID, 10, 0);
        lv_label_set_text(lp_label, "P");
        lv_obj_add_style(lp_label, &label_theme, 0);

        li_spinbox = lv_spinbox_create(lpid_tab);
        lv_spinbox_set_range(li_spinbox, -10000, 20000);
        lv_spinbox_step_prev(li_spinbox);
        lv_spinbox_set_digit_format(li_spinbox, 5, 2);
        lv_obj_set_width(li_spinbox, 100);
        lv_obj_align(li_spinbox, LV_ALIGN_CENTER, -20, 0);

        lv_obj_t * li_btn = lv_btn_create(lpid_tab);
        lv_obj_set_size(li_btn, h, h);
        lv_obj_align_to(li_btn, li_spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
        lv_obj_set_style_bg_img_src(li_btn, LV_SYMBOL_PLUS, 0);
        lv_obj_add_event_cb(li_btn, lv_spinbox_increment_event_cb_i, LV_EVENT_ALL,  NULL);

        li_btn = lv_btn_create(lpid_tab);
        lv_obj_set_size(li_btn, h, h);
        lv_obj_align_to(li_btn, li_spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        lv_obj_set_style_bg_img_src(li_btn, LV_SYMBOL_MINUS, 0);
        lv_obj_add_event_cb(li_btn, lv_spinbox_decrement_event_cb_i, LV_EVENT_ALL, NULL);

        lv_obj_t * li_label = lv_label_create(lpid_tab);
        lv_obj_align_to(li_label, li_btn, LV_ALIGN_OUT_LEFT_MID, 10, 0);
        lv_label_set_text(li_label, "I");
        lv_obj_add_style(li_label, &label_theme, 0);

        ld_spinbox = lv_spinbox_create(lpid_tab);
        lv_spinbox_set_range(ld_spinbox, -10000, 20000);
        lv_spinbox_step_prev(ld_spinbox);
        lv_obj_set_width(ld_spinbox, 100);
        lv_spinbox_set_digit_format(ld_spinbox, 5, 2);
        lv_obj_align(ld_spinbox, LV_ALIGN_BOTTOM_MID, -20, 0);

        lv_obj_t * ld_btn = lv_btn_create(lpid_tab);
        lv_obj_set_size(ld_btn, h, h);
        lv_obj_align_to(ld_btn, ld_spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
        lv_obj_set_style_bg_img_src(ld_btn, LV_SYMBOL_PLUS, 0);
        lv_obj_add_event_cb(ld_btn, lv_spinbox_increment_event_cb_d, LV_EVENT_ALL,  NULL);

        ld_btn = lv_btn_create(lpid_tab);
        lv_obj_set_size(ld_btn, h, h);
        lv_obj_align_to(ld_btn, ld_spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        lv_obj_set_style_bg_img_src(ld_btn, LV_SYMBOL_MINUS, 0);
        lv_obj_add_event_cb(ld_btn, lv_spinbox_decrement_event_cb_d, LV_EVENT_ALL, NULL);

        lv_obj_t * ld_label = lv_label_create(lpid_tab);
        lv_obj_align_to(ld_label, ld_btn, LV_ALIGN_OUT_LEFT_MID, 10, 0);
        lv_label_set_text(ld_label, "D");
        lv_obj_add_style(ld_label, &label_theme, 0);

        lv_spinbox_set_value(lp_spinbox, l_p);
        lv_spinbox_set_value(li_spinbox, l_i);
        lv_spinbox_set_value(ld_spinbox, l_d);

        ap_spinbox = lv_spinbox_create(apid_tab);
        lv_spinbox_set_range(ap_spinbox, -10000, 20000);
        lv_spinbox_step_prev(ap_spinbox);
        lv_obj_set_width(ap_spinbox, 100);
        lv_obj_align(ap_spinbox, LV_ALIGN_TOP_MID, -20, 0);
        lv_spinbox_set_digit_format(ap_spinbox, 5, 2);

        lv_obj_t * ap_btn = lv_btn_create(apid_tab);
        lv_obj_set_size(ap_btn, h, h);
        lv_obj_align_to(ap_btn, ap_spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
        lv_obj_set_style_bg_img_src(ap_btn, LV_SYMBOL_PLUS, 0);
        lv_obj_add_event_cb(ap_btn, lv_spinbox_increment_event_cb_p, LV_EVENT_ALL,  NULL);

        ap_btn = lv_btn_create(apid_tab);
        lv_obj_set_size(ap_btn, h, h);
        lv_obj_align_to(ap_btn, ap_spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        lv_obj_set_style_bg_img_src(ap_btn, LV_SYMBOL_MINUS, 0);
        lv_obj_add_event_cb(ap_btn, lv_spinbox_decrement_event_cb_p, LV_EVENT_ALL, NULL);

        lv_obj_t * ap_label = lv_label_create(apid_tab);
        lv_obj_align_to(ap_label, ap_btn, LV_ALIGN_OUT_LEFT_MID, 10, 0);
        lv_label_set_text(ap_label, "P");
        lv_obj_add_style(ap_label, &label_theme, 0);

        ai_spinbox = lv_spinbox_create(apid_tab);
        lv_spinbox_set_range(ai_spinbox, -10000, 20000);
        lv_spinbox_step_prev(ai_spinbox);
        lv_obj_set_width(ai_spinbox, 100);
        lv_obj_center(ai_spinbox);
        lv_obj_align(ai_spinbox, LV_ALIGN_CENTER, -20, 0);
        lv_spinbox_set_digit_format(ai_spinbox, 5, 2);

        lv_obj_t * ai_btn = lv_btn_create(apid_tab);
        lv_obj_set_size(ai_btn, h, h);
        lv_obj_align_to(ai_btn, ai_spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
        lv_obj_set_style_bg_img_src(ai_btn, LV_SYMBOL_PLUS, 0);
        lv_obj_add_event_cb(ai_btn, lv_spinbox_increment_event_cb_i, LV_EVENT_ALL,  NULL);

        ai_btn = lv_btn_create(apid_tab);
        lv_obj_set_size(ai_btn, h, h);
        lv_obj_align_to(ai_btn, ai_spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        lv_obj_set_style_bg_img_src(ai_btn, LV_SYMBOL_MINUS, 0);
        lv_obj_add_event_cb(ai_btn, lv_spinbox_decrement_event_cb_i, LV_EVENT_ALL, NULL);

        lv_obj_t * ai_label = lv_label_create(apid_tab);
        lv_obj_align_to(ai_label, ai_btn, LV_ALIGN_OUT_LEFT_MID, 10, 0);
        lv_label_set_text(ai_label, "I");
        lv_obj_add_style(ai_label, &label_theme, 0);

        ad_spinbox = lv_spinbox_create(apid_tab);
        lv_spinbox_set_range(ad_spinbox, -10000, 20000);
        lv_spinbox_step_prev(ad_spinbox);
        lv_obj_set_width(ad_spinbox, 100);
        lv_spinbox_set_digit_format(ad_spinbox, 5, 2);
        lv_obj_align(ad_spinbox, LV_ALIGN_BOTTOM_MID, -20, 0);

        lv_obj_t * ad_btn = lv_btn_create(apid_tab);
        lv_obj_set_size(ad_btn, h, h);
        lv_obj_align_to(ad_btn, ad_spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
        lv_obj_set_style_bg_img_src(ad_btn, LV_SYMBOL_PLUS, 0);
        lv_obj_add_event_cb(ad_btn, lv_spinbox_increment_event_cb_d, LV_EVENT_ALL,  NULL);

        ad_btn = lv_btn_create(apid_tab);
        lv_obj_set_size(ad_btn, h, h);
        lv_obj_align_to(ad_btn, ad_spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        lv_obj_set_style_bg_img_src(ad_btn, LV_SYMBOL_MINUS, 0);
        lv_obj_add_event_cb(ad_btn, lv_spinbox_decrement_event_cb_d, LV_EVENT_ALL, NULL);

        lv_obj_t * ad_label = lv_label_create(apid_tab);
        lv_obj_align_to(ad_label, ad_btn, LV_ALIGN_OUT_LEFT_MID, 10, 0);
        lv_label_set_text(ad_label, "D");
        lv_obj_add_style(ad_label, &label_theme, 0);

        lv_spinbox_set_value(ap_spinbox, a_p);
        lv_spinbox_set_value(ai_spinbox, a_i);
        lv_spinbox_set_value(ad_spinbox, a_d);

        pros::Task labelTask(labelUpdate, "Label");
        pros::Task updatePIDTask(pidUpdateTask, "PID Update");
    }
}