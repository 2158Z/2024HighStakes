#include "liblvgl/extra/widgets/tabview/lv_tabview.h"
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

namespace screen {
    double p = 0;
    double i = 0;
    double d = 0;

    static lv_style_t btn_theme;
    static lv_style_t tab_theme;
    static lv_style_t label_theme;

    lv_obj_t * odom_label;
    lv_obj_t * temp_label;
    lv_obj_t * battery_label;

    static lv_obj_t * p_spinbox;
    static lv_obj_t * i_spinbox;
    static lv_obj_t * d_spinbox;
    const char * temperature = "Motor Temps: 87, 87, 87, 87, 87, 86,";
    const char * odom = "X: 0 Y: 0";
    int autonID = 0;

    lv_coord_t tab = 60;

    const char * auton_map[] = {"Auton1", "Auton2", NULL};

    static void auton_handler(lv_event_t * e){
        autonID = lv_btnmatrix_get_selected_btn(lv_event_get_target(e));
    }

    void labelUpdate(){
        lv_label_set_text(temp_label, temperature);
        lv_label_set_text(odom_label, odom);
        lv_label_set_text_fmt(battery_label, "Temp: %.2f, Current:%d", pros::battery::get_temperature(), pros::battery::get_current());
    }

    
    static void lv_spinbox_increment_event_cb_p(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
            lv_spinbox_increment(p_spinbox);
        }
    }

    static void lv_spinbox_decrement_event_cb_p(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
            lv_spinbox_decrement(p_spinbox);
        }
    }

    static void lv_spinbox_increment_event_cb_i(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
            lv_spinbox_increment(i_spinbox);
        }
    }

    static void lv_spinbox_decrement_event_cb_i(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
            lv_spinbox_decrement(i_spinbox);
        }
    }

        static void lv_spinbox_increment_event_cb_d(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
            lv_spinbox_increment(d_spinbox);
        }
    }

    static void lv_spinbox_decrement_event_cb_d(lv_event_t * e)
    {
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
            lv_spinbox_decrement(d_spinbox);
        }
    }

    void main(){
        lv_obj_t * tabView = lv_tabview_create(lv_scr_act(), LV_DIR_LEFT, tab);
        lv_obj_t * auton_tab = lv_tabview_add_tab(tabView, "Auton");
        lv_obj_t * info_tab = lv_tabview_add_tab(tabView, "Info");
        lv_obj_t * pid_tab = lv_tabview_add_tab(tabView, "PID");

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

        lv_obj_t * auton_btnm = lv_btnmatrix_create(auton_tab);
        lv_obj_align(auton_btnm, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_style(auton_btnm, &btn_theme, 0);
        lv_obj_add_event_cb(auton_btnm, auton_handler, LV_EVENT_ALL, NULL);
        lv_btnmatrix_set_map(auton_btnm, auton_map);

        lv_obj_t * image_obj = lv_img_create(info_tab);
        lv_obj_set_size(image_obj, 120, 120); // {250,207 z2} {120,120 z2p}
        lv_obj_align(image_obj, LV_ALIGN_CENTER, 0, 0);
        lv_img_set_src(image_obj, &z2);

        odom_label = lv_label_create(info_tab);
        lv_obj_add_style(odom_label, &label_theme, 0);
        lv_obj_align(odom_label, LV_ALIGN_TOP_LEFT, 0, 0);
        
        temp_label = lv_label_create(info_tab);
        lv_obj_add_style(temp_label, &label_theme, 0);
        lv_obj_align(temp_label, LV_ALIGN_TOP_LEFT, 0, -15);

        battery_label = lv_label_create(info_tab);
        lv_obj_align(battery_label, LV_ALIGN_TOP_LEFT, 0, 30);
        lv_obj_add_style(battery_label, &label_theme, 0);

        p_spinbox = lv_spinbox_create(pid_tab);
        lv_spinbox_set_range(p_spinbox, -1000, 25000);
        lv_spinbox_set_digit_format(p_spinbox, 5, 2);
        lv_spinbox_step_prev(p_spinbox);
        lv_obj_set_width(p_spinbox, 100);
        lv_obj_align(p_spinbox, LV_ALIGN_TOP_MID, 0, 0);

        lv_coord_t h = lv_obj_get_height(p_spinbox);

        lv_obj_t * p_btn = lv_btn_create(pid_tab);
        lv_obj_set_size(p_btn, h, h);
        lv_obj_align_to(p_btn, p_spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
        lv_obj_set_style_bg_img_src(p_btn, LV_SYMBOL_PLUS, 0);
        lv_obj_add_event_cb(p_btn, lv_spinbox_increment_event_cb_p, LV_EVENT_ALL,  NULL);

        p_btn = lv_btn_create(pid_tab);
        lv_obj_set_size(p_btn, h, h);
        lv_obj_align_to(p_btn, p_spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        lv_obj_set_style_bg_img_src(p_btn, LV_SYMBOL_MINUS, 0);
        lv_obj_add_event_cb(p_btn, lv_spinbox_decrement_event_cb_p, LV_EVENT_ALL, NULL);

        lv_obj_t * p_label = lv_label_create(pid_tab);
        lv_obj_align_to(p_label, p_btn, LV_ALIGN_OUT_LEFT_MID, 10, 0);
        lv_label_set_text(p_label, "P");
        lv_obj_add_style(p_label, &label_theme, 0);

        i_spinbox = lv_spinbox_create(pid_tab);
        lv_spinbox_set_range(i_spinbox, -1000, 25000);
        lv_spinbox_set_digit_format(i_spinbox, 5, 2);
        lv_spinbox_step_prev(i_spinbox);
        lv_obj_set_width(i_spinbox, 100);
        lv_obj_center(i_spinbox);

        lv_obj_t * i_btn = lv_btn_create(pid_tab);
        lv_obj_set_size(i_btn, h, h);
        lv_obj_align_to(i_btn, i_spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
        lv_obj_set_style_bg_img_src(i_btn, LV_SYMBOL_PLUS, 0);
        lv_obj_add_event_cb(i_btn, lv_spinbox_increment_event_cb_i, LV_EVENT_ALL,  NULL);

        i_btn = lv_btn_create(pid_tab);
        lv_obj_set_size(i_btn, h, h);
        lv_obj_align_to(i_btn, i_spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        lv_obj_set_style_bg_img_src(i_btn, LV_SYMBOL_MINUS, 0);
        lv_obj_add_event_cb(i_btn, lv_spinbox_decrement_event_cb_i, LV_EVENT_ALL, NULL);

        lv_obj_t * i_label = lv_label_create(pid_tab);
        lv_obj_align_to(i_label, i_btn, LV_ALIGN_OUT_LEFT_MID, 10, 0);
        lv_label_set_text(i_label, "I");
        lv_obj_add_style(i_label, &label_theme, 0);

        d_spinbox = lv_spinbox_create(pid_tab);
        lv_spinbox_set_range(d_spinbox, -1000, 25000);
        lv_spinbox_set_digit_format(d_spinbox, 5, 2);
        lv_spinbox_step_prev(d_spinbox);
        lv_obj_set_width(d_spinbox, 100);
        lv_obj_align(d_spinbox, LV_ALIGN_BOTTOM_MID, 0, 0);

        lv_obj_t * d_btn = lv_btn_create(pid_tab);
        lv_obj_set_size(d_btn, h, h);
        lv_obj_align_to(d_btn, d_spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
        lv_obj_set_style_bg_img_src(d_btn, LV_SYMBOL_PLUS, 0);
        lv_obj_add_event_cb(d_btn, lv_spinbox_increment_event_cb_d, LV_EVENT_ALL,  NULL);

        d_btn = lv_btn_create(pid_tab);
        lv_obj_set_size(d_btn, h, h);
        lv_obj_align_to(d_btn, d_spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        lv_obj_set_style_bg_img_src(d_btn, LV_SYMBOL_MINUS, 0);
        lv_obj_add_event_cb(d_btn, lv_spinbox_decrement_event_cb_d, LV_EVENT_ALL, NULL);

        lv_obj_t * d_label = lv_label_create(pid_tab);
        lv_obj_align_to(d_label, d_btn, LV_ALIGN_OUT_LEFT_MID, 10, 0);
        lv_label_set_text(d_label, "D");
        lv_obj_add_style(d_label, &label_theme, 0);

        lv_spinbox_set_value(p_spinbox, p);
        lv_spinbox_set_value(i_spinbox, i);
        lv_spinbox_set_value(d_spinbox, d);

        pros::Task labelTask(labelUpdate, "Label");
    }
}