#include "liblvgl/extra/widgets/tabview/lv_tabview.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_style.h"
#include "liblvgl/widgets/lv_btnmatrix.h"
#include "main.h"
#include "liblvgl/lvgl.h"
#include "screen.h"

namespace screen {
    static lv_style_t btn_theme;
    static lv_style_t tab_theme;
    static lv_style_t label_theme;

    lv_obj_t * odom_label;
    lv_obj_t * temp_label;
    const char * temperature = "0";
    int autonID = 0;

    lv_coord_t tab = 60;

    const char * auton_map[] = {"Auton1", "Auton2", NULL};

    static void auton_handler(lv_event_t * e){
        autonID = lv_btnmatrix_get_selected_btn(lv_event_get_target(e));
    }

    void labelUpdate(){
        lv_label_set_text(temp_label, temperature);
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

        lv_obj_t * auton_btnm = lv_btnmatrix_create(auton_tab);
        lv_obj_align(auton_btnm, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_style(auton_btnm, &btn_theme, 0);
        lv_obj_add_event_cb(auton_btnm, auton_handler, LV_EVENT_ALL, NULL);

        odom_label = lv_label_create(info_tab);
        lv_obj_add_style(odom_label, &label_theme, 0);
        lv_obj_align(odom_label, LV_ALIGN_TOP_LEFT, 0, 0);
        
        temp_label = lv_label_create(info_tab);
        lv_obj_add_style(temp_label, &label_theme, 0);
        lv_obj_align(temp_label, LV_ALIGN_TOP_LEFT, 0, -15);

        pros::Task labelTask(labelUpdate, "Label");
    }
}