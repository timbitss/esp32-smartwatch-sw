//
// Created by Timothy on 2022-01-02.
//

#include "LVGL_App.h"
#include "lvgl.h"

void LVGL_App()
{
    lv_obj_t* label = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_text(label, "Hello World!");
}