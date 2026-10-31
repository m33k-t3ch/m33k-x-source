#pragma once
#include <Arduino.h>
#include <lvgl.h>
extern const lv_image_dsc_t g_shard_s, g_shard_t;
extern const lv_image_dsc_t* const g_run0[4];
extern const lv_image_dsc_t* const g_run1[4];
extern const lv_image_dsc_t* const g_jump[4];
extern const lv_image_dsc_t* const g_pick[4];
extern const lv_image_dsc_t* const g_wifi[6];   // bigger t3ch action poses for the Wi-Fi page
extern const lv_image_dsc_t* const g_flyers[3];   // runner bubbles: malware mine + skull
extern const lv_image_dsc_t  g_malware;            // wardrive enemy sprite
extern const lv_image_dsc_t* const g_carhead[4];   // character heads for the wardrive car
extern const lv_image_dsc_t* const g_blast[4];     // per-character runner blast
extern const bool g_run_anim[4];
