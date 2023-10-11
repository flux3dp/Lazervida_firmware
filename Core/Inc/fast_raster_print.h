#ifndef FAST_RASTER_PRINT_H
#define FAST_RASTER_PRINT_H

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
    kFastRasterPrintOff = 0,
    kFastRasterPrintOn
} FastRasterPrintMode;

typedef enum
{
    kRasterUltraHighRes, // 0.025 mm / pixel
    kRasterHighRes, // 0.05 mm / pixel
    kRasterMidRes,  // 0.1 mm / pixel
    kRasterLowRes   // 0.2 mm / pixel
} FastRasterPrintRes;



bool is_in_fast_raster_mode();
void fast_raster_mode_switch_on(float steps_per_mm, FastRasterPrintRes res);
void fast_raster_mode_switch_off();

// mm per pixel
float get_fast_raster_print_resolution();

// ================= Internal / Private ====================
// Filling
int fast_raster_mode_start_fill_new_line(uint16_t pixel_cnt);
int fast_raster_mode_fill_line_buffer(uint32_t new_32_pixels);
int fast_raster_mode_finish_line_filling();

// Printing
int fast_raster_mode_start_print_new_line();
//bool is_printing_fast_raster_line();
// ========================================================

void fast_raster_mode_inc_one_step(); // increment 1 to step count for the current printing line
bool is_on_fast_raster_mode_pixel_boundary();
uint8_t fast_raster_mode_pop_printing_bit();

// for st_go_idle() to determine whether to finish current line
//bool is_fast_raster_mode_no_pixel_remained(); 
//void fast_raster_mode_finish_current_line();

void fast_raster_print_DPC_handler(const char *line);
void fast_raster_print_DW_handler(const char *line);
void fast_raster_print_DFE_handler();
bool fast_raster_print_DPL_handler();

bool fast_raster_print_is_in_black_pixel();

/**
 *  For debug only
 */
//void show_fast_raster_mode_ctx();
//void dump_fast_raster_mode_filling_buf();


#endif