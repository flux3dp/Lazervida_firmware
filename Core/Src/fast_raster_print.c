#include "fast_raster_print.h"

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define LAZER_VIDA_BUFFER_SIZE 2075 // 415mm/0.025mm / 8(bits/byte) = 2075 bytes
#define LASER_PROFILE_BUFFER_SIZE LAZER_VIDA_BUFFER_SIZE


typedef enum
{
  kLinePatternEmpty,
  kLinePatternFilling,
  kLinePatternReady,
  kLinePatternPrinting,
  kLinePatternFinish
}LinePatternBufStatus;

typedef struct LinePatterCtx
{
  LinePatternBufStatus status;
  uint16_t pixel_cnt;
  uint16_t filling_byte_head;
  uint16_t printing_pixel_idx; // bit (pixel) index
  uint8_t buf[LASER_PROFILE_BUFFER_SIZE];
} LinePatterCtx;

typedef struct FastRasterPrintCtx
{
  FastRasterPrintMode mode;
  FastRasterPrintRes resolution;
  float steps_per_mm;

  LinePatterCtx current_printing_line; 
  LinePatterCtx current_filling_line;  

  uint32_t current_line_step_cnt; // the count of step has been traveled in the current printing line, reset to 0 in each D4PL, increment in TIM interrupt
} FastRasterPrintCtx;


static volatile FastRasterPrintCtx fast_raster_print_ctx = {0};


static void reset_graient_mode_ctx()
{
  memset((void*)(&fast_raster_print_ctx), 0, sizeof(FastRasterPrintCtx));
  fast_raster_print_ctx.current_printing_line.status = kLinePatternEmpty;
  fast_raster_print_ctx.current_filling_line.status = kLinePatternEmpty;
}



void fast_raster_mode_switch_on(float _steps_per_mm)
{
  fast_raster_print_ctx.mode = kFastRasterPrintOn;
  fast_raster_print_ctx.steps_per_mm = _steps_per_mm;
}



void fast_raster_mode_switch_off()
{
  fast_raster_print_ctx.mode = kFastRasterPrintOff;
  reset_graient_mode_ctx();
}



bool is_in_fast_raster_mode()
{
  if(fast_raster_print_ctx.mode == kFastRasterPrintOn)
    return true;
  else
    return false;
}


int set_fast_raster_print_resolution(FastRasterPrintRes new_res)
{
  if(is_in_fast_raster_mode()) // resolution is always set with turning on fast raster print mode
  {
    return -1;
  }

  fast_raster_print_ctx.resolution = new_res;
  return 0;
}

/**
 * @retval mm per pixel 
 */
float get_fast_raster_print_resolution()
{
  switch(fast_raster_print_ctx.resolution)
  {
    case kRasterUltraHighRes:
      return 0.025f;
      break;
    case kRasterHighRes:
      return 0.05f;
      break;
    case kRasterMidRes:
      return 0.1f;
      break;
    case kRasterLowRes:
    default:
      return 0.2f;
      break;
  }
}


/**
 * @param pixel_cnt set the total pixel count for the new line
 * @retval          0 for succeed
 *                  -1 for fail
 */
int fast_raster_mode_start_fill_new_line(uint16_t pixel_cnt)
{
  if( fast_raster_print_ctx.current_filling_line.status == kLinePatternEmpty )
  {
    if(pixel_cnt > LASER_PROFILE_BUFFER_SIZE * 8) // Exceed the available buffer space
    {
      return -1;
    }

    fast_raster_print_ctx.current_filling_line.pixel_cnt = pixel_cnt;
    fast_raster_print_ctx.current_filling_line.filling_byte_head = 0;
    fast_raster_print_ctx.current_filling_line.status = kLinePatternFilling;
    return 0;
  }

  return -1;
}



/**
 * 
 *  @retval 0 if success, others if fail
 */
int fast_raster_mode_fill_line_buffer(uint32_t new_32_pixels)    
{
  int i;
  uint8_t new_byte;
  if(fast_raster_print_ctx.current_filling_line.status == kLinePatternFilling)
  {
    // Has already filled the expected number of pixel before entering this function
    if ( fast_raster_print_ctx.current_filling_line.pixel_cnt == 0 
        || fast_raster_print_ctx.current_filling_line.filling_byte_head > (fast_raster_print_ctx.current_filling_line.pixel_cnt-1)/8 ) { 
      return -1; 
    }
        
    for(i = 0; i < 4; i++)
    {
      new_byte = 0xFF & (new_32_pixels >> ((3-i)*8));
      fast_raster_print_ctx.current_filling_line.buf[fast_raster_print_ctx.current_filling_line.filling_byte_head] = new_byte;
      fast_raster_print_ctx.current_filling_line.filling_byte_head += 1;
      if(fast_raster_print_ctx.current_filling_line.filling_byte_head > (fast_raster_print_ctx.current_filling_line.pixel_cnt-1)/8)
      {
        break;
      }
    }

    /* 
    printf("Fill idx: %d\n", 
        fast_raster_print_ctx.current_filling_line.filling_byte_head);
    */
      return 0;
  }
    
  return -1;
}

int fast_raster_mode_finish_line_filling()
{
  if(fast_raster_print_ctx.current_filling_line.status == kLinePatternFilling){
      fast_raster_print_ctx.current_filling_line.status = kLinePatternReady;
    return 0;
  } else {
    return -1;
  }
}


/**
 *
 * @retval: 0: successfully load a new line for printing
 *          -1: failed
 */
int fast_raster_mode_start_print_new_line()
{
  if( fast_raster_print_ctx.current_printing_line.status == kLinePatternFinish 
      || fast_raster_print_ctx.current_printing_line.status == kLinePatternEmpty )
  {
    // Try to load a new line from filling line and reset the filling line
    if(fast_raster_print_ctx.current_filling_line.status == kLinePatternReady) 
    {
      memcpy((void*)(&(fast_raster_print_ctx.current_printing_line)), (void*)(&(fast_raster_print_ctx.current_filling_line)), sizeof(LinePatterCtx));
      fast_raster_print_ctx.current_printing_line.printing_pixel_idx = 0;
      fast_raster_print_ctx.current_printing_line.status = kLinePatternPrinting;
            
      memset((void*)(&(fast_raster_print_ctx.current_filling_line)), 0, sizeof(LinePatterCtx));
      fast_raster_print_ctx.current_filling_line.pixel_cnt = 0;
      fast_raster_print_ctx.current_filling_line.filling_byte_head = 0;
      fast_raster_print_ctx.current_filling_line.status = kLinePatternEmpty;

      fast_raster_print_ctx.current_line_step_cnt = 0;

      //printString("[DEBUG: Start Printing Line]\n");

      return 0;
    } else {
      //printString("[DEBUG: NREADY]\n");
    }
  } else if (fast_raster_print_ctx.current_printing_line.status == kLinePatternPrinting ){
    //printf("DEBUG: Current Line Printing\n");
    //printString("[DEBUG: CLP]\n");
  } else {
    //printString("[DEBUG: ELSE]\n");
  }

  return -1;
}


bool is_printing_fast_raster_line()
{
  if(fast_raster_print_ctx.current_printing_line.status == kLinePatternPrinting){
    return true;
  } else {
    return false;
  }
}

void fast_raster_mode_inc_one_step()
{
  if(fast_raster_print_ctx.current_printing_line.status == kLinePatternPrinting)
    fast_raster_print_ctx.current_line_step_cnt += 1;
}


void fast_raster_mode_finish_current_line()
{
  fast_raster_print_ctx.current_printing_line.status = kLinePatternFinish;
}


static float fast_raster_mode_get_steps_per_pixel()
{
  return (get_fast_raster_print_resolution() * fast_raster_print_ctx.steps_per_mm);
}

static uint16_t fast_raster_mode_get_printing_pixel_idx()
{
  return fast_raster_print_ctx.current_printing_line.printing_pixel_idx;
}

/**
 * @brief Position offset of the nearest boundary 
 *        of the current pixel and the next one
 * 
 * @retval in unit of steps
 */
static uint32_t fast_raster_mode_get_nearest_boundary()
{
  return roundf( 
            fast_raster_mode_get_steps_per_pixel() * 
            (float)(fast_raster_mode_get_printing_pixel_idx()) );
}

bool is_on_fast_raster_mode_pixel_boundary()
{
  if(fast_raster_print_ctx.current_printing_line.status != kLinePatternPrinting)
  {
    return false;
  }

  // Check step index with pixel boundary
  if ( fast_raster_print_ctx.current_line_step_cnt == fast_raster_mode_get_nearest_boundary()) {
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Pop one bit from printing buffer
 * 
 * @retval 1 for laser on, 
 *         0 for laser off
 */
uint8_t fast_raster_mode_pop_printing_bit()
{
  uint8_t result = 0;

  if(fast_raster_print_ctx.current_printing_line.status != kLinePatternPrinting)
  {
    return 0;
  }

  // Exceed pixel count, always return 0
  if(fast_raster_print_ctx.current_printing_line.printing_pixel_idx >= fast_raster_print_ctx.current_printing_line.pixel_cnt)
  {
    return 0;
  }

  result = fast_raster_print_ctx.current_printing_line.buf[fast_raster_print_ctx.current_printing_line.printing_pixel_idx/8]; // get the current byte
  result >>= ( 7 - fast_raster_print_ctx.current_printing_line.printing_pixel_idx % 8 ); // move the current pixel to bit-1
  result &= 0x1; // filter out other bits

  fast_raster_print_ctx.current_printing_line.printing_pixel_idx += 1;
    
  return result;
}

bool is_fast_raster_mode_no_pixel_remained() {
  if (fast_raster_mode_get_printing_pixel_idx() >= fast_raster_print_ctx.current_printing_line.pixel_cnt) {
    return true;
  } else {
    return false;
  }
}


void show_fast_raster_mode_ctx()
{
  if(is_in_fast_raster_mode()){
      printf("Fast raster print mode ON\n");
  }else{
      printf("Fast raster print mode OFF\n");
  }

  printf("Fast raster print ctx:\n");


  if(fast_raster_print_ctx.current_printing_line.status == kLinePatternPrinting){
    printf("Line printing\n");
    printf("Printing pixel idx: %d\n",
        fast_raster_print_ctx.current_printing_line.printing_pixel_idx);
    printf("current line total pixel cnt: %d\n",
        fast_raster_print_ctx.current_printing_line.pixel_cnt);
    printf("current line step cnt: %ld\n",
        fast_raster_print_ctx.current_line_step_cnt);
  } else if(fast_raster_print_ctx.current_printing_line.status == kLinePatternFinish){
    printf("Line finish\n");
  } else {
    printf("Line not printing\n");
  }
  
  if(fast_raster_print_ctx.current_filling_line.status == kLinePatternFilling){
    printf("Line filling\n");
    printf("filling byte head: %d\n",
        fast_raster_print_ctx.current_filling_line.filling_byte_head);
    printf("filling line total pixel cnt: %d\n",
        fast_raster_print_ctx.current_filling_line.pixel_cnt);
  } else if(fast_raster_print_ctx.current_filling_line.status == kLinePatternEmpty){
    printf("Line not filling\n");
  } else if(fast_raster_print_ctx.current_filling_line.status == kLinePatternReady){
    printf("Line ready for print\n");
    printf("filling line total pixel cnt: %d\n", 
        fast_raster_print_ctx.current_filling_line.pixel_cnt);
  }

} 

void dump_fast_raster_mode_filling_buf()
{
  uint16_t cursor = 0;
  char hex_buf[3] = {0};
  while(cursor < fast_raster_print_ctx.current_filling_line.filling_byte_head)
  {
    snprintf(hex_buf, 3, "%02X", fast_raster_print_ctx.current_filling_line.buf[cursor]);
    printf(hex_buf);
    cursor++;
  }
  printf("\n");
}





// D1PC[NNNN]: set Pixel Cnt and start a line filling
void fast_raster_print_DPC_handler(const char *line)
{
  //int i;
  long line_pixel_cnt;
  char *strchr_pointer = strchr(line, 'C');
  line_pixel_cnt = (strtol(strchr_pointer + 1, NULL, 10));

  if(is_in_fast_raster_mode())
  {
    //printf("%d\n", line_pixel_cnt); 
    if(fast_raster_mode_start_fill_new_line(line_pixel_cnt) == 0){
      //report_status_message(STATUS_OK);
    }else{
      // Fail: Don't respond
    }
  }
}

// D2W[hhhhhhhh]: Fill 32 pixels (a Word of data)
void fast_raster_print_DW_handler(const char *line)
{
  char *strchr_pointer = strchr(line, 'W');
  char *temp_pointer = strchr_pointer + 1;

  int hex_cnt = 0;
  long new_32_pixels = 0;
  //char debug_buf[9] = {0};
  while( (*temp_pointer != '*') && (*temp_pointer != 0) )
  {
    if(*temp_pointer <= 'F' && *temp_pointer >= 'A'){
      new_32_pixels <<= 4;
      new_32_pixels += (10 + *temp_pointer - 'A');
    }else if(*temp_pointer <= '9' && *temp_pointer >= '0'){
      new_32_pixels <<= 4;
      new_32_pixels += (*temp_pointer - '0');
    }else {
      break;
    }

    hex_cnt++;
    temp_pointer++;

    if(hex_cnt >= 8) {  // accept 8 hex (32 pixels) at once
      /*   
      printString("new_32_pixels: ");
      snprintf(debug_buf, 9, "%08X", new_32_pixels);
      printString(debug_buf);
      printString("\n");
      */
      if( fast_raster_mode_fill_line_buffer((uint32_t)new_32_pixels) == 0){
        //report_status_message(STATUS_OK);
      }
      hex_cnt = 0;
      new_32_pixels = 0;
    }
  }

}

// D3FE: Filling End
void fast_raster_print_DFE_handler()
{
  if( is_in_fast_raster_mode())
  {
    int result = fast_raster_mode_finish_line_filling();
    if(result == 0)
    {
      //report_status_message(STATUS_OK);
    }
  }
}

/* D4PL: start Printing the next Line
 * Will be blocked if:
 * * Motor not idle (previous motion not complete)
 * * Can't load the context of next line
 */
bool fast_raster_print_DPL_handler()
{
  if(is_in_fast_raster_mode())
  {
    // motor has been checked in caller
    if (fast_raster_mode_start_print_new_line() == 0) {
      return true;
    } else {
      return false;
    }
  }
  return true;
}