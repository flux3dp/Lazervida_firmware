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
  kLinePatternFilling,  // Accepting D2W cmd
  kLinePatternReady,    // D3FE accepted
  kLinePatternPrinting, // D4PL accepted
  kLinePatternFinish
} LinePatternBufStatus;

typedef struct LinePatternCtx
{
  LinePatternBufStatus status;
  uint16_t pixel_cnt;
  uint16_t filling_byte_head;
  uint16_t printing_pixel_idx; // bit (pixel) index
  uint8_t buf[LASER_PROFILE_BUFFER_SIZE];
} LinePatternCtx;

typedef struct FastRasterPrintCtx
{
  FastRasterPrintMode mode;
  FastRasterPrintRes resolution;
  float steps_per_mm;

  uint8_t filling_line_idx;
  uint8_t printing_line_idx;
  LinePatternCtx line_ctx_array[2]; 

  uint32_t current_step_idx; // the count of step that has been traveled in the current printing line, reset to 0 in each D4PL, increment in TIM interrupt
} FastRasterPrintCtx;


static FastRasterPrintCtx raster_ctx = {0};


static void reset_graient_mode_ctx()
{
  memset((void*)(&raster_ctx), 0, sizeof(FastRasterPrintCtx));
  raster_ctx.filling_line_idx = 0;
  raster_ctx.printing_line_idx = 1;
  raster_ctx.line_ctx_array[raster_ctx.filling_line_idx].status = kLinePatternEmpty;
  raster_ctx.line_ctx_array[raster_ctx.printing_line_idx].status = kLinePatternEmpty;
  raster_ctx.resolution = kRasterHighRes;
}


void fast_raster_mode_switch_on(float steps_per_mm, FastRasterPrintRes res)
{
  raster_ctx.mode = kFastRasterPrintOn;
  raster_ctx.steps_per_mm = steps_per_mm;
  raster_ctx.resolution = res;
}


void fast_raster_mode_switch_off()
{
  raster_ctx.mode = kFastRasterPrintOff;
  reset_graient_mode_ctx();
}


bool is_in_fast_raster_mode()
{
  if(raster_ctx.mode == kFastRasterPrintOn)
    return true;
  else
    return false;
}


/**
 * @retval mm per pixel 
 */
float get_fast_raster_print_resolution()
{
  switch(raster_ctx.resolution)
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
  LinePatternCtx *filling_line_ctx = &(raster_ctx.line_ctx_array[raster_ctx.filling_line_idx]);
  if( filling_line_ctx->status == kLinePatternEmpty )
  {
    if(pixel_cnt > LASER_PROFILE_BUFFER_SIZE * 8) // Exceed the available buffer space
    {
      // trim pixel count to conform the buffer size
      pixel_cnt = LASER_PROFILE_BUFFER_SIZE * 8;
    }

    filling_line_ctx->pixel_cnt = pixel_cnt;
    filling_line_ctx->filling_byte_head = 0;
    filling_line_ctx->status = kLinePatternFilling;
    return 0;
  }

  return -1;
}



/**
 * @brief Fill data into the buffer
 *        Discard data if exceeding pixel count
 * @param new_32_pixels 32 pixels of data acquired from D2W command
 * @retval 0 if success, others if fail
 */
int fast_raster_mode_fill_line_buffer(uint32_t new_32_pixels)    
{
  int i;
  uint8_t new_byte;
  LinePatternCtx *filling_line_ctx = &(raster_ctx.line_ctx_array[raster_ctx.filling_line_idx]);
  if(filling_line_ctx->status == kLinePatternFilling)
  {
    // Has already filled the expected number of pixel before entering this function
    // NOTE: Simply ignore data exceeding pixel count without return error
    if ( filling_line_ctx->pixel_cnt == 0 
        || ( filling_line_ctx->filling_byte_head > (filling_line_ctx->pixel_cnt-1)/8 ) 
        ) { 
      return 0; 
    }

    for(i = 0; i < 4; i++)
    {
      new_byte = 0xFF & (new_32_pixels >> ((3-i)*8));
      filling_line_ctx->buf[filling_line_ctx->filling_byte_head] = new_byte;
      filling_line_ctx->filling_byte_head += 1;
      if(filling_line_ctx->filling_byte_head > (filling_line_ctx->pixel_cnt-1)/8)
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
  LinePatternCtx *filling_line_ctx = &(raster_ctx.line_ctx_array[raster_ctx.filling_line_idx]);
  if(filling_line_ctx->status == kLinePatternFilling){
      filling_line_ctx->status = kLinePatternReady;
    return 0;
  } else {
    return -1;
  }
}


/**
 * @brief triggered by D4PL cmd
 * 
 * @retval: 0: successfully load a new line for printing
 *          -1: failed
 */
int fast_raster_mode_start_print_new_line()
{
  LinePatternCtx *printing_line_ctx = &(raster_ctx.line_ctx_array[raster_ctx.printing_line_idx]);
  LinePatternCtx *filling_line_ctx = &(raster_ctx.line_ctx_array[raster_ctx.filling_line_idx]);
  uint8_t temp_idx;
  //if( printing_line_ctx->status == kLinePatternFinish 
  //    || printing_line_ctx->status == kLinePatternEmpty )
  //{
    // Try to load a new line from filling line and reset the filling line
    if(filling_line_ctx->status == kLinePatternReady) 
    {
      
      // Swap filling line and printing line
      temp_idx = raster_ctx.printing_line_idx;
      raster_ctx.printing_line_idx = raster_ctx.filling_line_idx;
      raster_ctx.filling_line_idx = temp_idx;
      printing_line_ctx = &(raster_ctx.line_ctx_array[raster_ctx.printing_line_idx]);
      filling_line_ctx = &(raster_ctx.line_ctx_array[raster_ctx.filling_line_idx]);

      // Update status
      printing_line_ctx->printing_pixel_idx = 0;
      printing_line_ctx->status = kLinePatternPrinting;
      filling_line_ctx->pixel_cnt = 0;
      filling_line_ctx->filling_byte_head = 0;
      filling_line_ctx->status = kLinePatternEmpty;
      raster_ctx.current_step_idx = 0;

      //printString("[DEBUG: Start Printing Line]\n");

      return 0;
    } else if (filling_line_ctx->status == kLinePatternEmpty) {
      if (printing_line_ctx->status == kLinePatternPrinting) {
        // Ignore re-enter: consider as success
        return 0;
      }
    } else {
      //printString("[DEBUG: NREADY]\n");
    }
  //} else if (printing_line_ctx->status == kLinePatternPrinting ){
  //  //printf("DEBUG: Current Line Printing\n");
  //  //printString("[DEBUG: CLP]\n");
  //} else {
  //  //printString("[DEBUG: ELSE]\n");
  //}

  return -1;
}


bool is_printing_fast_raster_line()
{
  if(raster_ctx.line_ctx_array[raster_ctx.printing_line_idx].status == kLinePatternPrinting){
    return true;
  } else {
    return false;
  }
}

void fast_raster_mode_inc_one_step()
{
  if(raster_ctx.line_ctx_array[raster_ctx.printing_line_idx].status == kLinePatternPrinting)
    raster_ctx.current_step_idx += 1;
}


void fast_raster_mode_finish_current_line()
{
  raster_ctx.line_ctx_array[raster_ctx.printing_line_idx].status = kLinePatternFinish;
}


static float fast_raster_mode_get_steps_per_pixel()
{
  return (get_fast_raster_print_resolution() * raster_ctx.steps_per_mm);
}

static uint16_t fast_raster_mode_get_printing_pixel_idx()
{
  return raster_ctx.line_ctx_array[raster_ctx.printing_line_idx].printing_pixel_idx;
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
  if(raster_ctx.line_ctx_array[raster_ctx.printing_line_idx].status != kLinePatternPrinting)
  {
    return false;
  }

  // Check step index with pixel boundary
  if ( raster_ctx.current_step_idx == fast_raster_mode_get_nearest_boundary()) {
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
  LinePatternCtx *current_print_line = &(raster_ctx.line_ctx_array[raster_ctx.printing_line_idx]);
  if(current_print_line->status != kLinePatternPrinting)
  {
    return 0;
  }

  // Exceed pixel count, always return 0
  if(current_print_line->printing_pixel_idx >= current_print_line->pixel_cnt)
  {
    return 0;
  }

  result = current_print_line->buf[current_print_line->printing_pixel_idx / 8]; // get the current byte
  result >>= ( 7 - current_print_line->printing_pixel_idx % 8 ); // move the current pixel to bit-1
  result &= 0x1; // filter out other bits

  current_print_line->printing_pixel_idx += 1;
    
  return result;
}

/*
bool is_fast_raster_mode_no_pixel_remained() {
  if (fast_raster_mode_get_printing_pixel_idx() >= 
      raster_ctx.line_ctx_array[raster_ctx.printing_line_idx].pixel_cnt) {
    return true;
  } else {
    return false;
  }
}
*/
/*
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
*/




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

/**
 * @brief Fill n*32 pixels (n Words of data)
 * @param line D2W[hhhhhhhh...] (8*n hex char)
 */
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