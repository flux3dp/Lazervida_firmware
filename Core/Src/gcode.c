/*
  gcode.c - rs274/ngc parser.
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"
#include "peripherals.h"
#include "sensors.h"
#include "flux_machine.h"
#include "peripherals.h"

// NOTE: Max line number is defined by the g-code standard to be 99999. It seems to be an
// arbitrary value, and some GUIs may require more. So we increased it based on a max safe
// value when converting a float (7.2 digit precision)s to an integer.
#define MAX_LINE_NUMBER 10000000
#define MAX_TOOL_NUMBER 255 // Limited by max unsigned 8-bit value

#define AXIS_COMMAND_NONE 0
#define AXIS_COMMAND_NON_MODAL 1
#define AXIS_COMMAND_MOTION_MODE 2
#define AXIS_COMMAND_TOOL_LENGTH_OFFSET 3 // *Undefined but required

// Declare gc extern struct
parser_state_t gc_state;
parser_block_t gc_block;

extern float tilt_average;

#define FAIL(status) return(status);


void gc_init()
{
  memset(&gc_state, 0, sizeof(parser_state_t));

  // // Load default G54 coordinate system.
  // if (!(settings_read_coord_data(gc_state.modal.coord_select,gc_state.coord_system))) {
  //   report_status_message(STATUS_SETTING_READ_FAIL);
  // }
}


// Sets g-code parser position in mm. Input in steps. Called by the system abort and hard
// limit pull-off routines.
void gc_sync_position()
{
}


// Executes one line of 0-terminated G-Code. The line is assumed to contain only uppercase
// characters and signed floating point values (no whitespace). Comments and block delete
// characters have been removed. In this function, all units and positions are converted and
// exported to grbl's internal functions in terms of (mm, mm/min) and absolute machine
// coordinates, respectively.
/**
 * @brief 
 * 
 * @param line 
 * @return uint8_t STATUS code for response
 */
uint8_t gc_execute_line(const char *line)
{
  /* -------------------------------------------------------------------------------------
     STEP 1: Initialize parser block struct and copy current g-code state modes. The parser
     updates these modes and commands as the block line is parser and will only be used and
     executed after successful error-checking. The parser block struct also contains a block
     values struct, word tracking variables, and a non-modal commands tracker for the new
     block. This struct contains all of the necessary information to execute the block. */

  memset(&gc_block, 0, sizeof(parser_block_t)); // Initialize the parser block struct.
  memcpy(&gc_block.modal,&gc_state.modal,sizeof(gc_modal_t)); // Copy current modes

  uint8_t axis_command = AXIS_COMMAND_NONE;
  uint8_t axis_0, axis_1, axis_linear;
  uint8_t coord_select = 0; // Tracks G10 P coordinate selection for execution

  // Initialize bitflag tracking variables for axis indices compatible operations.
  uint8_t axis_words = 0; // XYZ tracking
  uint8_t ijk_words = 0; // IJK tracking

  // Initialize command and value words and parser flags variables.
  uint16_t command_words = 0; // Tracks G and M command words. Also used for modal group violations.
  uint16_t value_words = 0; // Tracks value words.
  uint8_t gc_parser_flags = GC_PARSER_NONE;

  // Determine if the line is a jogging motion or a normal g-code block.
  if (line[0] == '$') { // NOTE: `$J=` already parsed when passed to this function.
    // Set G1 and G94 enforced modes to ensure accurate error checks.
    gc_parser_flags |= GC_PARSER_JOG_MOTION;
    gc_block.modal.motion = MOTION_MODE_LINEAR;
    #ifdef USE_LINE_NUMBERS
      gc_block.values.n = JOG_LINE_NUMBER; // Initialize default line number reported during jog.
    #endif
  }

  /* -------------------------------------------------------------------------------------
     STEP 2: Import all g-code words in the block line. A g-code word is a letter followed by
     a number, which can either be a 'G'/'M' command or sets/assigns a command value. Also,
     perform initial error-checks for command word modal group violations, for any repeated
     words, and for negative values set for the value words F, N, P, T, and S. */

  uint8_t word_bit; // Bit-value for assigning tracking variables
  uint8_t char_counter;
  char letter;
  float value;
  uint8_t int_value = 0;
  uint16_t mantissa = 0;
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) { char_counter = 3; } // Start parsing after `$J=`
  else { char_counter = 0; }

  while (line[char_counter] != 0) { // Loop until no more g-code words in line.

    // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
    letter = line[char_counter];
    if((letter < 'A') || (letter > 'Z')) { FAIL(STATUS_EXPECTED_COMMAND_LETTER); } // [Expected word letter]
    char_counter++;
    if (!read_float(line, &char_counter, &value)) { FAIL(STATUS_BAD_NUMBER_FORMAT); } // [Expected word value]

    // Convert values to smaller uint8 significand and mantissa values for parsing this word.
    // NOTE: Mantissa is multiplied by 100 to catch non-integer command values. This is more
    // accurate than the NIST gcode requirement of x10 when used for commands, but not quite
    // accurate enough for value words that require integers to within 0.0001. This should be
    // a good enough comprimise and catch most all non-integer errors. To make it compliant,
    // we would simply need to change the mantissa to int16, but this add compiled flash space.
    // Maybe update this later.
    int_value = trunc(value);
    mantissa =  round(100*(value - int_value)); // Compute mantissa for Gxx.x commands.
    // NOTE: Rounding must be used to catch small floating point errors.

    // Check if the g-code word is supported or errors due to modal group violations or has
    // been repeated in the g-code block. If ok, update the command or record its value.
    switch(letter) {

      /* 'G' and 'M' Command Words: Parse commands and check for modal group violations.
         NOTE: Modal group numbers are defined in Table 4 of NIST RS274-NGC v3, pg.20 */

      case 'G':
        // Determine 'G' command and its modal group
        switch(int_value) {
          case 10: case 28: case 30: case 92:
            // Check for G10/28/30/92 being called with G0/1/2/3/38 on same block.
            // * G43.1 is also an axis command but is not explicitly defined this way.
            if (mantissa == 0) { // Ignore G28.1, G30.1, and G92.1
              if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict]
              axis_command = AXIS_COMMAND_NON_MODAL;
            }
            // No break. Continues to next line.
          case 4: case 53:
            word_bit = MODAL_GROUP_G0;
            gc_block.non_modal_command = int_value;
            if ((int_value == 28) || (int_value == 30) || (int_value == 92)) {
              if (!((mantissa == 0) || (mantissa == 10))) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); }
              gc_block.non_modal_command += mantissa;
              mantissa = 0; // Set to zero to indicate valid non-integer G command.
            }                
            break;
          case 0: case 1: case 2: case 3: case 38:
            // Check for G0/1/2/3/38 being called with G10/28/30/92 on same block.
            // * G43.1 is also an axis command but is not explicitly defined this way.
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict]
            axis_command = AXIS_COMMAND_MOTION_MODE;
            // No break. Continues to next line.
          case 80:
            word_bit = MODAL_GROUP_G1;
            gc_block.modal.motion = int_value;
            if (int_value == 38){
              if (!((mantissa == 20) || (mantissa == 30) || (mantissa == 40) || (mantissa == 50))) {
                FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G38.x command]
              }
              gc_block.modal.motion += (mantissa/10)+100;
              mantissa = 0; // Set to zero to indicate valid non-integer G command.
            }  
            break;
          case 17: case 18: case 19:
            word_bit = MODAL_GROUP_G2;
            gc_block.modal.plane_select = int_value - 17;
            break;
          case 90: case 91:
            if (mantissa == 0) {
              word_bit = MODAL_GROUP_G3;
              gc_block.modal.distance = int_value - 90;
            } else {
              word_bit = MODAL_GROUP_G4;
              if ((mantissa != 10) || (int_value == 90)) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G90.1 not supported]
              mantissa = 0; // Set to zero to indicate valid non-integer G command.
              // Otherwise, arc IJK incremental mode is default. G91.1 does nothing.
            }
            break;
          case 20: case 21:
            word_bit = MODAL_GROUP_G6;
            gc_block.modal.units = 21 - int_value;
            break;
          case 40:
            word_bit = MODAL_GROUP_G7;
            // NOTE: Not required since cutter radius compensation is always disabled. Only here
            // to support G40 commands that often appear in g-code program headers to setup defaults.
            // gc_block.modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
            break;
          case 61:
            word_bit = MODAL_GROUP_G13;
            if (mantissa != 0) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G61.1 not supported]
            // gc_block.modal.control = CONTROL_MODE_EXACT_PATH; // G61
            break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G command]
        }
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [Unsupported or invalid Gxx.x command]
        // Check for more than one command per modal group violations in the current block
        // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;

      case 'M':

        // Determine 'M' command and its modal group
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [No Mxx.x commands]
        switch(int_value) {
          case 0: case 1: case 2: case 30:
            word_bit = MODAL_GROUP_M4;
            switch(int_value) {
              case 0: gc_block.modal.program_flow = PROGRAM_FLOW_PAUSED; break; // Program pause
              case 1: break; // Optional stop not supported. Ignore.
              default: gc_block.modal.program_flow = int_value; // Program end and reset
            }
            break;
          #ifdef ENABLE_M7
            case 7: case 8: case 9:
          #else
            case 8: case 9:
          #endif
            word_bit = MODAL_GROUP_M8;
            switch(int_value) {
            }
            break;
          #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
            case 56:
              word_bit = MODAL_GROUP_M9;
              gc_block.modal.override = OVERRIDE_PARKING_MOTION;
              break;
          #endif
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported M command]
        }

        // Check for more than one command per modal group violations in the current block
        // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;
      // ===================== FLUX's dedicated commands =======================
      case 'B':
        switch(int_value) {
          case 11:
            set_power_24v();
            return (STATUS_OK);
          case 12:
            reset_power_24v();
            return (STATUS_OK);
          case 13:
            I2C_scanning();
            return (STATUS_OK);
          case 14:
            start_firmware_update();
            return (STATUS_INVALID_STATEMENT);
          case 101:
            print_uint8_base2_ndigit(MSA311_get_partid(), 8);
            printString("\n");
            return (STATUS_OK);
          case 102:
            print_uint32_base2_ndigit(Adafruit_MSA311_getDataRate(), 4);
            printString("\n");
            return (STATUS_OK);
          case 103:
            print_uint32_base2_ndigit(Adafruit_MSA311_getPowerMode(), 2);
            printString("\n");
            return (STATUS_OK);
          case 104:
            print_uint32_base2_ndigit(Adafruit_MSA311_getBandwidth(), 2);
            printString("\n");
            return (STATUS_OK);
          case 105:
            print_uint32_base2_ndigit(Adafruit_MSA311_getRange(), 2);
            printString("\n");
            return (STATUS_OK);
          case 106:
            print_uint8_base2_ndigit(Adafruit_MSA311_getMotionInterruptStatus(), 8);
            printString("\n");
            return (STATUS_OK);
          case 107:
            MSA311_setActiveInterruptThresh(10);
            return (STATUS_OK);
          case 108:
            MSA311_setActiveInterruptThresh(100);
            return (STATUS_OK);
          case 109:
            MSA311_setActiveInterruptThresh(150);
            return (STATUS_OK);
          case 110:
            MSA311_setActiveInterruptThresh(200);
            return (STATUS_OK);
          case 111:
            MSA311_setActiveInterruptThresh(90);
            return (STATUS_OK);
          case 112:
            print_uint8_base10(MSA311_getActiveInterruptThresh());
            return (STATUS_OK);
          case 113:
            MSA311_setActiveInterruptDur(MSA311_ACTIVEDUR_2_MS);
            return (STATUS_OK);
          case 114:
            MSA311_setActiveInterruptDur(MSA311_ACTIVEDUR_3_MS);
            return (STATUS_OK);
          case 115:
            MSA311_setActiveInterruptDur(MSA311_ACTIVEDUR_4_MS);
            return (STATUS_OK);
          case 116:
            print_uint8_base10(MSA311_getActiveInterruptDur() + 1);
            return (STATUS_OK);
          case 117:
            eeprom_dump();
            return (STATUS_OK);
          */
          default:
            FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
        }
        break;
      
      case 'D':
        break;
      // =================== End of FLUX's dedicated commands =====================

      // NOTE: All remaining letters assign values.
      default:

        /* Non-Command Words: This initial parsing phase only checks for repeats of the remaining
           legal g-code words and stores their value. Error-checking is performed later since some
           words (I,J,K,L,P,R) have multiple connotations and/or depend on the issued commands. */
        switch(letter){
          case 'L': word_bit = WORD_L; gc_block.values.l = int_value; break;
          case 'N': word_bit = WORD_N; gc_block.values.n = trunc(value); break;
          case 'P': word_bit = WORD_P; gc_block.values.p = value; break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
        }

        // NOTE: Variable 'word_bit' is always assigned, if the non-command letter is valid.
        if (bit_istrue(value_words,bit(word_bit))) { FAIL(STATUS_GCODE_WORD_REPEATED); } // [Word repeated]
        // Check for invalid negative values for words F, N, P, T, and S.
        // NOTE: Negative value check is done here simply for code-efficiency.
        if ( bit(word_bit) & (bit(WORD_F)|bit(WORD_N)|bit(WORD_P)|bit(WORD_T)|bit(WORD_S)) ) {
          if (value < 0.0) { FAIL(STATUS_NEGATIVE_VALUE); } // [Word value cannot be negative]
        }
        value_words |= bit(word_bit); // Flag to indicate parameter assigned.

    }
  }
  // Parsing complete!

  // [0. Non-specific/common error-checks and miscellaneous setup]:

  // Determine implicit axis command conditions. Axis words have been passed, but no explicit axis
  // command has been sent. If so, set axis command to current motion mode.
  if (axis_words) {
    if (!axis_command) { axis_command = AXIS_COMMAND_MOTION_MODE; } // Assign implicit motion-mode
  }

  // Check for valid line number N value.
  if (bit_istrue(value_words,bit(WORD_N))) {
    // Line number value cannot be less than zero (done) or greater than max line number.
    if (gc_block.values.n > MAX_LINE_NUMBER) { FAIL(STATUS_GCODE_INVALID_LINE_NUMBER); } // [Exceeds max line number]
  }
  // [10. Dwell ]: P value missing. P is negative (done.) NOTE: See below.
  if (gc_block.non_modal_command == NON_MODAL_DWELL) {
    if (bit_isfalse(value_words,bit(WORD_P))) { FAIL(STATUS_GCODE_VALUE_WORD_MISSING); } // [P word missing]
    bit_false(value_words,bit(WORD_P));
  }
  
  switch (gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:
      break;
    default:

      // At this point, the rest of the explicit axis commands treat the axis values as the traditional
      // target position with the coordinate system offsets, G92 offsets, absolute override, and distance
      // modes applied. This includes the motion mode commands. We can now pre-compute the target position.
      // NOTE: Tool offsets may be appended to these conversions when/if this feature is added.

      // Check remaining non-modal commands for errors.
      switch (gc_block.non_modal_command) {
        case NON_MODAL_RESET_COORDINATE_OFFSET:
          // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
          break;
        case NON_MODAL_ABSOLUTE_OVERRIDE:
          // [G53 Errors]: G0 and G1 are not active. Cutter compensation is enabled.
          // NOTE: All explicit axis word commands are in this modal group. So no implicit check necessary.
          if (!(gc_block.modal.motion == MOTION_MODE_SEEK || gc_block.modal.motion == MOTION_MODE_LINEAR)) {
            FAIL(STATUS_GCODE_G53_INVALID_MOTION_MODE); // [G53 G0/1 not active]
          }
          break;
      }
  }

  // [20. Motion modes ]:
  if (gc_block.modal.motion == MOTION_MODE_NONE) {
    // [G80 Errors]: Axis word are programmed while G80 is active.
    // NOTE: Even non-modal commands or TLO that use axis words will throw this strict error.
    if (axis_words) { FAIL(STATUS_GCODE_AXIS_WORDS_EXIST); } // [No axis words allowed]

  // Check remaining motion modes, if axis word are implicit (exist and not used by G10/28/30/92), or
  // was explicitly commanded in the g-code block.
  } else if ( axis_command == AXIS_COMMAND_MOTION_MODE ) {

    if (gc_block.modal.motion == MOTION_MODE_SEEK) {
      // [G0 Errors]: Axis letter not configured or without real value (done.)
      // Axis words are optional. If missing, set axis command flag to ignore execution.
      if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

    // All remaining motion modes (all but G0 and G80), require a valid feed rate value. In units per mm mode,
    // the value must be positive. In inverse time mode, a positive value must be passed with each block.
    } else {
      switch (gc_block.modal.motion) {
        case MOTION_MODE_LINEAR:
          // [G1 Errors]: Feed rate undefined. Axis letter not configured or without real value.
          // Axis words are optional. If missing, set axis command flag to ignore execution.
          if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }
          break;
        case MOTION_MODE_PROBE_TOWARD_NO_ERROR: case MOTION_MODE_PROBE_AWAY_NO_ERROR:
          gc_parser_flags |= GC_PARSER_PROBE_IS_NO_ERROR; // No break intentional.
        case MOTION_MODE_PROBE_TOWARD: case MOTION_MODE_PROBE_AWAY:
          if ((gc_block.modal.motion == MOTION_MODE_PROBE_AWAY) || 
              (gc_block.modal.motion == MOTION_MODE_PROBE_AWAY_NO_ERROR)) { gc_parser_flags |= GC_PARSER_PROBE_IS_AWAY; }
          // [G38 Errors]: Target is same current. No axis words. Cutter compensation is enabled. Feed rate
          //   is undefined. Probe is triggered. NOTE: Probe check moved to probe cycle. Instead of returning
          //   an error, it issues an alarm to prevent further motion to the probe. It's also done there to
          //   allow the planner buffer to empty and move off the probe trigger before another probing cycle.
          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
          break;
      }
    }
  }

  // [21. Program flow ]: No error checks required.

  // [0. Non-specific error-checks]: Complete unused value words check, i.e. IJK used when in arc
  // radius mode, or axis words that aren't used in the block.
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
    // Jogging only uses the F feed rate and XYZ value words. N is valid, but S and T are invalid.
    bit_false(value_words,(bit(WORD_N)|bit(WORD_F)));
  } else {
    bit_false(value_words,(bit(WORD_N)|bit(WORD_F)|bit(WORD_S)|bit(WORD_T))); // Remove single-meaning value words.
  }
  if (axis_command) { bit_false(value_words,(bit(WORD_X)|bit(WORD_Y)|bit(WORD_Z))); } // Remove axis words.
  if (value_words) { FAIL(STATUS_GCODE_UNUSED_WORDS); } // [Unused words]

  /* -------------------------------------------------------------------------------------
     STEP 4: EXECUTE!!
     Assumes that all error-checking has been completed and no failure modes exist. We just
     need to update the state and execute the block according to the order-of-execution.
  */

  // [0. Non-specific/common error-checks and miscellaneous setup]:
  // NOTE: If no line number is present, the value is zero.
  gc_state.line_number = gc_block.values.n;
  #ifdef USE_LINE_NUMBERS
    pl_data->line_number = gc_state.line_number; // Record data for planner use.
  #endif

  // [1. Comments feedback ]:  NOT SUPPORTED

  // [2. Set feed rate mode ]:
  // [21. Program flow ]:
  // M0,M1,M2,M30: Perform non-running program flow actions. During a program pause, the buffer may
  // refill and can only be resumed by the cycle start run-time command.
  gc_state.modal.program_flow = gc_block.modal.program_flow;
  if (gc_state.modal.program_flow) {
    protocol_buffer_synchronize(); // Sync and finish all remaining buffered motions before moving on.
    if (gc_state.modal.program_flow == PROGRAM_FLOW_PAUSED) {
      if (sys.state != STATE_CHECK_MODE) {
        system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
        protocol_execute_realtime(); // Execute suspend.
      }
    } else { // == PROGRAM_FLOW_COMPLETED
      // Upon program complete, only a subset of g-codes reset to certain defaults, according to
      // LinuxCNC's program end descriptions and testing. Only modal groups [G-code 1,2,3,5,7,12]
      // and [M-code 7,8,9] reset to [G1,G17,G90,G94,G40,G54,M5,M9,M48]. The remaining modal groups
      // [G-code 4,6,8,10,13,14,15] and [M-code 4,5,6] and the modal words [F,S,T,H] do not reset.
      gc_state.modal.motion = MOTION_MODE_LINEAR;
      gc_state.modal.plane_select = PLANE_SELECT_XY;
      gc_state.modal.distance = DISTANCE_MODE_ABSOLUTE;
      #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
        #ifdef DEACTIVATE_PARKING_UPON_INIT
          gc_state.modal.override = OVERRIDE_DISABLED;
        #else
          gc_state.modal.override = OVERRIDE_PARKING_MOTION;
        #endif
      #endif

      // Execute coordinate change and spindle/coolant stop.
      if (sys.state != STATE_CHECK_MODE) {
        system_flag_wco_change(); // Set to refresh immediately just in case something altered.
        reset_power_24v();
        // spindle_set_state(SPINDLE_DISABLE,0.0);
        // coolant_set_state(COOLANT_DISABLE);
      }
      report_feedback_message(MESSAGE_PROGRAM_END);
    }
    gc_state.modal.program_flow = PROGRAM_FLOW_RUNNING; // Reset program flow.
  }

  // TODO: % to denote start of program.

  return(STATUS_OK);
}


/*
  Not supported:

  - Canned cycles
  - Tool radius compensation
  - A,B,C-axes
  - Evaluation of expressions
  - Variables
  - Override control (TBD)
  - Tool changes
  - Switches

   (*) Indicates optional parameter, enabled through config.h and re-compile
   group 0 = {G92.2, G92.3} (Non modal: Cancel and re-enable G92 offsets)
   group 1 = {G81 - G89} (Motion modes: Canned cycles)
   group 4 = {M1} (Optional stop, ignored)
   group 6 = {M6} (Tool change)
   group 7 = {G41, G42} cutter radius compensation (G40 is supported)
   group 8 = {G43} tool length offset (G43.1/G49 are supported)
   group 8 = {M7*} enable mist coolant (* Compile-option)
   group 9 = {M48, M49, M56*} enable/disable override switches (* Compile-option)
   group 10 = {G98, G99} return mode canned cycles
   group 13 = {G61.1, G64} path control mode (G61 is supported)
*/
