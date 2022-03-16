// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               eeprom.c
* \li Compiler:           IAR EWAAVR 3.10c
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All devices with split EEPROM erase/write
*                         capabilities can be used.
*                         The example is written for ATmega48.
*
* \li AppNote:            AVR103 - Using the EEPROM Programming Modes.
*
* \li Description:        Example on how to use the split EEPROM erase/write
*                         capabilities in e.g. ATmega48. All EEPROM
*                         programming modes are tested, i.e. Erase+Write,
*                         Erase-only and Write-only.
*
*                         $Revision: 1.6 $
*                         $Date: Friday, February 11, 2005 07:16:44 UTC $
****************************************************************************/
#include "eeprom.h"
#include "grbl.h"

//#define EEPROM_SIZE  FLASH_PAGE_SIZE  /* Page size = 1KByte = 0x400 for STM32F103C8 */
// The last page of flash (e.g. For STM32F103C8: 127KB-128KB)
#define FLASH_START_ADDRESS_FOR_EEPROM    ((unsigned char*)(FLASH_BANK1_END + 1 - FLASH_PAGE_SIZE))
unsigned char Virtual_EEPROM[FLASH_PAGE_SIZE];

/* Define to reduce code size. */
//#define EEPROM_IGNORE_SELFPROG //!< Remove SPM flag polling.

/**
 * @brief Write the intermediate Virtual_EEPROM[] to actual flash memory
 */
void write_to_flash() {
  uint32_t nAddress = (uint32_t)(FLASH_START_ADDRESS_FOR_EEPROM);
  uint16_t nSize = FLASH_PAGE_SIZE;
  uint16_t *pBuffer = (uint16_t *)Virtual_EEPROM;

  FLASH_EraseInitTypeDef flash_erase_options;
  flash_erase_options.Banks = FLASH_BANK_1;
  flash_erase_options.NbPages = 1;
  flash_erase_options.PageAddress = (uint32_t)(FLASH_START_ADDRESS_FOR_EEPROM);
  flash_erase_options.TypeErase = FLASH_TYPEERASE_PAGES;
  uint32_t page_error;
  
  HAL_FLASH_Unlock();
  uint32_t ret;
  ret = HAL_FLASHEx_Erase(&flash_erase_options, &page_error);
  /* If erase operation was failed, a Flash error code is returned */
  if (ret != HAL_OK)
  {
    printString("Error: ");
    printInteger((int)ret);
    printString("\n");
    return;
  }

  // Start Program the flash
  while (nSize > 0)
  {
    if (*pBuffer != 0xffff) {
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, nAddress, *pBuffer++);
    } else {
      pBuffer++;
    }
    if (*pBuffer != 0xffff) {
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, nAddress + 2, *pBuffer++);
    } else {
      pBuffer++;
    }
    nSize -= 4;
    nAddress += 4;
  }

  HAL_FLASH_Lock();
}

/**
 * @brief Read data in (non-volatile) flash into intermediate Virtual_EEPROM[]
 */
static void read_from_flash_into_eeprom() {
  // Copy the data in non-volatile (flash) memory into virtual EEPROM in RAM
  int i;
  for (i = 0; i < FLASH_PAGE_SIZE; i++) {
    Virtual_EEPROM[i] = (unsigned char)(*(FLASH_START_ADDRESS_FOR_EEPROM + i));
  }
}

/**
 * @brief initialize the Virtual EEPROM buffer in RAM with non-volatile data in flash
 * 
 */
void eeprom_init() {
  read_from_flash_into_eeprom();
}

/*! \brief  Read byte from EEPROM.
 *
 *  This function reads one byte from a given EEPROM address.
 *
 *  \note  The CPU is halted for 4 clock cycles during EEPROM read.
 *
 *  \param  addr  EEPROM address to read from.
 *  \return  The byte read from the EEPROM address.
 */
unsigned char eeprom_get_char( unsigned int addr )
{
  return Virtual_EEPROM[addr];
}

/*! \brief  Write byte to EEPROM.
 *
 *  This function writes one byte to a given EEPROM address.
 *  The differences between the existing byte and the new value is used
 *  to select the most efficient EEPROM programming mode.
 *
 *  \note  The CPU is halted for 2 clock cycles during EEPROM programming.
 *
 *  \note  When this function returns, the new EEPROM value is not available
 *         until the EEPROM programming time has passed. The EEPE bit in EECR
 *         should be polled to check whether the programming is finished.
 *
 *  \note  The EEPROM_GetChar() function checks the EEPE bit automatically.
 *
 *  \param  addr  EEPROM address to write to.
 *  \param  new_value  New EEPROM value.
 */
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
  Virtual_EEPROM[addr] = new_value;
}

/**
 * @brief Set n bytes of data of Virtual_EEPROM the same new_value 
 * 
 * @param addr 
 * @param new_value 
 * @param n_byte 
 */
void eeprom_put_char_n_bytes( unsigned int addr, unsigned char new_value, size_t n_byte )
{
  memset(&Virtual_EEPROM[addr], new_value, n_byte);
}

// Extensions added as part of Grbl 

/**
 * @brief Copy the data in source buf into specific part of Virtual_EEPROM 
 *        and then write to non-volatile flash memory
 * 
 * @param destination start byte idx of Virtual_EEPROM
 * @param source 
 * @param size 
 */
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
    checksum = (checksum << 1) | (checksum >> 7); // left circular shift for 1 bit
    checksum += *source;
    eeprom_put_char(destination++, *(source++)); 
  }
  eeprom_put_char(destination, checksum);
  
  // Write the intermediate Virtual EEProm buffer to actual non-volatile flash memory
  write_to_flash();
}

/**
 * @brief Copy the data in specific part of Virtual_EEPROM into dest buf
 * 
 * @param destination 
 * @param source start byte idx of Virtual_EEPROM
 * @param size 
 * @return int false if stored checksum != calculated checksum
 *             true if stored checksum == calculated checksum
 */
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) { 
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) | (checksum >> 7); // left circular shift for 1 bit
    checksum += data;    
    *(destination++) = data; 
  }
  return(checksum == eeprom_get_char(source));
}

// end of file
