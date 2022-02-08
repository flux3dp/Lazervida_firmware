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

// TODO: Check whether this is correct for STM32F103T8x
#define PAGE_SIZE  (uint16_t)0x400  /* Page size = 1KByte */
#define EEPROM_START_ADDRESS    ((unsigned char*)0x0801fc00) // The flash location 127KB-128KB
unsigned char Virtual_EEPROM[PAGE_SIZE];

/* Define to reduce code size. */
#define EEPROM_IGNORE_SELFPROG //!< Remove SPM flag polling.

/**
 * @brief Write the intermediate EEBuffer[] to actual flash memory
 */
void write_to_flash() {
  uint32_t nAddress = (uint32_t)(EEPROM_START_ADDRESS);
  uint16_t nSize = PAGE_SIZE;
  uint16_t *pBuffer = (uint16_t *)Virtual_EEPROM;

  FLASH_EraseInitTypeDef flash_erase_options;
  flash_erase_options.Banks = FLASH_BANK_1;
  flash_erase_options.NbPages = 1;
  flash_erase_options.PageAddress = (uint32_t)(EEPROM_START_ADDRESS);
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

void eeprom_init() {
  // Copy the data in non-volatile virtual memory into virtual EEPROM in RAM
  int i;
  for (i = 0; i < PAGE_SIZE; i++) {
    Virtual_EEPROM[i] = (unsigned char)(*(EEPROM_START_ADDRESS + i));
  }
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

// Extensions added as part of Grbl 


void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
    checksum = (checksum << 1) | (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++)); 
  }
  eeprom_put_char(destination, checksum);
  
  // Write the intermediate Virtual EEProm buffer to actual non-volatile flash memory
  write_to_flash();
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) { 
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) | (checksum >> 7);
    checksum += data;    
    *(destination++) = data; 
  }
  return(checksum == eeprom_get_char(source));
}

// end of file
