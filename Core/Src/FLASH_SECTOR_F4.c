/**
 ***************************************************************************************************************
 ***************************************************************************************************************
 ***************************************************************************************************************
 File:	      FLASH_SECTOR_F4.c
 Modifier:   ControllersTech.com
 Updated:    27th MAY 2021
 ***************************************************************************************************************
 Copyright (C) 2017 ControllersTech.com
 This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
 of the GNU General Public License version 3 as published by the Free Software Foundation.
 This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
 or indirectly by this software, read more about this on the GNU General Public License.
 ***************************************************************************************************************
 */

#include "FLASH_SECTOR_F4.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"

/* DEFINE the SECTORS according to your reference manual
 * STM32F446RE have:-
 *  Sector 0 to Sector 3 each 16KB
 *  Sector 4 as 64KB
 *  Sector 5 to Sector 7 each 128KB
 */

static uint32_t GetSector(uint32_t Address) {
	uint32_t sector = 0;

	if ((Address < 0x08003FFF) && (Address >= 0x08000000)) {
		sector = FLASH_SECTOR_0;
	} else if ((Address < 0x08007FFF) && (Address >= 0x08004000)) {
		sector = FLASH_SECTOR_1;
	} else if ((Address < 0x0800BFFF) && (Address >= 0x08008000)) {
		sector = FLASH_SECTOR_2;
	} else if ((Address < 0x0800FFFF) && (Address >= 0x0800C000)) {
		sector = FLASH_SECTOR_3;
	} else if ((Address < 0x0801FFFF) && (Address >= 0x08010000)) {
		sector = FLASH_SECTOR_4;
	} else if ((Address < 0x0803FFFF) && (Address >= 0x08020000)) {
		sector = FLASH_SECTOR_5;
	} else if ((Address < 0x0805FFFF) && (Address >= 0x08040000)) {
		sector = FLASH_SECTOR_6;
	} else if ((Address < 0x0807FFFF) && (Address >= 0x08060000)) {
		sector = FLASH_SECTOR_7;
	}
	/*  else if((Address < 0x0809FFFF) && (Address >= 0x08080000))
	 {
	 sector = FLASH_SECTOR_8;
	 }
	 else if((Address < 0x080BFFFF) && (Address >= 0x080A0000))
	 {
	 sector = FLASH_SECTOR_9;
	 }
	 else if((Address < 0x080DFFFF) && (Address >= 0x080C0000))
	 {
	 sector = FLASH_SECTOR_10;
	 }
	 else if((Address < 0x080FFFFF) && (Address >= 0x080E0000))
	 {
	 sector = FLASH_SECTOR_11;
	 }
	 else if((Address < 0x08103FFF) && (Address >= 0x08100000))
	 {
	 sector = FLASH_SECTOR_12;
	 }
	 else if((Address < 0x08107FFF) && (Address >= 0x08104000))
	 {
	 sector = FLASH_SECTOR_13;
	 }
	 else if((Address < 0x0810BFFF) && (Address >= 0x08108000))
	 {
	 sector = FLASH_SECTOR_14;
	 }
	 else if((Address < 0x0810FFFF) && (Address >= 0x0810C000))
	 {
	 sector = FLASH_SECTOR_15;
	 }
	 else if((Address < 0x0811FFFF) && (Address >= 0x08110000))
	 {
	 sector = FLASH_SECTOR_16;
	 }
	 else if((Address < 0x0813FFFF) && (Address >= 0x08120000))
	 {
	 sector = FLASH_SECTOR_17;
	 }
	 else if((Address < 0x0815FFFF) && (Address >= 0x08140000))
	 {
	 sector = FLASH_SECTOR_18;
	 }
	 else if((Address < 0x0817FFFF) && (Address >= 0x08160000))
	 {
	 sector = FLASH_SECTOR_19;
	 }
	 else if((Address < 0x0819FFFF) && (Address >= 0x08180000))
	 {
	 sector = FLASH_SECTOR_20;
	 }
	 else if((Address < 0x081BFFFF) && (Address >= 0x081A0000))
	 {
	 sector = FLASH_SECTOR_21;
	 }
	 else if((Address < 0x081DFFFF) && (Address >= 0x081C0000))
	 {
	 sector = FLASH_SECTOR_22;
	 }
	 else if (Address < 0x081FFFFF) && (Address >= 0x081E0000)
	 {
	 sector = FLASH_SECTOR_23;
	 }*/
	return sector;
}

uint8_t bytes_temp[4];
#include "stm32f4xx_hal.h"

uint32_t GetSector(uint32_t Address);
/*uint32_t FLash_Erase_Sector {
	FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SECTORError;
    uint32_t StartSector = GetSector(StartSectorAddress);
    uint32_t EndSectorAddress = StartSectorAddress;
    uint32_t EndSector = GetSector(EndSectorAddress);
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = StartSector;
    EraseInitStruct.NbSectors = (EndSector - StartSector) + 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK) {
        HAL_FLASH_Lock();
        return HAL_FLASH_GetError();
    }
}*/
uint32_t Flash_Write_Data_Int(uint32_t StartSectorAddress, uint32_t Data) {
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, Data);
    HAL_FLASH_Lock();
    return 0; // Success
}

uint32_t Flash_Erase(uint32_t StartSectorAddress) {
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	HAL_FLASH_Unlock();
	uint32_t StartSector = GetSector(StartSectorAddress);
	uint32_t EndSectorAddress = StartSectorAddress ;
	uint32_t EndSector = GetSector(EndSectorAddress);
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = StartSector;
	EraseInitStruct.NbSectors = (EndSector - StartSector) + 1;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK) {
		return HAL_FLASH_GetError();
	}
	HAL_FLASH_Lock();
	return 0;
}
void float2Bytes(uint8_t *ftoa_bytes_temp, float float_variable) {
	union {
		float a;
		uint8_t bytes[4];
	} thing;

	thing.a = float_variable;

	for (uint8_t i = 0; i < 4; i++) {
		ftoa_bytes_temp[i] = thing.bytes[i];
	}

}

float Bytes2float(uint8_t *ftoa_bytes_temp) {
	union {
		float a;
		uint8_t bytes[4];
	} thing;

	for (uint8_t i = 0; i < 4; i++) {
		thing.bytes[i] = ftoa_bytes_temp[i];
	}

	float float_variable = thing.a;
	return float_variable;
}

uint32_t Flash_Write_Data(uint32_t StartSectorAddress, uint32_t *Data,
		uint16_t numberofwords) {
	int sofar = 0;
	HAL_FLASH_Unlock();
	while (sofar < numberofwords) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress,
				Data[sofar]) == HAL_OK) {
			StartSectorAddress += 4;
			sofar++;
		}
	}
	HAL_FLASH_Lock();
	return 0;
}

void Flash_Read_Data(uint32_t StartSectorAddress, uint32_t *RxBuf,
		uint16_t numberofwords) {
	while (1) {

		*RxBuf = *(__IO uint32_t*) StartSectorAddress;
		StartSectorAddress += 4;
		RxBuf++;
		if (!(numberofwords--))
			break;
	}
}

void Convert_To_Str(uint32_t *Data, char *Buf) {
	int numberofbytes = ((strlen((char*) Data) / 4)
			+ ((strlen((char*) Data) % 4) != 0)) * 4;

	for (int i = 0; i < numberofbytes; i++) {
		Buf[i] = Data[i / 4] >> (8 * (i % 4));
	}
}

