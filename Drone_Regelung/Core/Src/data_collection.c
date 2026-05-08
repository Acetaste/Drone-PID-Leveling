/*
 * data_collection.c
 *
 *  Created on: 25.3 2026
 *      Author: matthias Ellinger
 */


#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "data_collection.h"
#include "user_constants.h"



void send_header(data_header_struct* data_header, UART_HandleTypeDef* UART_handle)
{
	uint8_t uart_buffer[50];
	sprintf((char*) uart_buffer,"HEADER:P%dR%dY%dT%dN%d\n", data_header->desired_pitch,data_header->desired_roll,data_header->desired_yaw,data_header->loop_time,(int) data_header-> numb_measurements);
	HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
}

void send_collected_data(data_collection_struct* data_buff, int16_t numb_measurements, UART_HandleTypeDef* UART_handle)
{
	for(int i = 0;i < numb_measurements; i++)
	{
		uint8_t uart_buffer[50];
		sprintf((char*) uart_buffer,"N%hdP%hdR%hdY%hd\n", data_buff[i].index, data_buff[i].pitch, data_buff[i].roll, data_buff[i].yaw);
		HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
	}
}

void erase_pages(void)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page = (FLASH_REGION_START - FLASH_BASE) / FLASH_PAGE_SIZE;
    EraseInitStruct.NbPages = FLASH_NUMB_PAGES;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    HAL_FLASH_Lock();

}



void flash_write( uint64_t *data, uint16_t len, uint32_t address)
{
    HAL_FLASH_Unlock();

    for (int i = 0; i < len; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data[i]);
        address += 8;
    }
    HAL_FLASH_Lock();
}


