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


/**
 * @brief function to send formated header over UART
 * @param data_header the struct containing the data header information
 * @param UART_handle a pointer to the UART_Handle to send the data to
 */
void send_header(data_header_struct* Data_Header, UART_HandleTypeDef* UART_Handle)
{
	uint8_t uart_buffer[50];
	sprintf((char*) uart_buffer,"HEADER:P%dR%dY%dT%dN%d\n", Data_Header->desired_pitch,Data_Header->desired_roll,Data_Header->desired_yaw,Data_Header->loop_time,(int) Data_Header-> numb_measurements);
	HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
}

/**
 * @brief function to send collected data over UART
 * @param Data_Buff a pointer to an array of Numb_Measurements containing the the collected data
 * @param Numb_Measurements the size of the Data_Buff array
 * @param UART_handle a pointer to the UART_Handle to send the data to
 */
void send_collected_data(data_collection_struct* Data_Buff, int16_t Numb_Measurements, UART_HandleTypeDef* UART_Handle)
{
	for(int i = 0;i < Numb_Measurements; i++)
	{
		uint8_t uart_buffer[50];
		sprintf((char*) uart_buffer,"N%hdP%hdR%hdY%hd\n", Data_Buff[i].index, Data_Buff[i].pitch, Data_Buff[i].roll, Data_Buff[i].yaw);
		HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
	}
}

/**
 * @brief function to erase the in user_constants.h defined number of pages
 */
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

/**
 * @brief function to write Len number of Data elements to Adress in flash
 * @param Data a pointer to Len elements containing the data
 * @param Len the number of Data elements to be written
 * @param Adress the starting adress of the section of the flash
 */
void flash_write( uint64_t *Data, uint16_t Len, uint32_t Address)
{
    HAL_FLASH_Unlock();

    for (int i = 0; i < Len; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, Data[i]);
        Address += 8;
    }
    HAL_FLASH_Lock();
}


