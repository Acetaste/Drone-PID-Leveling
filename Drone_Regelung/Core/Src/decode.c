/*
 * decode.c
 *
 *  Created on: Nov 2, 2025
 *      Author: matth
 */
#include "decode.h"
#include <stdint.h>
char decode(int binary)
{
	char result;
	switch(binary)
	{
	case 3125149440:
		result = '1';
		break;
	case 3108437760:
		result = '2';
		break;
	case 3091726080:
		result = '3';
		break;
	case 3141861120:
		result = '4';
		break;
	case 3208707840:
		result = '5';
		break;
	case 3158572800:
		result = '6';
		break;
	case 4161273600:
		result = '7';
		break;
	case 3927310080:
		result = '8';
		break;
	case 4127850240:
		result = '9';
		break;
	case 3910598400:
		result = '*';
		break;
	case 3860463360:
		result = '0';
		break;
	case 4061003520:
		result = '#';
		break;
	case 3877175040:
		result = 'U';
		break;
	case 2907897600:
		result = 'D';
		break;
	case 4144561920:
		result = 'L';
		break;
	case 2774204160:
		result = 'R';
		break;
	case 3810328320:
		result = 'O';
		break;
	}
	return result;
}




