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
	case 977665792:
		result = '1';
		break;
	case 960954112 :
		result = '2';
		break;
	case 944242432:
		result = '3';
		break;
	case 994377472:
		result = '4';
		break;
	case 1061224192:
		result = '5';
		break;
	case 1011089152:
		result = '6';
		break;
	case 2013789952:
		result = '7';
		break;
	case 1779826432:
		result = '8';
		break;
	case 1980366592:
		result = '9';
		break;
	case 1763114752:
		result = '*';
		break;
	case 1712979712:
		result = '0';
		break;
	case 1913519872 :
		result = '#';
		break;
	case  1729691392:
		result = 'U';
		break;
	case 760413952:
		result = 'D';
		break;
	case 1997078272:
		result = 'L';
		break;
	case 626720512:
		result = 'R';
		break;
	case 1662844672:
		result = 'O';
		break;
	}
	return result;
}




