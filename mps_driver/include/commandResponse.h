/*********************************************************************************************************
*                                       Copyright Notice
*
*********************************************************************************************************/

/*********************************************************************************************************
*                                  COMMAND & RESPONSE OPERATIONS
* Filename      : commandResponse.h
* Version       : V1.0.0
* Programmers(s): Xiang He
**********************************************************************************************************
* Notes         : (1) n/a
*/

#ifndef __COMMAND_RESPONSE_H
#define __COMMAND_RESPONSE_H

/* Includes ---------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <wiringPiI2C.h>
#include <wiringPi.h>

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <mps_driver/MPS.h>

using namespace std;
using namespace ros;

/* Defines ----------------------------------------------------------------------------------------------*/
#define HEADER_LENGTH	6
#define HEADER          0x7E
#define VERSION         0x0001          /* packet version */

#define CTRL_ANS_ENGINE 0x000B
#define UAV_ANSWER      0x000C

#define CHECKSUM_ERROR  0x0001
#define BAD_COMMAND     0x0002
#define SUCCESS         0x0003

#define GET_U16(p)  (uint16_t) (((p)[0] << 8) | (p)[1])
#define GET_U32(p)  (uint32_t) (((p)[0] << 24) | ((p)[1] << 16) | ((p)[2] << 8) | (p)[3])
#define GET_F32(d, s)  *(uint32_t *) &(d) = GET_U32(s)
#define PUT_U16(p, v)  { ((uint8_t *)(p))[0] = (((v) >> 8) & 0xff); ((uint8_t *)(p))[1] = ((v) & 0xff); }
#define PUT_U32(p, v)  { ((uint8_t *)(p))[0] = (((v) >> 24) & 0xff); ((uint8_t *)(p))[1] = (((v) >> 16) & 0xff); ((uint8_t *)(p))[2] = (((v) >> 8) & 0xff); ((uint8_t *)(p))[3] = ((v) & 0xff); }
#define PUT_F32(p, v)  { ((uint8_t *)(p))[0] = (((*(int *)&(v)) >> 24) & 0xff); ((uint8_t *)(p))[1] = (((*(int *)&(v)) >> 16) & 0xff); ((uint8_t *)(p))[2] = (((*(int *)&(v)) >> 8) & 0xff); ((uint8_t *)(p))[3] = ((*(int *)&(v)) & 0xff); }

/* Structures -------------------------------------------------------------------------------------------*/
/* Variables --------------------------------------------------------------------------------------------*/  
/* Function Prototypes ----------------------------------------------------------------------------------*/
int sendCommand_I2C(uint16_t, uint8_t *, uint16_t, int fd);
uint8_t* receiveData_I2C(uint16_t, uint16_t *, uint32_t, int fd); 


#endif /* __COMMAND_RESPONSE_H */
