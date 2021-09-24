
#ifndef __UART_H
#define __UART_H

/* Includes ---------------------------------------------------------------------------------------------*/
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>

/* Defines ----------------------------------------------------------------------------------------------*/
#define DEV_USB       "/dev/ttyUSB"    /* USB Serial connection */
#define DEV_AMA       "/dev/serial"    /* raw serial device */
#define DEV_ACM       "/dev/ttyACM"    /* Virtual COM port */

/* Variables --------------------------------------------------------------------------------------------*/  
/* Function Prototypes ----------------------------------------------------------------------------------*/
int openSerialPort(char *device, int port);
void closeSerialPort(int);
int readSerial(unsigned char *, uint32_t);
void serialWrite(char *, uint32_t);
void signalHandler(int sig);

#endif /* __UART_H */
