/*********************************************************************************************************
*                                       Copyright Notice
*
*********************************************************************************************************/

/********************************************************************************************************==*
*                                      UART test client for NNTS
* Filename      : uartTest.c
* Version       : V1.1.0
* Programmers(s): Hank Yung
**********************************************************************************************************
* Notes         : See ...
*/
#define __MAIN_C

/* Includes ---------------------------------------------------------------------------------------------*/
#include <errno.h>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>

#include <iostream>
#include "uart.h"
#include "checksum.h"

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <mps_driver/MPS.h>
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"


using namespace std;
using namespace ros;


/* Defines ----------------------------------------------------------------------------------------------*/
/*
 * Conversion macros for switching between Little and Big Endian.
*/
#define FLAMMABLE
#define INTERNAL_USE
#define SWAP16(num)        (((num & 0xff00) >> 8) | (num << 8))
#define SWAP32(num)        (((num & 0xff000000) >> 24) | ((num & 0x00ff0000) >> 8) | ((num & 0x0000ff00) << 8) | (num << 24))

/* Command Status */
#define UART_SUCCESS           0x00
#define UART_CRC_ERROR         0x01
#define UART_BAD_PARAM         0x02
#define UART_EXE_FAILED        0x03
#define UART_NO_MEM            0x04
#define UART_UNKNOWN_CMD       0x05

#define UART_LOCAL_ERROR       0xFF   /* Error generated locally - not from sensor */

/* commands */
#define CMD_ANSWER       0x01
#define CMD_DRONE        0x99
#define CMD_ENGDATA      0x02
#ifdef FLAMMABLE
#define CMD_CONC         0x03
#define CMD_ID           0x04
#endif

#define CMD_TEMP         0x21
#define CMD_PRES         0x22
#define CMD_REL_HUM      0x23
#define CMD_ABS_HUM      0x24

#define CMD_STATUS       0x41
#define CMD_VERSION      0x42

#define CMD_MEAS               0x61
#define CMD_SHUTDOWN           0x62
#define CMD_BOOTLOADER         0x63
#define CMD_WRITE_FW           0x64
#define CMD_CLEAR_HISTORY      0x65
#define CMD_REBOOT             0x66
#define CMD_ANALOG_OUT_V       0x67

#define RQST_HDR_LENGTH     sizeof(uartRqstHeader_t)
#define REPLY_HDR_LENGTH    sizeof(uartReplyHeader_t)
#define NUM_OF_CMDS         (sizeof(uart_cmds) / sizeof(uart_cmd_t))
#define UART_MAX_DATA_SIZE  (1024*5)    /* maximum packet:  header + payload */
#define ENGDATA_CHUNKSIZE   512         /* size of each chunk of engineering data */
#define FINAL_PACKET        0x8000      /* bit to indicate last chunk of engineering data */

#define GAS_NAME_LENGTH     64
#ifdef INTERNAL_USE
#define MAX_JSON_SIZE       4480
#define CONSOLE_BUFSIZE     2048
#endif

/* Structure definitions --------------------------------------------------------------------------------*/
typedef struct {
  uint16_t cmdID;
  uint16_t length;
  uint16_t reserved;
  uint16_t cksum;
} uartRqstHeader_t;

typedef struct {
  uint8_t cmdID;
  uint8_t status;
  uint16_t length;
  uint16_t cksum;
} uartReplyHeader_t;

typedef struct {
  uint8_t cmdID;
  uint16_t req_size;   /* Request size */
  uint16_t res_size;   /* Response size */
  uint32_t (*func)(uint8_t cmdID, uint8_t *data, uint16_t size);
} uart_cmd_t;

typedef struct {
  uint8_t sw_w;
  uint8_t sw_x;
  uint8_t sw_y;
  uint8_t sw_z;
  uint8_t hw_w;
  uint8_t hw_x;
  uint8_t proto_w;
  uint8_t proto_x;
} uart_version_t;

#ifdef FLAMMABLE
typedef struct {
  uint32_t cycleCount;
  float concentration;
  uint32_t flamID;
  float temp;
  float pressure;
  float relHumidity;
  float absHumidity;
} answer_t;
#else
#error Need to define expected answer type!
#endif

typedef struct {
  uint16_t pktnum;
  uint16_t length;
  uint8_t data[ENGDATA_CHUNKSIZE];
} uart_engdata_t;

typedef struct {
  float temp;
  float pressure;
  float humidity;
  float absHumidity;
  float humidAirDensity;
} enviro_reply_t;

#ifdef INTERNAL_USE
typedef struct {
  uint32_t status;       /* Extend status from uint8_t to uint32_t to make word alignment */
  uint32_t cycleCount;   /* starts from 1 with first complete cycle */
  float concentration;
  uint32_t flamID;
  float temp;
  float pressure;
  float relHumidity;
  float absHumidity;
#ifdef BACKUP_BME280
  float tempB;
  float humidityB;
#endif
} flam_answer_t;

typedef struct {
  flam_answer_t answer;
  float dDSC1MidCompNorm;    /* answer.flamAnswer.dDSC1MidCompNorm */
  float dDSC1MaxCompNorm;    /* answer.flamAnswer.dDSC1MaxCompNorm */
  float midKelvinPower;      /* answer.dscResults[0].feature.midPower (mid kelvin power) */
  float maxKelvinPower;      /* answer.dscResults[0].feature.maxPower (max kelvin power) */
  float midKelvinResistance; /* answer.dscResults[0].feature.midResistance (mid kelvin resistance) */
  float maxKelvinResistance; /* answer.dscResults[0].feature.maxResistance (max kelvin resistance) */
  float midTotalPower;       /* answer.dscResults[0].heater.totalPower[0] (mid total power) */
  float maxTotalPower;       /* answer.dscResults[0].heater.totalPower[1] (max total power) */
  float midTotalResistance;  /* answer.dscResults[0].heater.totalResistance[0] (mid total resistance) */
  float maxTotalResistance;  /* answer.dscResults[0].heater.totalResistance[1] (max total resistance) */
  float ambientResistance_k; /* answer.dscResults[0].feature.ambientResistance_k */
  float ambientResistance_t; /* answer.dscResults[0].feature.ambientResistance_t */
} flam_engdata_t;
#endif

/* Functions --------------------------------------------------------------------------------------------*/
//extern int openSerialPort(char *device, int port);
//extern void closeSerialPort(int);
//extern int readSerial(char *readBuffer, uint32_t bytesToRead);
//extern uint16_t crc_generate(uint8_t *buffer, size_t length, uint16_t startValue);
static uint32_t uartSingleRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
static uint8_t uartSend(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
static uint8_t uartReSend(uint8_t cmdID);
static uint32_t uartRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
static uint32_t ReadFloat(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadInteger(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadVersion(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadString(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadAnswer(uint8_t cmdID, uint8_t *data, uint16_t size);
#ifdef INTERNAL_USE
static void DumpEngDataCSV(FILE *fp, uint8_t *data);
#endif
static uint32_t ReadByte(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t WriteByte(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t WriteFloat(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadEngData(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadEngDataDRONE(uint8_t cmdID, uint8_t *data, uint16_t size);
static void DumpRqstHdr(uartRqstHeader_t *);
static void DumpReplyHdr(uartReplyHeader_t *);
static void DumpHexa(uint8_t *p, uint32_t len);

/* Variables --------------------------------------------------------------------------------------------*/
int uartFP;
uint32_t verbose = 0, hexdump = 0;
#ifdef INTERNAL_USE
uint32_t saveInZionFormat = 0;
char *timestamp = "";
#endif
uint32_t numOfRetries = 0;
uint32_t rxTimeout = 0, rxBytes = 0, uartState = 0;
static uartRqstHeader_t pktHdrCache;
static uint8_t payloadCache[256];
static uint32_t payloadCacheLen = 0;
char *filename = NULL;
uart_cmd_t uart_cmds[] = {
  {CMD_ANSWER, 0, sizeof(answer_t), ReadAnswer},
  {CMD_MEAS, 1, 0, WriteByte},
#ifdef FLAMMABLE
  {CMD_CONC, 0, 4, ReadFloat},
  {CMD_ID, 0, 4, ReadInteger},
#endif
  {CMD_ENGDATA, 0, sizeof(uart_engdata_t), ReadEngData},
  {CMD_DRONE  , 0, sizeof(uart_engdata_t), ReadEngDataDRONE},
  {CMD_TEMP, 0, 4, ReadFloat},
  {CMD_PRES, 0, 4, ReadFloat},
  {CMD_REL_HUM, 0, 4, ReadFloat},
  {CMD_ABS_HUM, 0, 4, ReadFloat},
  {CMD_STATUS, 0, 1, ReadByte},
  {CMD_VERSION, 0, 8, ReadVersion},
  {CMD_SHUTDOWN, 0, 0, WriteByte},
  {CMD_BOOTLOADER, 0, 0, WriteByte}
};
answer_t *answer;
flam_engdata_t *answer_eng;

void usage(void) {
  printf("\nUsage:\n");
  printf("  uartTest -c <cmdID> [ -v <value>]\n");
  printf("   -c:  <cmdID> to execute in hex (e.g. 0x61)\n");
  printf("   -D:  <device> name (default:  /dev/ttyAMA)\n");
  printf("   -f:  full path of the file <name> (e.g. /tmp/abc.bin)\n");
  printf("   -p:  COM port number (default: port 0)\n");
  printf("   -v:  <value> of parameter to send with the command in hex (e.g. 0x01)\n");
  printf("   -V:  verbose mode\n");
  printf("   -x:  hex dump\n");
#ifdef INTERNAL_USE
  printf("   -z:  Dump engineering data in 'Zion' format\n");
#endif
  printf("\nExamples:\n");
  printf("  uartTest -c 0x21 (default: /dev/ttyAMA0, read temp)\n");
  printf("  uartTest -D /dev/ttyUSB -p 1 -c 0x42 (/dev/ttyUSB1, read version information)\n");
  printf("  uartTest -D /dev/ttyAMA -p 1 -c 0x01 (/dev/ttyAMA1, read answer)\n");
  printf("  uartTest -D /dev/ttyAMA -p 0 -c 0x61 -v 0x2 (/dev/ttyAMA0, do 'continuous' measurement; set MEAS to 2)\n");
#ifdef INTERNAL_USE
  printf("  uartTest -D /dev/ttyAMA -p 0 -c 0x02 -z -f /tmp/engdata.csv (/dev/ttyAMA0, dump 'engineering data' in CSV ('Zion') format into file /tmp/engdata.csv)\n");
#endif
  exit(1);
}

static uint8_t uartSend(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  uartRqstHeader_t header;
  uint16_t cksum, rxCksum, length;

  memset(&header, 0, RQST_HDR_LENGTH);
  header.cmdID = cmdID;
  header.length = payloadLen;

  cksum = crc_generate((uint8_t *) &header, RQST_HDR_LENGTH, 0xFFFF);
  header.cksum = cksum;

  if(payloadLen != 0) {
    if(payload == NULL) {
      printf("No payload given but payload lengh is non-zero\n");
      return 1;
    }
    cksum = crc_generate(payload, payloadLen, cksum);
  }
  header.cksum = cksum;

  if(verbose) {
    DumpRqstHdr(&header);
    if(hexdump)
      DumpHexa((uint8_t *) &header, RQST_HDR_LENGTH);
  }
  
  if(write(uartFP, (uint8_t *) &header, RQST_HDR_LENGTH) != RQST_HDR_LENGTH) {
    printf("Failed to send header: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
    return 1;
  }
  if(numOfRetries != 0) {
    pktHdrCache = header;
  }

  if(payloadLen) {
    if(hexdump) {
      printf("  Payload");
      DumpHexa(payload, payloadLen);
    }

    if(write(uartFP, payload, payloadLen) != payloadLen) {
      printf("Failed to send payload: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
      return 1;
    }

    if(numOfRetries != 0) {
      memcpy(payloadCache, payload, payloadLen);
      payloadCacheLen = payloadLen;
    }
  }

  return 0;
}

uint32_t uartRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  int32_t retry = 1;
  uint32_t status;

  status = uartSingleRecv(cmdID, payload, payloadLen);
  if((status == UART_SUCCESS) || (numOfRetries == 0))
    return status;

  do {
    if((status = uartReSend(cmdID)) != 0) {
      break;
    }

    status = uartSingleRecv(cmdID, payload, payloadLen);
  } while ((retry++ < numOfRetries) && (status != UART_SUCCESS));

  return status;
}

static uint32_t uartSingleRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  uint16_t rxCksum, cksum;
  int rxLen;
  uint32_t timeout;
  uartRqstHeader_t header;
  uartReplyHeader_t *reply;
  uint8_t buffer[UART_MAX_DATA_SIZE+1];

  memset(buffer, 0, sizeof(buffer));

  rxLen = readSerial(buffer, sizeof(uartReplyHeader_t));
  if(rxLen <= 0) {
    printf("Failed to get reply: %s (%d)\n", strerror(errno),  errno);
    return UART_LOCAL_ERROR;
  }

  reply = (uartReplyHeader_t *) buffer;
  if(rxLen < REPLY_HDR_LENGTH) {
    printf("Incomplete header received: %d bytes\n", rxLen);
    DumpReplyHdr(reply);
    return UART_LOCAL_ERROR;
  }

  if(reply->length != 0) {  /* Is there a payload for this reply? */
    rxLen = readSerial(&buffer[REPLY_HDR_LENGTH], reply->length);
    if(rxLen < reply->length) {
      printf("Failed to get reply payload: %s (%d)\n", strerror(errno),  errno);
      return UART_LOCAL_ERROR;
    }
  }

  rxCksum = reply->cksum;
  reply->cksum = 0;  /* zero out checksum field */
  cksum = crc_generate(buffer, REPLY_HDR_LENGTH + reply->length, 0xFFFF);
  if(rxCksum != cksum) {
    printf("Checksum failed: expected 0x%x, received 0x%x\n", cksum, rxCksum);
    reply->cksum = rxCksum;   /* restore received checksum */
    DumpReplyHdr(reply);
    return UART_LOCAL_ERROR;
  }

  reply->cksum = rxCksum;   /* restore received checksum */

  if(reply->status != UART_SUCCESS) {
    printf("Command returned error status: 0x%x\n", reply->status);
    DumpReplyHdr(reply);
    return (reply->status);  /* Sensor sent error status */
  }

  if(reply->cmdID != cmdID) {
    printf("cmdID mismatch: expected 0x%x, received 0x%x\n", cmdID, reply->cmdID);
    DumpReplyHdr(reply);
    return UART_LOCAL_ERROR;
  }

  if (reply->length == 0)
    return UART_SUCCESS;  /* No payload, we are done. */

  if(payloadLen < reply->length) {
    printf("Buffer too small for payload (%d < %d)\n", payloadLen, reply->length);
    return UART_LOCAL_ERROR;
  }

  memset(payload, 0, payloadLen);
  memcpy(payload, &buffer[REPLY_HDR_LENGTH], reply->length);
  return UART_SUCCESS;
}

static uint8_t uartReSend(uint8_t cmdID) {
  uartReplyHeader_t reply;
  uint16_t cksum, rxCksum, length;

  if(write(uartFP, (uint8_t *) &pktHdrCache, RQST_HDR_LENGTH) != RQST_HDR_LENGTH) {
    printf("Failed to send header: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
    return 1;
  }

  if(payloadCacheLen) {
    if(write(uartFP, payloadCache, payloadCacheLen) != payloadCacheLen) {
      printf("Failed to send payload: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
      return 1;
    }
  }

  return 0;
}

uint32_t run_MPS(uint8_t cmdID, uint32_t value, answer_t *answer_MPS){
  int ii;
  uint32_t sts;
  uint8_t reply[UART_MAX_DATA_SIZE];

  for(ii = 0; ii < NUM_OF_CMDS; ii++) {
    if(uart_cmds[ii].cmdID != cmdID)
      continue;
    
    if(uart_cmds[ii].req_size) {
      sts = uart_cmds[ii].func(cmdID, (uint8_t *) &value, uart_cmds[ii].req_size);
    } else if(uart_cmds[ii].res_size) {
      sts = uart_cmds[ii].func(cmdID, reply, uart_cmds[ii].res_size);
    } else {
      sts = uart_cmds[ii].func(cmdID, NULL, 0);
    }
    break;
  }

  if(ii == NUM_OF_CMDS) {
    printf("No such command: 0x%x\n", cmdID);
    sts = 1;
  }

  if(cmdID == CMD_ANSWER){
    *answer_MPS = *answer;
  }

  return sts;
  //closeSerialPort(uartFP);
  //exit(sts);

}

uint32_t run_MPS(uint8_t cmdID, uint32_t value, flam_engdata_t *answer_MPS){
  int ii;
  uint32_t sts;
  uint8_t reply[UART_MAX_DATA_SIZE];

  for(ii = 0; ii < NUM_OF_CMDS; ii++) {
    if(uart_cmds[ii].cmdID != cmdID)
      continue;
    
    if(uart_cmds[ii].req_size) {
      sts = uart_cmds[ii].func(cmdID, (uint8_t *) &value, uart_cmds[ii].req_size);
    } else if(uart_cmds[ii].res_size) {
      sts = uart_cmds[ii].func(cmdID, reply, uart_cmds[ii].res_size);
    } else {
      sts = uart_cmds[ii].func(cmdID, NULL, 0);
    }
    break;
  }

  if(ii == NUM_OF_CMDS) {
    printf("No such command: 0x%x\n", cmdID);
    sts = 1;
  }

  if(cmdID == CMD_DRONE){
    *answer_MPS = *answer_eng;
  }

  return sts;
  //closeSerialPort(uartFP);
  //exit(sts);

}


static uint32_t ReadFloat(uint8_t cmdID, uint8_t *data, uint16_t size) {
  float *value;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  value = (float *) data;
  printf("Command[0x%02x]: %f\n", cmdID, *value);

  return 0;
}

static uint32_t ReadInteger(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uint32_t *value;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  value = (uint32_t *) data;
  printf("Command[0x%02x]: %u\n", cmdID, *value);

  return 0;
}

static uint32_t ReadVersion(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uart_version_t *version;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  version = (uart_version_t *) data;
  printf("SW Version: %u.%u.%u.%u, HW Version: %u.%u, Protocol: %u.%u\n",
         version->sw_w, version->sw_x, version->sw_y, version->sw_z,
         version->hw_w, version->hw_x, version->proto_w, version->proto_x);
  return 0;
}

#ifdef INTERNAL_USE
static void DumpEngDataCSV(FILE *fp, uint8_t *data) {
  flam_engdata_t *engData = (flam_engdata_t *) data;

    fprintf(fp, "%s, %d, %f, %f, %f, %f, %f, %f, %f, %f, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
            timestamp,
            SWAP32(engData->answer.cycleCount), engData->answer.temp, engData->answer.pressure, engData->answer.relHumidity,
            engData->answer.absHumidity, 0.0, 0.0, engData->dDSC1MidCompNorm, engData->dDSC1MaxCompNorm,
            SWAP32(engData->answer.flamID), engData->answer.concentration, engData->midKelvinPower, engData->maxKelvinPower,
            engData->midTotalResistance, engData->maxTotalResistance,
            engData->midTotalPower, engData->maxTotalPower,
            engData->ambientResistance_t, engData->ambientResistance_k,
            engData->midKelvinResistance, engData->maxKelvinResistance);
    printf("%s, %d, %f, %f, %f, %f, %f, %f\n",
           timestamp,
           SWAP32(engData->answer.cycleCount), engData->answer.temp, engData->answer.pressure, engData->answer.relHumidity,
           engData->answer.absHumidity, 0.0, 0.0);
}
#endif

static uint32_t ReadString(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uart_version_t *version;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  printf("%s\n", data);
  return 0;
}

static uint32_t ReadAnswer(uint8_t cmdID, uint8_t *data, uint16_t size) {

 if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

 if(uartRecv(cmdID, data, size) != 0)
   return 1;
  
 answer = (answer_t *) data;
  /*
#ifdef FLAMMABLE && defined(DEBUG)
  printf("Cycle: %u\nGas: %d\nConcentration: %f\nTEMP: %f\nPRESS: %f\nREL_HUM: %f\nABS_HUM: %f\n",
         SWAP32(answer->cycleCount), answer->flamID, answer->concentration, answer->temp, answer->pressure, answer->relHumidity, answer->absHumidity);
#endif
  */

  return 0;
}

static uint32_t ReadByte(uint8_t cmdID, uint8_t *data, uint16_t size) {

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  printf("Command[0x%02x]: 0x%x\n", cmdID, *data);

  return 0;
}

static uint32_t WriteByte(uint8_t cmdID, uint8_t *data, uint16_t size) {
  if(uartSend(cmdID, data, size) != 0)
    return 1;
  
  if(uartRecv(cmdID, NULL, 0) != 0)
    return 1;

  return 0;
}

static uint32_t WriteFloat(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uint32_t val;
  float fval;

  val = *((uint32_t *) data);
  fval = ((float) val) / 100.0;

  printf("%s: %d %f\n", __FUNCTION__, val, fval);
  if(uartSend(cmdID, (uint8_t *) &fval, sizeof(fval)) != 0)
    return 1;

  if(uartRecv(cmdID, NULL, 0) != 0)
    return 1;

  return 0;
}

static uint32_t ReadEngData(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uint16_t pktnum = 0, txPktnum, rxPktnum;
  uart_engdata_t *engdata;
  uint8_t *p;
  uint16_t crc = 0, length;
  FILE *fp;
#ifdef INTERNAL_USE
  uint32_t writeHeaders = 0;
  struct stat statbuf;
#endif

  if(filename == NULL) {
    printf("Missing filename parameters ('-f' option)\n");
    return 0;
  }

#ifdef INTERNAL_USE
  if(saveInZionFormat) {
    if(stat(filename, &statbuf) < 0)
      writeHeaders = 1;   /* new file - write column headers */
    fp = fopen(filename, "a");
  }
  else
    fp = fopen(filename, "ab");
#else
  fp = fopen(filename, "ab");
#endif
  if(fp == NULL) {
    printf("File open (%s) failed: %s\n", filename, strerror(errno));
    return 1;
  }

#ifdef INTERNAL_USE
  if(writeHeaders) {
      fprintf(fp,  "Time [s], Cycle , T [C], P [kPa], RH [%%], AH [g/m3], HAD [kg/m3], PPMH2O [PPM], dDSCMidCompNorm [%%LEL], dDSCMaxCompNorm [%%LEL], FlamID , FlamConc [%%LEL], DSC1MidP_K [mW], DSC1MaxP_K [mW], DSC1MidR_T [Ohms], DSC1MaxR_T [Ohms], DSC1MidP_T [mW], DSC1MaxP_T [mW], DSC1AmbR_T [Ohms],DSC1AmbR_K [Ohms], DSC1MidR_K [Ohms], DSC1MaxR_K [Ohms]\n");
  }
#endif

  do {
    txPktnum = SWAP16(pktnum);  /* outbound -> Big Endian */
    if(uartSend(cmdID, (uint8_t*) &txPktnum, sizeof(txPktnum)) != 0)
      return 1;

    if(uartRecv(cmdID, data, size) != 0) {
      printf("Failed to read\n");
      return 1;
    }

    engdata = (uart_engdata_t *) data;
    rxPktnum = SWAP16(engdata->pktnum);  /* inbound -> Little Endian */
    rxPktnum &= ~FINAL_PACKET;
    if(rxPktnum != pktnum) {
      printf("Unexpected packet # received (%d vs %d)\n", pktnum, rxPktnum);
      break;
    }

    length = SWAP16(engdata->length);
#ifdef INTERNAL_USE
    if(saveInZionFormat) {
        DumpEngDataCSV(fp, engdata->data);
    } else {
        fwrite(engdata->data, ENGDATA_CHUNKSIZE, 1, fp);
    }
    fflush(stdout);
#else
    fwrite(engdata->data, ENGDATA_CHUNKSIZE, 1, fp);
    fflush(stdout);
#endif
    pktnum++;
  } while ((SWAP16(engdata->pktnum) & FINAL_PACKET) == 0);  /* Continue while FINAL_PACKET bit is not set */

  fclose(fp);
  return 0;
}

static uint32_t ReadEngDataDRONE(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uint16_t pktnum = 0, txPktnum, rxPktnum;
  uart_engdata_t *engdata;
  uint8_t *p;
  uint16_t crc = 0, length;
  FILE *fp;
  cmdID=0x02;
  
#ifdef INTERNAL_USE
  uint32_t writeHeaders = 0;
  struct stat statbuf;
#endif
  
  do {
    txPktnum = SWAP16(pktnum);  /* outbound -> Big Endian */
    if(uartSend(cmdID, (uint8_t*) &txPktnum, sizeof(txPktnum)) != 0)
      return 1;

    if(uartRecv(cmdID, data, size) != 0) {
      printf("Failed to read\n");
      return 1;
    }

    engdata = (uart_engdata_t *) data;
    answer_eng = (flam_engdata_t *) (engdata->data);
        
    rxPktnum = SWAP16(engdata->pktnum);  /* inbound -> Little Endian */
    rxPktnum &= ~FINAL_PACKET;
    if(rxPktnum != pktnum) {
      printf("Unexpected packet # received (%d vs %d)\n", pktnum, rxPktnum);
      break;
    }

    length = SWAP16(engdata->length);
    pktnum++;
  } while ((SWAP16(engdata->pktnum) & FINAL_PACKET) == 0);  /* Continue while FINAL_PACKET bit is not set */
  
  return 0;
}


static void DumpRqstHdr(uartRqstHeader_t *rqst) {
  printf("----\nREQUEST:\n");
  printf("  Hdr Size: %lu\n", sizeof(uartRqstHeader_t));
  printf("  CmdID: 0x%x\n", rqst->cmdID);
  printf("  Length: %d\n", rqst->length);
  printf("  Reserved: 0x%x\n", rqst->reserved);
  printf("  Checksum: 0x%x\n", rqst->cksum);
}

static void DumpReplyHdr(uartReplyHeader_t *reply) {
  printf("----\nREPLY:\n");
  printf("  CmdID: 0x%x\n", reply->cmdID);
  printf("  Status: 0x%x\n", reply->status);
  printf("  Length: %d\n", reply->length);
  printf("  Checksum: 0x%x\n", reply->cksum);
}

static void DumpHexa(uint8_t  *p, uint32_t len) {
  int ii;

  for(ii = 0; ii < len; ii++) {
    if((ii % 8) == 0)
      printf("\n    [%02d]: ", ii);

    printf("0x%02x ", *p++);
  }
  printf("\n");
}
