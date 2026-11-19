#include "command_parser.h"
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#define QUEUE_SIZE 32U
static char lines[QUEUE_SIZE][COMMAND_LINE_SIZE],building[COMMAND_LINE_SIZE];
static unsigned head,tail,position;
static uint8_t discard;
static uint32_t last_byte_ms;
static CommandStats stats;
void CommandParser_Init(void){memset(&stats,0,sizeof(stats));head=tail=position=0;discard=0;last_byte_ms=0;}
void CommandParser_Clear(void){head=tail=position=0;discard=0;}
void CommandParser_DiscardPartial(void){position=0;discard=1;stats.malformed++;}
void CommandParser_Feed(const uint8_t *bytes,size_t length,uint32_t now) {
  if(position && (uint32_t)(now-last_byte_ms)>500U){discard=1;position=0;stats.malformed++;}
  last_byte_ms=now;
  for(size_t i=0;i<length;i++) {
    unsigned char ch=bytes[i];
    if(ch=='\r')continue;
    if(ch=='\n') {
      if(!discard && position) {
        building[position]=0;
        if(head-tail<QUEUE_SIZE){memcpy(lines[head%QUEUE_SIZE],building,position+1U);head++;stats.accepted++;}
        else stats.queue_dropped++;
      }
      position=0;discard=0;continue;
    }
    if(discard)continue;
    if((ch<32U && ch!='\t') || ch>126U || position>=COMMAND_LINE_SIZE-1U) {
      stats.malformed++;discard=1;position=0;continue;
    }
    building[position++]=(char)toupper(ch);
  }
}
static uint8_t ParseDecimal(
    const char *text,
    const char **end_text,
    float *value)
{
  const char *cursor = text;
  float sign = 1.0f;
  uint32_t integer_part = 0U;
  uint32_t fractional_part = 0U;
  uint32_t fractional_scale = 1U;
  uint8_t has_digit = 0U;

  if (*cursor == '-')
  {
    sign = -1.0f;
    cursor++;
  }
  else if (*cursor == '+')
  {
    cursor++;
  }

  while (isdigit((unsigned char)*cursor))
  {
    has_digit = 1U;
    if (integer_part > 100000U) return 0U;
    integer_part = integer_part * 10U + (uint32_t)(*cursor - '0');
    cursor++;
  }

  if (*cursor == '.')
  {
    cursor++;
    while (isdigit((unsigned char)*cursor))
    {
      has_digit = 1U;
      if (fractional_scale < 1000000U)
      {
        fractional_part = fractional_part * 10U
                        + (uint32_t)(*cursor - '0');
        fractional_scale *= 10U;
      }
      cursor++;
    }
  }

  if (has_digit == 0U) return 0U;
  *value = sign * ((float)integer_part
      + (float)fractional_part / (float)fractional_scale);
  *end_text = cursor;
  return 1U;
}

static uint8_t ParseThreeFloats(const char *text, float values[3])
{
  const char *cursor = text;
  float parsed[3];

  for (uint32_t i = 0U; i < 3U; i++)
  {
    while (isspace((unsigned char)*cursor)) cursor++;
    const char *end = NULL;
    if (ParseDecimal(cursor, &end, &parsed[i]) == 0U) return 0U;
    cursor = end;

    if (i < 2U)
    {
      if (*cursor == ',')
      {
        cursor++;
      }
      else if (isspace((unsigned char)*cursor))
      {
        while (isspace((unsigned char)*cursor)) cursor++;
      }
      else
      {
        return 0U;
      }
    }
  }

  while (isspace((unsigned char)*cursor)) cursor++;
  if (*cursor != '\0') return 0U;
  memcpy(values, parsed, sizeof(parsed));
  return 1U;
}


static UsbCommand_t classify(const char *command,ParsedCommand *out) {
    if ((strcmp(command, "CAL GYRO") == 0)
        || (strcmp(command, "CAL_GYRO") == 0)) return USB_COMMAND_CAL_GYRO;
    if ((strcmp(command, "CAL MAG") == 0)
        || (strcmp(command, "CAL_MAG") == 0)) return USB_COMMAND_CAL_MAG;
    if ((strcmp(command, "CAL ACCEL START") == 0)
        || (strcmp(command, "CAL_ACCEL") == 0)
        || (strcmp(command, "CAL_ACCEL_START") == 0)) return USB_COMMAND_CAL_ACCEL_START;
    if ((strcmp(command, "CAL ACCEL CAPTURE") == 0)
        || (strcmp(command, "CAL_ACCEL_CAPTURE") == 0)) return USB_COMMAND_CAL_ACCEL_CAPTURE;
    if ((strcmp(command, "CAL ACCEL CANCEL") == 0)
        || (strcmp(command, "CAL_ACCEL_CANCEL") == 0)) return USB_COMMAND_CAL_ACCEL_CANCEL;
    if ((strcmp(command, "ZERO YAW") == 0)
        || (strcmp(command, "ZERO_YAW") == 0)) return USB_COMMAND_ZERO_YAW;
    if (strcmp(command, "STATUS") == 0) return USB_COMMAND_STATUS;
    const char *setpoint_text = NULL;
    if (strncmp(command, "SET_ATTITUDE,", 13U) == 0)
    {
      setpoint_text = &command[13];
    }
    else if (strncmp(command, "SET ATTITUDE ", 13U) == 0)
    {
      setpoint_text = &command[13];
    }
    if (setpoint_text != NULL)
    {
      float parsed_setpoint[3];
      if ((ParseThreeFloats(setpoint_text, parsed_setpoint) != 0U)
          && (parsed_setpoint[0] >= -180.0f)
          && (parsed_setpoint[0] <= 180.0f)
          && (parsed_setpoint[1] >= -89.0f)
          && (parsed_setpoint[1] <= 89.0f)
          && (parsed_setpoint[2] >= -180.0f)
          && (parsed_setpoint[2] <= 180.0f))
      {
        memcpy(out->values, parsed_setpoint, sizeof(parsed_setpoint));
        return USB_COMMAND_SET_ATTITUDE;
      }
    }
    {
      char axis_name[8] = {0};
      unsigned int enabled = 0U;int consumed=0;
      if ((sscanf(command, "PID,%7[A-Z],%u%n", axis_name, &enabled,&consumed) == 2)
          || (sscanf(command, "PID %7[A-Z] %u%n", axis_name, &enabled,&consumed) == 2))
      {
        if(command[consumed] || enabled>1U)return USB_COMMAND_UNKNOWN;
        if (strcmp(axis_name, "ROLL") == 0) out->axis = 0;
        else if (strcmp(axis_name, "PITCH") == 0) out->axis = 1;
        else if (strcmp(axis_name, "YAW") == 0) out->axis = 2;
        else if (strcmp(axis_name, "ALL") == 0) out->axis = 3;
        else return USB_COMMAND_UNKNOWN;
        out->enabled = (enabled != 0U) ? 1U : 0U;
        return USB_COMMAND_PID;
      }
    }
    if ((strcmp(command, "MODE,WORKING") == 0)
        || (strcmp(command, "SET_MODE,WORKING") == 0)
        || (strcmp(command, "MODE WORKING") == 0)) return USB_COMMAND_MODE_WORKING;
    if ((strcmp(command, "MODE,CHARGING") == 0)
        || (strcmp(command, "SET_MODE,CHARGING") == 0)
        || (strcmp(command, "MODE CHARGING") == 0)) return USB_COMMAND_MODE_CHARGING;
    if ((strcmp(command, "EMERGENCY_STOP") == 0)
        || (strcmp(command, "ESTOP") == 0)) return USB_COMMAND_ESTOP;
    if ((strcmp(command, "EMERGENCY_STOP_CLEAR") == 0)
        || (strcmp(command, "ESTOP_CLEAR") == 0)) return USB_COMMAND_ESTOP_CLEAR;
    if (command[0] != '\0') return USB_COMMAND_UNKNOWN;

  return USB_COMMAND_UNKNOWN;
}
int CommandParser_Pop(ParsedCommand *out) {
  if(head==tail)return 0;
  memset(out,0,sizeof(*out));
  memcpy(out->text,lines[tail%QUEUE_SIZE],COMMAND_LINE_SIZE);tail++;
  out->type=classify(out->text,out);
  if(out->type==USB_COMMAND_UNKNOWN)stats.malformed++;
  return 1;
}
CommandStats CommandParser_Stats(void){return stats;}
