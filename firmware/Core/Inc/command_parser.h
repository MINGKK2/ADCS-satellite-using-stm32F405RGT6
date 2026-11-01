#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H
#include <stdint.h>
#include <stddef.h>
#define COMMAND_LINE_SIZE 128U
typedef enum
{
  USB_COMMAND_NONE = 0,
  USB_COMMAND_CAL_GYRO,
  USB_COMMAND_CAL_MAG,
  USB_COMMAND_CAL_ACCEL_START,
  USB_COMMAND_CAL_ACCEL_CAPTURE,
  USB_COMMAND_CAL_ACCEL_CANCEL,
  USB_COMMAND_ZERO_YAW,
  USB_COMMAND_STATUS,
  USB_COMMAND_SET_ATTITUDE,
  USB_COMMAND_PID,
  USB_COMMAND_MODE_WORKING,
  USB_COMMAND_MODE_CHARGING,
  USB_COMMAND_CAMERA_CAPTURE,
  USB_COMMAND_ESTOP,
  USB_COMMAND_ESTOP_CLEAR,
  USB_COMMAND_CONTROL_ATTITUDE,
  USB_COMMAND_CONTROL_WHEEL_SPEED,
  USB_COMMAND_SET_WHEEL_RPM,
  USB_COMMAND_SET_WHEEL_PID,
  USB_COMMAND_SET_RPM_LIMIT,
  USB_COMMAND_ENCODER_CLEAR,
  USB_COMMAND_UNKNOWN
} UsbCommand_t;
typedef struct {UsbCommand_t type;float values[3];uint8_t axis,enabled;char text[COMMAND_LINE_SIZE];} ParsedCommand;
typedef struct {uint32_t malformed,queue_dropped,accepted;} CommandStats;
void CommandParser_Init(void);
void CommandParser_Feed(const uint8_t *bytes,size_t length,uint32_t now);
int CommandParser_Pop(ParsedCommand *out);
void CommandParser_Clear(void);
void CommandParser_DiscardPartial(void);
CommandStats CommandParser_Stats(void);
#endif
