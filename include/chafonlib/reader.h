#ifndef READER_READER_H
#define READER_READER_H

#include <stdint.h>

typedef enum {
  READER_NO_ERROR = 0,
  READER_DEVICE_CONFIGURATION_ERROR,
  READER_DEVICE_COMMUNICATION_ERROR,
  READER_INVALID_PARAMETER,
  READER_ERROR_SIZE
} reader_error_kind;

typedef enum {
  READER_UNKNOWN_MODE = 0,
  READER_COMMAND_MODE,
  READER_RTI_MODE,
  READER_RTIT_MODE,
  READER_MODE_SIZE,
} reader_mode;

typedef struct {
  char* message;
  reader_error_kind kind;
} reader_error;

typedef struct {
  uint8_t address;
  uint8_t command;
  uint8_t size; // data size
  uint8_t *data;
} reader_command;

typedef struct {
  uint8_t address;
  uint8_t command;
  uint8_t status;
  uint8_t size; // data size
  uint8_t *data;
} reader_response;

typedef struct {
  int device;
  reader_mode mode;
} reader_handle;


/**
 * @brief Used to retrieve message from an error.
 * 
 * @param error the error
 * @return char const* message
 */
char const * reader_error_to_string(reader_error error);

/**
 * @brief Initializes reader handle.
 * 
 * @param reader the reader handle
 * @param device path to the reader device, usually: /dev/ttyUSB0
 * @param error  error returned, nullable
 */
void reader_init(reader_handle *reader, char const * const device, reader_error * error);

/**
 * @brief Destroys the reader
 * 
 * @param reader the reader handle
 */
void reader_destroy(reader_handle *reader);

/**
 * @brief Executes a reader command
 *        and returns it's response.
 * 
 * @param reader reader handle
 * @param command command to execute
 * @param error error returned, nullable
 * @return reader_response
 */
 reader_response reader_execute(reader_handle * const reader, 
                              reader_command const command, reader_error * error);

#endif