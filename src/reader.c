#include "chafonlib/reader.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define CRC_INIT 0xFFFF

static char const *const reader_error_strings[] = {
    "READER_NO_ERROR", "READER_DEVICE_CONFIGURATION_ERROR",
    "READER_DEVICE_COMMUNICATION_ERROR", "READER_INVALID_PARAMETER"};

char const *reader_error_to_string(reader_error r) {
  char const *error = NULL;
  if (r.kind < READER_ERROR_SIZE) {
    error = reader_error_strings[r.kind];
  }
  return error;
}

void reader_init(reader_handle *reader, char const *const d_path, reader_error * error) {
  int device = open(d_path, O_RDWR);
  if (!device) {
    if (error) {
      error->kind = READER_DEVICE_CONFIGURATION_ERROR;
      error->message = strerror(errno);
    }
    return;
  }

  struct termios tty;
  if (tcgetattr(device, &tty) != 0) {
    if (error) {
      error->kind = READER_DEVICE_CONFIGURATION_ERROR;
      error->message = strerror(errno);
    }
    return;
  }

  struct termios {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t c_line;
    cc_t c_cc[NCCS];
  };

  tty.c_cflag &= ~PARENB; // no parity-bit
  tty.c_cflag &= ~CSTOPB; // single bit stop

  tty.c_cflag &= ~CSIZE; // clear bit-size
  tty.c_cflag |= CS8;    // 8 bit per byte

  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
  tty.c_cflag |=
      CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON; // disable canonical mode

  tty.c_lflag &= ~ECHO;   // Disable echo
  tty.c_lflag &= ~ECHOE;  // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo

  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g.
                         // newline chars)
  tty.c_oflag &=
      ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10;
  tty.c_cc[VMIN] = 1;

  cfsetispeed(&tty, B57600);
  cfsetospeed(&tty, B57600);

  if (tcsetattr(device, TCSANOW, &tty) != 0) {
    if (error) {
      error->kind = READER_DEVICE_CONFIGURATION_ERROR;
      error->message = strerror(errno);
    }
    return;
  }

  reader->device = device;
  reader->mode = READER_UNKNOWN_MODE;
}

void reader_destroy(reader_handle *r) {
  close(r->device);
}

uint16_t crc16_mcrf4xx(uint16_t crc, uint8_t *data, size_t len) {
  if (!data || len < 0)
    return crc;

  while (len--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0x8408;
      else
        crc = (crc >> 1);
    }
  }
  return crc;
}

void write_frame(reader_handle *const reader, reader_command const c, reader_error * error) {
  
  assert(reader);

  if (c.size > 251) {
    if (error) {
      error->kind = READER_INVALID_PARAMETER;
      error->message = "command data length too large";
    }
    return;
  }

  size_t len = c.size + 5;
  uint8_t *buff = (uint8_t *) calloc(sizeof(char), len);

  buff[0] = len - 1;
  // adr + cmd + crc = 4bytes
  buff[1] = c.address;
  buff[2] = c.command;

  memcpy(buff + sizeof(char) * 3, c.data, c.size);

  uint16_t crc = crc16_mcrf4xx(CRC_INIT, buff, len - 2);

  buff[len - 2] = (uint8_t)crc;        // lsb
  buff[len - 1] = (uint8_t)(crc >> 8); // msb

  int w = write(reader->device, buff, len);
  free(buff);

  if (w == -1) {
    if (error) {
      error->kind = READER_DEVICE_COMMUNICATION_ERROR;
      error->message = "unexpected EOF";
    }
  }

  if (error) {
    error->kind = READER_NO_ERROR;
    error->message = "ok";
  }
}

reader_response read_frame(reader_handle *const reader, reader_error * error) {

  assert(reader);

  reader_response response = {0};
  uint8_t buffer[255] = {0};
  int length = 0;
  int r = read(reader->device, &buffer, sizeof(uint8_t));

  length = buffer[0];
  if (r <= 0 || length < 4) {
    if (error) {
      error->kind = READER_DEVICE_COMMUNICATION_ERROR;
      error->message = "unexpected packet size recieved";
    }
    return response;
  }

  r = read(reader->device, buffer + sizeof(uint8_t), length);
  if (r != length) {
    if (error) {
      error->kind = READER_DEVICE_COMMUNICATION_ERROR;
      error->message = "unexpected packet size recieved";
    }
    return response;
  }

  uint16_t r_crc = (buffer[length - 2] | (buffer[length - 1] << 8));
  uint16_t a_crc = crc16_mcrf4xx(CRC_INIT, buffer, length - 2);

  if (r_crc != a_crc) {
    if (error) {
      error->kind = READER_DEVICE_COMMUNICATION_ERROR;
      error->message = "packet with invalid crc recieved";
    }
    return response;
  }

  response.command = buffer[1];
  response.address = buffer[2];
  response.status = buffer[3];
  response.size = length - 5;
  response.data = calloc(sizeof(uint8_t), length - 5);

  memcpy(response.data, buffer + 4 * sizeof(uint8_t), response.size);

  if (error) {
    error->kind = READER_NO_ERROR;
    error->message = "ok";
  }

  return response;
}

reader_response reader_execute(reader_handle *const reader,
                            reader_command const command, reader_error * error) {
  assert(reader);
  assert(reader->mode == READER_COMMAND_MODE);

  reader_error err = {0};
  write_frame(reader, command, &err);
  if (err.kind) {
    if (error) {
      error->kind = err.kind;
      error->message = err.message;
    }
    return (reader_response) {0};
  }

  reader_response response = read_frame(reader, &err);
  if (err.kind) {
    if (error) {
      error->kind = err.kind;
      error->message = err.message;
    }
    return (reader_response) {0};
  }

  if (response.command != command.command) {
    if (error) {
      error->kind = READER_DEVICE_COMMUNICATION_ERROR;
      error->message = "response command does not match executed command";
    }
    return (reader_response) {0};
  }

  if (error) {
    error->kind = READER_NO_ERROR;
    error->message = "ok";
  }

  return (reader_response) {0};
}