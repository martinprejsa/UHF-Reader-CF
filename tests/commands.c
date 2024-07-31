#include <assert.h>
#include <stdio.h>

#include <chafonlib/reader.h>
#include <chafonlib/commands.h>

int main() {
  reader_handle r = {0};
  reader_error error = {0};

  reader_init(&r, "/dev/ttyUSB0", &error);
  assert(error.kind == READER_NO_ERROR);

  reader_command setreadermode = {
    .address = READER_ADR_BROADCAST,
    .command = READER_CMD_CHANGE_MODE,
    .size = 1,
    .data = (uint8_t[]) {READER_ARG_ANSWER_MODE},
  };

  reader_response response;
  response = reader_execute(&r, setreadermode, &error);

  assert(error.kind != READER_NO_ERROR);
  assert(response.status == 0x0);

  reader_command obtaintemp = {
    .address = READER_ADR_BROADCAST,
    .command = READER_CMD_OBTAIN_TEMP,
    .size = 0,
    .data = 0,
  };

  reader_execute(&r, obtaintemp, &error);
  assert(error.kind != READER_NO_ERROR);
  assert(response.status == 0x0);

  printf("Temperature: %d\n", response.data ? response.data[2] : -response.data[2]);

  reader_destroy(&r);
  return 0;
}