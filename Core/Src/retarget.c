#include "main.h"
#include "rs485.h"
#include <unistd.h>

int _write(int file, char *ptr, int len)
{
  (void)file;

  if (ptr == NULL || len <= 0)
  {
    return 0;
  }

  (void)RS485_Transmit((const uint8_t *)ptr, (uint16_t)len, 100U);

  return len;
}
