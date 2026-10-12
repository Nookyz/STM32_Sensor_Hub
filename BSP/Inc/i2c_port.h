#ifndef I2C_PORT_H
#define I2C_PORT_H

#include <stdint.h>
#include "main.h"
#include "status.h"

typedef struct
{
  uint32_t errors;     /* Timeouts, bus errors, stuck busy */
  uint32_t nacks;      /* Device did not acknowledge (not counted by probe) */
  uint32_t recoveries; /* Bus recovery attempts */
} i2c_stats_t;

/** Bind the driver to a HAL handle. Call after MX_I2C1_Init(). */
status_t i2c_port_init(I2C_HandleTypeDef *hi2c);

/** Check whether a 7-bit address ACKs. NACK is a normal answer, not an error. */
status_t i2c_port_probe(uint8_t addr7);

status_t i2c_port_write(uint8_t addr7, const uint8_t *data, uint16_t len);
status_t i2c_port_read(uint8_t addr7, uint8_t *data, uint16_t len);

/** Register access: write/read with an 8-bit register address. */
status_t i2c_port_mem_write(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len);
status_t i2c_port_mem_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len);

/** Free a stuck bus (9 SCL clocks + STOP) and re-init the peripheral. */
status_t i2c_port_recover(void);

void i2c_port_get_stats(i2c_stats_t *out);

#endif /* I2C_PORT_H */