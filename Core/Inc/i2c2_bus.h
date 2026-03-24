#ifndef I2C2_BUS_H
#define I2C2_BUS_H

/** Serialize all I2C2 traffic (LCD + RGB + touch) — interleaved HAL transfers corrupt slaves (RGB stuck red). */
void I2c2Bus_Init(void);
void I2c2Bus_Lock(void);
void I2c2Bus_Unlock(void);

#endif
