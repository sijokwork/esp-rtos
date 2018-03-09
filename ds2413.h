#ifndef DRIVER_DS2413_H_
#define DRIVER_DS2413_H_

#include "onewire/onewire.h"

#ifdef	__cplusplus
extern "C" {
#endif

/** @file ds2413.h
 *
 *  Communicate with the DS2413 family of one-wire  ICs.
 *
 */

typedef onewire_addr_t ds2413_addr_t;

/** An address value which can be used to indicate "any device on the bus" */
#define DS2413_ANY ONEWIRE_NONE

/** Find the addresses of all DS2413 devices on the bus.
 *
 *  Scans the bus for all devices and places their addresses in the supplied
 *  array.  If there are more than `addr_count` devices on the bus, only the
 *  first `addr_count` are recorded.
 *
 *  @param pin         The GPIO pin connected to the DS2413 bus
 *  @param addr_list   A pointer to an array of ds2413_addr_t values.  This
 *                     will be populated with the addresses of the found
 *                     devices.
 *  @param addr_count  Number of slots in the `addr_list` array.  At most this
 *                     many addresses will be returned.
 *
 *  @returns The number of devices found.  Note that this may be less than,
 *  equal to, or more than `addr_count`, depending on how many DS2413 devices
 *  are attached to the bus.
 */
int ds2413_scan_devices(int pin, ds2413_addr_t *addr_list, int addr_count);

/**
*
*/

bool ds18b20_pio_access_read(int pin, ds2413_addr_t addr, uint8_t *buffer);

/*
* Read the DS2413 PIN 
*  
*  @param pin           The GPIO pin connected to the DS2413 bus
*  @param addr_list   A pointer to an array of ds2413_addr_t values.  This
*                     will be populated with the addresses of the found
*                     devices.
*  @returns    the 8 bit data of pio pin stat from msb 
*/

uint8_t ds2413_read_pin(int pin, ds2413_addr_t addr);

/*
*
*/

bool ds18b20_pio_access_write(int pin, ds2413_addr_t addr,uint8_t pioval);

#ifdef	__cplusplus
}
#endif

#endif  /* DRIVER_DS2413_H_ */
