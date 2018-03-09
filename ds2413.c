#include "FreeRTOS.h"
#include "task.h"
#include "math.h"

#include "ds2413.h"

#define DS2413_ONEWIRE_PIN  (8)

#define DS2413_FAMILY_ID    0x3A
#define DS2413_READ_ROM     0x33
#define DS2413_MATCH_ROM    0x55
#define DS2413_SEARCH_ROM   0xF0
#define DS2413_SKIP_ROM     0xCC

#define DS2413_RESUME       	   0xA5
#define DS2413_OVERDRIVE_SKIP_ROM  0x3C
#define DS2413_OVERDRIVE_MATCH_ROM 0x69


#define DS2413_ACCESS_READ  0xF5
#define DS2413_ACCESS_WRITE 0x5A
#define DS2413_ACK_SUCCESS  0xAA
#define DS2413_ACK_ERROR 0xFF

#define os_sleep_ms(x) vTaskDelay(((x) + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS)


#ifdef DS18B20_DEBUG
#define debug(fmt, ...) printf("%s" fmt "\n", "DS18B20: ", ## __VA_ARGS__);
#else
#define debug(fmt, ...)
#endif

bool ds18b20_pio_access_write(int pin, ds2413_addr_t addr,uint8_t pioval) {

    uint8_t ack = 0;

    pioval |= 0xFC;

    if (!onewire_reset(pin)) {
        return false;
    }
    if (addr == DS2413_ANY) {
        onewire_skip_rom(pin);
    } else {
        onewire_select(pin, addr);
    }
    onewire_write(pin,DS2413_ACCESS_WRITE);
    onewire_write(pin,(pioval));
    onewire_write(pin,(~pioval));
    ack =onewire_read(pin);

  if (ack == DS2413_ACK_SUCCESS)
  {
    onewire_read(pin);                       /* Read the status byte      */
  }
  onewire_reset(pin);
    
return (ack == DS2413_ACK_SUCCESS ? true : false);
} 


bool ds18b20_pio_access_read(int pin, ds2413_addr_t addr, uint8_t *buffer) {
    uint8_t crc;
    uint8_t expected_crc;

    if (!onewire_reset(pin)) {
        return false;
    }
    if (addr == DS2413_ANY) {
        onewire_skip_rom(pin);
    } else {
        onewire_select(pin, addr);
    }
    onewire_write(pin, DS2413_ACCESS_READ);

    for (int i = 0; i < 8; i++) {
        buffer[i] = onewire_read(pin);
    }
    crc = onewire_read(pin);

    expected_crc = onewire_crc8(buffer, 8);
    if (crc != expected_crc) {
        debug("CRC check failed reading scratchpad: %02x %02x %02x %02x %02x %02x %02x %02x : %02x (expected %02x)\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], crc, expected_crc);
        return false;
    }

    return true;
}

uint8_t ds2413_read_pin(int pin, ds2413_addr_t addr) {
    uint8_t pio_out_data_bit[8];
    
    if (!ds18b20_pio_access_read(pin, addr, pio_out_data_bit)) {
        return NAN;
    }
    
    //bool ok = false;
    //ok = (!pio_out_data_bit & 0x0F) == (pio_out_data_bit >> 4); /* Compare nibbles            */
    //pio_out_data_bit &= 0x0F; 				  /* Clear inverted values      */

    uint8_t Pio_State=pio_out_data_bit[8];

return Pio_State;

}

int ds2413_scan_devices(int pin, ds2413_addr_t *addr_list, int addr_count) {
    onewire_search_t search;
    onewire_addr_t addr;
    int found = 0;

    onewire_search_start(&search);
    while ((addr = onewire_search_next(&search, pin)) != ONEWIRE_NONE) {
        uint8_t family_id = (uint8_t)addr;
        if (family_id == DS2413_FAMILY_ID) {
            if (found < addr_count) {
                addr_list[found] = addr;
            }
            found++;
        }
    }
    return found;
}




