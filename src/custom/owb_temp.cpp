#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#define MAX_DEVICES 2

static DS18B20_Info * devices[MAX_DEVICES] = {0};
OneWireBus * owb;
static int num_devices = 0;

void owb_temp_init(gpio_num_t pin);
void owb_temp_read_all();
int owb_temp_get_results(float * results[], int num);

void owb_temp_init(gpio_num_t pin)
{
    // Create a 1-Wire bus, using the RMT timeslot driver
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, pin, RMT_CHANNEL_6, RMT_CHANNEL_7);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    // Find all connected devices
    printf("Find devices:\n");
    OneWireBus_ROMCode device_rom_codes[MAX_DEVICES];
    OneWireBus_SearchState search_state;
    bool found = false;
    num_devices = 0;
    owb_search_first(owb, &search_state, &found);
    return;
    while (found)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        printf("  %d : %s\n", num_devices, rom_code_s);
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    printf("Found %d devices\n", num_devices);

    // Create DS18B20 devices on the 1-Wire bus
    for (int i = 0; i < num_devices; ++i)
    {
        DS18B20_Info * ds18b20_info = ds18b20_malloc();  // heap allocation
        devices[i] = ds18b20_info;

        ds18b20_init(ds18b20_info, owb, device_rom_codes[i]); // associate with bus and device
        ds18b20_use_crc(ds18b20_info, true);           // enable CRC check on all reads
        ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION_12_BIT);
    }
}

void owb_temp_read_all()
{
    //ds18b20_convert_all(owb);
}

int owb_temp_get_results(float * results[], int num)
{
    int error = 0;

    if (num > num_devices) return -1;

    // In this application all devices use the same resolution,
    // so use the first device to determine the delay
    //ds18b20_wait_for_conversion(devices[0]);

    for (int i = 0; i < num; ++i)
    {
        //error |= ds18b20_read_temp(devices[i], results[i]);
    }
    return error;
}