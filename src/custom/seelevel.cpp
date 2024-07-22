#include "hasplib.h"
#include "driver/rmt.h"

#define SL_BYTES 12
#define SL_MINPOWER (256 * 0.15) // limit minimum power to 15%

#define RMT_CHANNEL RMT_CHANNEL_4 // other channels used by LCD?

int sl_power[3] = {0};

static volatile int sl_sensor = 0;
static hw_timer_t* sl_timer = nullptr;
static int sl_pin_out;
static unsigned long last_read = 0, last_off = 0;
static RingbufHandle_t rmt_rb = nullptr;

void ICACHE_RAM_ATTR sl_timerint()
{
    static int state = 0;
    static int sensor_idx = 0;

    switch(state) {
        case 0:
            digitalWrite(sl_pin_out, HIGH);
            timerAlarmWrite(sl_timer, 2450, true); // set timer 2450 us
            sensor_idx = 0;
            state++;
            break;
        case 1:
            digitalWrite(sl_pin_out, LOW);
            timerAlarmWrite(sl_timer, 85, true); // set timer 85 us
            state++;
            break;
        case 2:
            digitalWrite(sl_pin_out, HIGH);
            timerAlarmWrite(sl_timer, 290, true); // set timer 290 us
            if(sensor_idx < sl_sensor) {
                state = 1;
            } else {
                state++;
            }
            sensor_idx++;
            break;
        case 3:
            timerStop(sl_timer);
            timerAlarmWrite(sl_timer, 0, true);
            state = 0;
            rmt_rx_start(RMT_CHANNEL, true); // clear out RMT buffer
            break;
    }
}

void sl_readtank(int sensor)
{
    sl_sensor = sensor;

    while(millis() - last_off < 400) delay(1); // force sensor to power off for at least 400 ms from last read

    timerStart(sl_timer); // this starts the power-up and read sequence
    last_read = millis();
}

int sl_result()
{
    char sl_data[12];
    char* segments = &sl_data[2];

    // convert the received bitstream into bytes
    int i = 0, bit = 0, err = 0;
    size_t size;
    rmt_item32_t *items = nullptr;
    do {
        items = (rmt_item32_t*) xRingbufferReceive(rmt_rb, &size, pdMS_TO_TICKS(20)); // get available items with 20 ms timeout
        if (items == nullptr) { // timeout
            err = -1;
            break;
        }
        for (int j = 0; i < SL_BYTES && j < size >> 2; j ++) { // size is in bytes, items are 4 bytes
            //LOG_VERBOSE(TAG_CUSTOM, "duration0: %d level0:%d duration1: %d level1: %d\n", items->duration0, items->level0, items->duration1, items->level1);
            sl_data[i] <<= 1;
            if (items->duration0 > 30) sl_data[i] |= 1; // 30 us threshold
//            if (items->duration0 > 40) sl_data[i] |= 1; // 40 us threshold
            vRingbufferReturnItem(rmt_rb, (void*)items); // return space to ringbuffer
            items ++;
            bit ++;
            if(bit == 8) {
                bit = 0;
                i ++;
            }
        }
    } while (i < SL_BYTES);

    rmt_rx_stop(RMT_CHANNEL);
    digitalWrite(sl_pin_out, LOW); // remove sensor power
    last_off = millis();

    LOG_VERBOSE(TAG_CUSTOM, "\nseelevel bytes: %d data: %d %d %d %d %d %d %d %d %d %d %d %d\n", i, sl_data[0],
                sl_data[1], sl_data[2], sl_data[3], sl_data[4], sl_data[5], sl_data[6], sl_data[7], sl_data[8],
                sl_data[9], sl_data[10], sl_data[11]);

    if (err) return err;

    int chksum = 0;
    for(i = 2; i < SL_BYTES; i++) {
        chksum += sl_data[i];
    }
    if(chksum != ((int)(sl_data[0] & 0xF) << 8 | sl_data[1])) return -2; // checksum error

    int power = sl_power[sl_sensor];
    int segment_bot = 0, sum = 0;
    bool stop_power = false;
    for(i = SL_BYTES - 3; i >= 0; i--) { // go from bottom of tank to top
        if(segment_bot == 0) { // if we haven't found the lowest segment yet, look for the first usable segment
            if(segments[i] == 0 || segments[i] == 255) continue;
            segment_bot = i;
        }
        sum += segments[i]; // add up all segment levels to use later in level calc
        if(stop_power) continue;
        if(i == segment_bot) {
            // on bottom segment use level as power if it's greater than the stored
            if(sum > power) power = sum;
        } else if(i != 0 && segments[i - 1] >= SL_MINPOWER && segments[i] >= power * 0.7) {
            // if next segment is not zero, use this segment for power calc
            power = (float)sum / (segment_bot - i + 1);
        } else {
            stop_power = true; // don't use subsequent segments for power calc
        }
    }

    if(power < SL_MINPOWER) power = SL_MINPOWER; // limit min power
    int max = power * (segment_bot + 1);       // max is the maximum value for all segments considering the power
    int level = 100.0 * sum / max;
    if(level > 100) level = 100;

    sl_power[sl_sensor] = power; // save calculated power to use later

    LOG_VERBOSE(TAG_CUSTOM, "seelevel #%d level %d%% max %d sum %d power %d\n", sl_sensor, level, max, sum, power);
    return level;
}

void sl_init(int pin_in, int pin_out)
{
    pinMode(pin_out, OUTPUT);
    pinMode(pin_in, INPUT); // no pull
    sl_pin_out = pin_out;

    rmt_config_t rmt_rx_config = {
        .rmt_mode = RMT_MODE_RX,
        .channel = RMT_CHANNEL,
        .gpio_num = (gpio_num_t)pin_in,
        .clk_div = 80,
        .mem_block_num = 1,
        .flags = 0,
        .rx_config = {
            .idle_threshold = 500,
            .filter_ticks_thresh = 15,
            .filter_en = true,
        }
    };
    rmt_config(&rmt_rx_config);
    rmt_driver_install(rmt_rx_config.channel, 5000, 0);
    rmt_get_ringbuf_handle(rmt_rx_config.channel, &rmt_rb);

    sl_timer = timerBegin(1, 80, true); // begin Timer1 with 1 us period (80MHz/80)
    timerStop(sl_timer);
    timerAttachInterrupt(sl_timer, &sl_timerint, true); // attach the ISR to Timer1
    timerAlarmWrite(sl_timer, 0, true);
    timerAlarmEnable(sl_timer);
}