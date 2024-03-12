#include "hasplib.h"

#if defined(HASP_USE_CUSTOM) && true

#include "mqtt_client.h"

#include <Wire.h>
#include <esp_adc_cal.h>
#include "hasp_debug.h"
#include "hasp_gui.h"
#include "vl53l0x-arduino/VL53L0X.h"
#include "SparkFun_VEML7700_Arduino_Library/src/SparkFun_VEML7700_Arduino_Library.h"
#include "TCA6416A/src/TCA6416A.h"

#define PIN_SDA 19
#define PIN_SCL 20
#define PIN_SNS_TANKS_OUT 17
#define PIN_SNS_TANKS_IN 18

#define PINA_SNS_LP 12

#define PINX_PUMP_OUT 0
#define PINX_PUMP_IN 1
#define PINX_HEATER_LP 2
#define PINX_HEATER_LP_FLT 3
#define PINX_HEATER_AC 4
#define PINX_SNS_LP_PWR 5

#define HASP_CLOCKLBL_PG 0
#define HASP_CLOCKLBL_ID 1
#define HASP_PUMPBUT_PG 1
#define HASP_PUMPBUT_ID 10
#define HASP_HEATERLPBUT_PG 1
#define HASP_HEATERLPBUT_ID 11
#define HASP_HEATERACBUT_PG 1
#define HASP_HEATERACBUT_ID 12
#define HASP_AGSBUT_PG 1
#define HASP_AGSBUT_ID 15
#define HASP_GENMODAL_PG 1
#define HASP_GENMODAL_ID 200
#define HASP_FRESHBAR_PG 1
#define HASP_FRESHBAR_ID 22
#define HASP_GREYBAR_PG 1
#define HASP_GREYBAR_ID 23
#define HASP_BLACKBAR_PG 1
#define HASP_BLACKBAR_ID 24
#define HASP_LPBAR_PG 1
#define HASP_LPBAR_ID 21
#define HASP_BATTBAR_PG 1
#define HASP_BATTBAR_ID 20
#define HASP_FRESHLBL_PG 1
#define HASP_FRESHLBL_ID 52
#define HASP_GREYLBL_PG 1
#define HASP_GREYLBL_ID 53
#define HASP_BLACKLBL_PG 1
#define HASP_BLACKLBL_ID 54
#define HASP_LPLBL_PG 1
#define HASP_LPLBL_ID 51
#define HASP_BATTLBL_PG 1
#define HASP_BATTLBL_ID 50

#define SL_RETRY 3
#define SL_TANKS 3

extern int mqttQos;
extern esp_mqtt_client_handle_t mqttClient;
extern bool last_mqtt_state;
extern bool current_mqtt_state;

void sl_init(int pin_in, int pin_out);
void sl_readtank(int sensor);
int sl_result();
extern int sl_power[3];

static VL53L0X range_sensor;
static VEML7700 als_sensor;

// hack to allow access to MQTT variables (for subscription)
typedef struct
{
    mqtt_event_callback_t event_handle;
} mqtt_config_storage_t;
struct esp_mqtt_client
{
    void* transport_list;
    void* transport;
    mqtt_config_storage_t* config;
};

String bms_mqtt_prefix = String("fancont/fancont_inv/");

TCA6416A ex;

StaticJsonDocument<256> levels;
StaticJsonDocument<256> control;
StaticJsonDocument<256> status;

esp_adc_cal_characteristics_t adc2_chars;

bool mqtt_started = false;
mqtt_event_callback_t hasp_mqtt_event_handler;

bool pump_update                 = false;
unsigned long pump_update_millis = 0;

int batt_cap_pct[4]              = {0};
unsigned long batt_cap_millis[4] = {0};

int tank_state                   = 0;
unsigned long tank_update_millis = 0;

void lp_loop()
{
    uint32_t mv;
    float r;
    int pct;
    static bool first = true;
    static float mv_filt;
    static int state                    = 0;
    static unsigned long periodicmillis = 0, samplemillis = 0;

    if(millis() - periodicmillis > 2000) {
        periodicmillis = millis() - 1000;
    }

    if(state == 0) {
        if(millis() - periodicmillis < 1000) return;
        ex.pin_write(PINX_SNS_LP_PWR, HIGH);
        samplemillis = millis();
        state++;
        return;
    }

    if(millis() - samplemillis < 50) return;

    state = 0;
    periodicmillis += 1000;
    mv = esp_adc_cal_raw_to_voltage(analogRead(PINA_SNS_LP), &adc2_chars);
    if(first) {
        first   = false;
        mv_filt = mv;
    } else {
        mv_filt = mv_filt + (mv - mv_filt) * 0.05;
    }
    ex.pin_write(PINX_SNS_LP_PWR, LOW);

    // get resistance from voltage divider with 56.8mA current source (LM317 current source with 22R) and 15k/15k
    // divider
    r = mv_filt * (0.001 / ((15.0 / (15 + 15) * (1.25 / 22))));

    // sensor is 0-90R with 90R being 100% full
    pct = 100 * r / 90;
    if(pct > 115) {
        levels[F("lp")] = -1;
    } else if(pct > 100) {
        levels[F("lp")] = 100;
    } else {
        levels[F("lp")] = pct;
    }
}

void handle_pump(bool state)
{
    if((state && control[F("pump")] == "OFF") || (!state && control[F("pump")] == "ON")) {
        ex.pin_write(PINX_PUMP_OUT, HIGH);
        delay(200);
        ex.pin_write(PINX_PUMP_OUT, LOW);
        if(control[F("pump")] == "ON") {
            control[F("pump")] = "OFF";
        } else {
            control[F("pump")] = "ON";
        }
        pump_update        = true;
        pump_update_millis = millis();
    }
}

void handle_heaterlp(bool state)
{
    if(state) {
        control[F("heater_lp")] = "ON";
        ex.pin_write(PINX_HEATER_LP, HIGH);
    } else {
        control[F("heater_lp")] = "OFF";
        ex.pin_write(PINX_HEATER_LP, LOW);
    }
}

void handle_heaterac(bool state)
{
    if(state) {
        control[F("heater_ac")] = "ON";
        ex.pin_write(PINX_HEATER_AC, HIGH);
    } else {
        control[F("heater_ac")] = "OFF";
        ex.pin_write(PINX_HEATER_AC, LOW);
    }
}

// this is for MQTT subscription
static void mqtt_event_handler(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    int msg_id;
    static StaticJsonDocument<512> doc;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch(event_id) {
        case MQTT_EVENT_CONNECTED:
            LOG_VERBOSE(TAG_CUSTOM, "MQTT connected");
            esp_mqtt_client_subscribe(mqttClient, (bms_mqtt_prefix + "#").c_str(), mqttQos);
            break; // fall through to original handler
        case MQTT_EVENT_DATA:
            String topic  = String(event->topic).substring(0, event->topic_len);
            String msg    = String(event->data).substring(0, event->data_len);
            String search = bms_mqtt_prefix + "bms";
            // LOG_VERBOSE(TAG_CUSTOM, "MQTT incoming: %s - %s\n", topic.c_str(), msg.c_str());
            if(!topic.startsWith(search)) {
                if(topic.startsWith(bms_mqtt_prefix)) return; // ignore all incoming with our prefix
                break;                                        // not our prefix so fall through to original handler
            }
            unsigned int bms = topic.substring(search.length()).toInt();
            if(bms > 3) return;
            deserializeJson(doc, msg);
            batt_cap_pct[bms]    = doc["cap_remaining_pct"];
            batt_cap_millis[bms] = millis();
            return;
    }
    hasp_mqtt_event_handler(event);
}

void custom_toggle_event_handler(lv_obj_t* obj, lv_event_t event)
{
    uint8_t pg, id;
    static bool agsbut_long = false;

    hasp_find_id_from_obj(obj, &pg, &id);
    if(event == LV_EVENT_VALUE_CHANGED) {
        if(pg == HASP_PUMPBUT_PG && id == HASP_PUMPBUT_ID) {
            handle_pump(lv_btn_get_state(obj) & LV_STATE_CHECKED);
        } else if(pg == HASP_HEATERLPBUT_PG && id == HASP_HEATERLPBUT_ID) {
            handle_heaterlp(lv_btn_get_state(obj) & LV_STATE_CHECKED);
        } else if(pg == HASP_HEATERACBUT_PG && id == HASP_HEATERACBUT_ID) {
            handle_heaterac(lv_btn_get_state(obj) & LV_STATE_CHECKED);
        }
    } else if(event == LV_EVENT_LONG_PRESSED && pg == HASP_AGSBUT_PG && id == HASP_AGSBUT_ID) {
        agsbut_long = true;
        obj         = hasp_find_obj_from_page_id(HASP_GENMODAL_PG, HASP_GENMODAL_ID);
        if(!obj) return;
        lv_obj_set_hidden(obj, false);
    } else if(event == LV_EVENT_RELEASED && agsbut_long && pg == HASP_AGSBUT_PG && id == HASP_AGSBUT_ID) {
        agsbut_long = false;
        if(lv_btn_get_state(obj) & LV_STATE_CHECKED) {
            lv_obj_clear_state(obj, LV_STATE_CHECKED);
        } else {
            lv_obj_add_state(obj, LV_STATE_CHECKED);
        }
        return; // don't run hasp handler
    }
    toggle_event_handler(obj, event);
}

void custom_setup()
{
    // Initialization code here
    randomSeed(millis());

    Wire.begin(PIN_SDA, PIN_SCL);

    ex.begin(0);

    ex.pin_write(PINX_PUMP_OUT, LOW);
    ex.pin_mode(PINX_PUMP_OUT, OUTPUT);
    ex.pin_mode(PINX_PUMP_IN, INPUT_PULLDOWN);
    ex.pin_write(PINX_HEATER_LP, LOW);
    ex.pin_mode(PINX_HEATER_LP, OUTPUT);
    ex.pin_mode(PINX_HEATER_LP_FLT, INPUT_PULLDOWN);
    ex.pin_write(PINX_HEATER_AC, LOW);
    ex.pin_mode(PINX_HEATER_AC, OUTPUT);
    ex.pin_write(PINX_SNS_LP_PWR, LOW);
    ex.pin_mode(PINX_SNS_LP_PWR, OUTPUT);

    range_sensor.setTimeout(500);
    range_sensor.init(true); // 2.8V I/O mode
    // long range
    range_sensor.setSignalRateLimit(1.2);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    range_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    range_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    range_sensor.startContinuous(50);

    als_sensor.begin();

    sl_init(PIN_SNS_TANKS_IN, PIN_SNS_TANKS_OUT);

    analogReadResolution(12);
//    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_12, ADC_WIDTH_12Bit, 1100, &adc2_chars);
    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_11, ADC_WIDTH_12Bit, 1100, &adc2_chars);

    control[F("heater_ac")] = "OFF";
    control[F("heater_lp")] = "OFF";

    lv_obj_t* obj;
    obj = hasp_find_obj_from_page_id(HASP_PUMPBUT_PG, HASP_PUMPBUT_ID);
    if(!obj) return;
    lv_obj_set_event_cb(obj, custom_toggle_event_handler);
    obj = hasp_find_obj_from_page_id(HASP_HEATERLPBUT_PG, HASP_HEATERLPBUT_ID);
    if(!obj) return;
    lv_obj_set_event_cb(obj, custom_toggle_event_handler);
    obj = hasp_find_obj_from_page_id(HASP_HEATERACBUT_PG, HASP_HEATERACBUT_ID);
    if(!obj) return;
    lv_obj_set_event_cb(obj, custom_toggle_event_handler);
    obj = hasp_find_obj_from_page_id(HASP_AGSBUT_PG, HASP_AGSBUT_ID);
    if(!obj) return;
    lv_obj_set_event_cb(obj, custom_toggle_event_handler);

    tank_update_millis = millis();

    custom_every_second();
    custom_every_5seconds();

    LOG_VERBOSE(TAG_CUSTOM, "Setup complete");
}

void custom_loop()
{
    static unsigned long tofmillis = 0;
    lp_loop();

    if (millis() - tofmillis > 250) {
        status[F("tof")] = range_sensor.readRangeContinuousMillimeters();
        tofmillis = millis();

        char payload[256];
        serializeJson(status, payload, sizeof(payload));
        dispatch_state_subtopic("status", payload);
    }

    if(tank_state < SL_TANKS << 1) {
        static int tries = SL_RETRY;
        if(!(tank_state & 1)) { // on even states do read
            if(millis() - tank_update_millis > 2000) {
                tank_update_millis = millis();
            } else if(millis() - tank_update_millis > 450) {
                tank_update_millis += 450;
                sl_readtank(tank_state >> 1);
                tank_state++;
            }
        } else { // on odd states get result
            if(millis() - tank_update_millis > 50) {
                tank_update_millis += 50;
                int result = sl_result();
                if(result >= 0 || --tries == 0) {
                    switch(tank_state) {
                        case 1:
                            levels[F("fresh")] = result;
                            break;
                        case 3:
                            levels[F("grey")] = result;
                            break;
                        case 5:
                            levels[F("black")] = result;
                            break;
                    }
                    tank_state++; // if good result or no more retries go on to next sensor
                    tries = SL_RETRY;
                } else {
                    tank_state--; // go back to read state
                }
            }
        }
    }
    // this allow for subscription to MQTT messages
    if(!mqtt_started && mqttClient != nullptr) {
        hasp_mqtt_event_handler = mqttClient->config->event_handle;
        esp_mqtt_client_register_event(mqttClient, MQTT_EVENT_CONNECTED, mqtt_event_handler, mqttClient);
        esp_mqtt_client_register_event(mqttClient, MQTT_EVENT_DATA, mqtt_event_handler, mqttClient);
        mqtt_started = true;
        LOG_VERBOSE(TAG_CUSTOM, "MQTT registered for events");
    }
}

void update_bar(uint8_t pg, uint8_t id, int val)
{
    lv_obj_t* obj;
    obj = hasp_find_obj_from_page_id(pg, id);
    if(!obj) return;
    lv_bar_set_value(obj, val, LV_ANIM_ON);
}

void update_label(uint8_t pg, uint8_t id, int val)
{
    lv_obj_t* obj;
    char str[10];
    obj = hasp_find_obj_from_page_id(pg, id);
    if(!obj) return;
    if(val < 0) {
        lv_label_set_text(obj, "FLT");
        lv_obj_set_style_local_outline_opa(obj, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_COVER);
    } else {
        sprintf(str, "%d%%", val);
        lv_label_set_text(obj, str);
        lv_obj_set_style_local_outline_opa(obj, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    }
}

void custom_every_second()
{
    status[F("als")] = als_sensor.getLux();

    if(!pump_update || millis() - pump_update_millis > 5000) { // only update pump after 5s from last change
        control[F("pump")] = ex.pin_read(PINX_PUMP_IN) ? "ON" : "OFF";

        lv_obj_t* button = hasp_find_obj_from_page_id(HASP_PUMPBUT_PG, HASP_PUMPBUT_ID);
        if(!button) return;
        if(control[F("pump")] == "ON") {
            lv_obj_add_state(button, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(button, LV_STATE_CHECKED);
        }

        pump_update_millis = millis() + 5000; // set so that it doesn't roll over eventually
        pump_update        = false;
    }

    control[F("heater_lp_fault")] = ex.pin_read(PINX_HEATER_LP_FLT) ? "ON" : "OFF";
    lv_obj_t* obj                 = hasp_find_obj_from_page_id(HASP_HEATERLPBUT_PG, HASP_HEATERLPBUT_ID);
    if(!obj) return;
    if(control[F("heater_lp_fault")] == "ON") {
        lv_obj_set_style_local_outline_opa(obj, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_COVER);
    } else {
        lv_obj_set_style_local_outline_opa(obj, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    }

    char payload[256];
    serializeJson(control, payload, sizeof(payload));
    dispatch_state_subtopic("control", payload);
    // LOG_VERBOSE(TAG_CUSTOM, "%s\n", payload);

/*    obj = hasp_find_obj_from_page_id(HASP_CLOCKLBL_PG, HASP_CLOCKLBL_ID);
    char buffer[128];
    time_t rawtime;
    struct tm* timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%b %d %I:%M:%S%p %Z", timeinfo);
    lv_label_set_text(obj, buffer);*/
}

void custom_every_5seconds()
{
    if(tank_state == SL_TANKS << 1) { // only request a new tank update if we have completed the last one
        tank_state         = 0;
        tank_update_millis = millis();
    }

    int batt_cap_avg = 0;
    for(int i = 0; i < 4; i++) {
        if(millis() - batt_cap_millis[i] > 10000) { // 10s timeout for battery update
            batt_cap_avg = -1;
            break;
        }
        batt_cap_avg += batt_cap_pct[i];
    }

    if(batt_cap_avg != -1) batt_cap_avg >>= 2;

    update_bar(HASP_FRESHBAR_PG, HASP_FRESHBAR_ID, levels[F("fresh")]);
    update_bar(HASP_GREYBAR_PG, HASP_GREYBAR_ID, levels[F("grey")]);
    update_bar(HASP_BLACKBAR_PG, HASP_BLACKBAR_ID, levels[F("black")]);
    update_bar(HASP_LPBAR_PG, HASP_LPBAR_ID, levels[F("lp")]);
    update_bar(HASP_BATTBAR_PG, HASP_BATTBAR_ID, batt_cap_avg);

    update_label(HASP_FRESHLBL_PG, HASP_FRESHLBL_ID, levels[F("fresh")]);
    update_label(HASP_GREYLBL_PG, HASP_GREYLBL_ID, levels[F("grey")]);
    update_label(HASP_BLACKLBL_PG, HASP_BLACKLBL_ID, levels[F("black")]);
    update_label(HASP_LPLBL_PG, HASP_LPLBL_ID, levels[F("lp")]);
    update_label(HASP_BATTLBL_PG, HASP_BATTLBL_ID, batt_cap_avg);

    char payload[256];
    serializeJson(levels, payload, sizeof(payload));
    dispatch_state_subtopic("levels", payload);
    // LOG_VERBOSE(TAG_CUSTOM, "%s\n", payload);
}

bool custom_pin_in_use(uint8_t pin)
{
    switch(pin) {
        case PIN_SDA:
        case PIN_SCL:
        case PIN_SNS_TANKS_OUT:
        case PIN_SNS_TANKS_IN:
        case PINA_SNS_LP:
            return true;
        default:
            return false;
    }
}

void custom_get_sensors(JsonDocument& doc)
{
    JsonObject diag      = doc.createNestedObject(F("diag"));
    diag[F("fresh_pwr")] = sl_power[0];
    diag[F("grey_pwr")]  = sl_power[1];
    diag[F("black_pwr")] = sl_power[2];
}

void custom_topic_payload(const char* topic, const char* payload, uint8_t source)
{
    lv_obj_t* obj = nullptr;
    bool state    = !strcasecmp(payload, "ON");

    if(!state && !!strcasecmp(payload, "OFF")) {
        return;
    }

    if(!strcmp(topic, "pump")) {
        obj = hasp_find_obj_from_page_id(HASP_PUMPBUT_PG, HASP_PUMPBUT_ID);
        handle_pump(state);
    }

    if(!strcmp(topic, "heater_lp")) {
        obj = hasp_find_obj_from_page_id(HASP_HEATERLPBUT_PG, HASP_HEATERLPBUT_ID);
        handle_heaterlp(state);
    }

    if(!strcmp(topic, "heater_ac")) {
        obj = hasp_find_obj_from_page_id(HASP_HEATERACBUT_PG, HASP_HEATERACBUT_ID);
        handle_heaterac(state);
    }
    if(!obj) return;
    if(state) {
        lv_obj_add_state(obj, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(obj, LV_STATE_CHECKED);
    }
}

#endif // HASP_USE_CUSTOM