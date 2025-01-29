#include "hasplib.h"

#if defined(HASP_USE_CUSTOM) && true

#include <math.h>
#include "mqtt_client.h"

#include <Wire.h>
#include <I2CScanner.h>
#include <esp_adc_cal.h>
#include "hasp_debug.h"
#include "hasp_gui.h"
#include <vl53l1x_class.h>
#include <SparkFun_VEML7700_Arduino_Library.h>
#include <TCA6416A.h>
#include <DS248X.h>
#include <ICM_20948.h>

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

static VL53L1X range_sensor = VL53L1X(&Wire, 0);
static VEML7700 als_sensor;
static DS248X owMaster;
static ICM_20948_I2C icm;


#define CONVERT_TEMPERATURE_COMMAND       0x44
#define READ_SCRATCHPAD_COMMAND           0xBE
#define WRITE_SCRATCHPAD_COMMAND          0x4e

uint8_t getCRC8(uint8_t* addr, uint8_t len);
uint8_t owAddress[2][8];

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

StaticJsonDocument<256> values;
StaticJsonDocument<256> control;
StaticJsonDocument<256> status;
StaticJsonDocument<256> imu;
StaticJsonDocument<128> tof;

esp_adc_cal_characteristics_t adc2_chars;

bool mqtt_started = false;
mqtt_event_callback_t hasp_mqtt_event_handler;

bool pump_update = false;
unsigned long pump_update_millis = 0;

int batt_cap_pct[4] = {0};
unsigned long batt_cap_millis[4] = {0};

int tank_state = 0;
unsigned long tank_update_millis = 0;

StaticJsonDocument<512> nv_config;
static const char nv_filename[] = "/nv_config";

void nv_config_save(void)
{
    char buffer[1024];

    size_t len = serializeJson(nv_config, buffer, sizeof(buffer));

    LOG_TRACE(TAG_CUSTOM, F(D_FILE_SAVING), nv_filename);
    File file = HASP_FS.open(nv_filename, "wb");

    if(file) {
        file.write((const uint8_t *)buffer, len);
        file.close();
        LOG_INFO(TAG_CUSTOM, "NV config saved: %s", buffer);
    } else {
        LOG_ERROR(TAG_CUSTOM, D_FILE_SAVE_FAILED, nv_filename);
    }
}

void nv_config_load(void)
{
    const char filename[] = "/nv_config";

    if(HASP_FS.exists(filename)) {
        char buffer[1024];

        LOG_TRACE(TAG_CUSTOM, F(D_FILE_LOADING), nv_filename);
        File file = HASP_FS.open(nv_filename, "rb");

        if(file) {
            file.read((uint8_t *)buffer, sizeof(buffer));
            file.close();
            
            LOG_INFO(TAG_CUSTOM, "NV config loaded: %s", buffer);
            deserializeJson(nv_config, (const char *)buffer); // use const to force deserialize to duplicate input
        } else {
            LOG_ERROR(TAG_CUSTOM, D_FILE_LOAD_FAILED, nv_filename);
        }
    }

    range_sensor.VL53L1X_SetXtalk(nv_config[F("tof_xtalk")] | 0); // saved crosstalk cps
    range_sensor.VL53L1X_SetROI(nv_config[F("tof_roi_w")] | 16, nv_config[F("tof_roi_h")] | 16); // 16x16 is max ROI
    range_sensor.VL53L1X_SetROICenter(nv_config[F("tof_roi_c")] | 199); // 199 = center pixel

    icm.setBiasAccelX(nv_config[F("imu_acc_x")] | 0L);
    icm.setBiasAccelY(nv_config[F("imu_acc_y")] | 0L);
    icm.setBiasAccelZ(nv_config[F("imu_acc_z")] | 0L);
    icm.setBiasGyroX(nv_config[F("imu_gyro_x")] | 0L);
    icm.setBiasGyroY(nv_config[F("imu_gyro_y")] | 0L);
    icm.setBiasGyroZ(nv_config[F("imu_gyro_z")] | 0L);
    icm.setBiasCPassX(nv_config[F("imu_cpass_x")] | 0L);
    icm.setBiasCPassY(nv_config[F("imu_cpass_y")] | 0L);
    icm.setBiasCPassZ(nv_config[F("imu_cpass_z")] | 0L);
}

void nv_config_clear(void)
{
    LOG_VERBOSE(TAG_CUSTOM, "NV memory cleared");

    HASP_FS.remove(nv_filename);
    nv_config_load();
}

bool icm_get_bias(void)
{
    int32_t bias = 0;
    bool success = (icm.getBiasGyroX(&bias) == ICM_20948_Stat_Ok);
    nv_config[F("imu_gyro_x")] = bias;
    success &= (icm.getBiasGyroY(&bias) == ICM_20948_Stat_Ok);
    nv_config[F("imu_gyro_y")] = bias;
    success &= (icm.getBiasGyroZ(&bias) == ICM_20948_Stat_Ok);
    nv_config[F("imu_gyro_z")] = bias;
    success &= (icm.getBiasAccelX(&bias) == ICM_20948_Stat_Ok);
    nv_config[F("imu_acc_x")] = bias;
    success &= (icm.getBiasAccelY(&bias) == ICM_20948_Stat_Ok);
    nv_config[F("imu_acc_y")] = bias;
    success &= (icm.getBiasAccelZ(&bias) == ICM_20948_Stat_Ok);
    nv_config[F("imu_acc_z")] = bias;
    success &= (icm.getBiasCPassX(&bias) == ICM_20948_Stat_Ok);
    nv_config[F("imu_cpass_x")] = bias;
    success &= (icm.getBiasCPassY(&bias) == ICM_20948_Stat_Ok);
    nv_config[F("imu_cpass_y")] = bias;
    success &= (icm.getBiasCPassZ(&bias) == ICM_20948_Stat_Ok);
    nv_config[F("imu_cpass_z")] = bias;

    if (success) {
        LOG_VERBOSE(TAG_CUSTOM, "IMU bias retrieved");
    } else {
        LOG_VERBOSE(TAG_CUSTOM, "IMU bias retrieval failed!");
    }
    return success;
}

void OwGetTempSensors()
{
    uint8_t crc8;

    for(int i = 0; i < 2; i++) {
        if(!owMaster.OWSearch(owAddress[i], true)) break;
        LOG_VERBOSE(TAG_CUSTOM, "\tFound device: %x", owAddress[i][0]);
        // Calculate the CRC (if the CRC byte, the 8th, byte is included
        // in the CRC calculation then getCRC8() will return 0)
        crc8 = getCRC8(owAddress[i], 7);
        LOG_VERBOSE(TAG_CUSTOM, "Calculated CRC of device serial is %x", crc8);
        if(owAddress[i][0] == 0x28) {
            LOG_VERBOSE(TAG_CUSTOM, "This device is a 1-wire thermometer");
        }
    }
    owMaster.OWResetSearch();
}

int OwReadTemp(uint8_t* addr)
{
    if(addr[0] != 0x28) return -2;
    if(!owMaster.OWReset()) return -1;
    owMaster.selectChannel(0);
    owMaster.OWSelect(addr);
    owMaster.setStrongPullup();
    owMaster.OWWriteByte(CONVERT_TEMPERATURE_COMMAND);
    return 0;
}

float OwGetTempResult(uint8_t* addr)
{
    double tempInC;

    uint8_t data[9];
    uint8_t CRCerror = 0;

    // delay(TEMPERATURE_CONVERSION_DELAY + DELAY_ADD);
    owMaster.clearStrongPullup();

    if(addr[0] != 0x28) return -200;
    if(!owMaster.OWReset()) return -100;
    owMaster.OWSelect(addr);
    owMaster.OWWriteByte(READ_SCRATCHPAD_COMMAND);

    // Getting temperature info from device state...
    // we need 9 uint8_t's
    for(int i = 0; i < 9; i++) {
        data[i] = owMaster.OWReadByte();
    }

    tempInC = (double)(*(int16_t *)&data[0]) / 16.0;
    CRCerror = data[8] != getCRC8(data, 8);

    if(CRCerror) {
        Serial.println("\tCRC error reading temperature");
        return -300;
    }

    if(tempInC > 125) {
        Serial.println("\tTemperature read error: over-range! ");
        return -400;
    }
    return tempInC;
}

void lp_loop()
{
    uint32_t mv;
    float r;
    int pct;
    static bool first = true;
    static float mv_filt;
    static int state = 0;
    static unsigned long periodicmillis = 0, samplemillis = 0;

    if(millis() - periodicmillis > 2000) {
        periodicmillis = millis() - 1000UL;
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
    periodicmillis += 1000UL;
    mv = esp_adc_cal_raw_to_voltage(analogRead(PINA_SNS_LP), &adc2_chars);
    if(first) {
        first = false;
        mv_filt = mv;
    } else {
        mv_filt = mv_filt + (mv - mv_filt) * 0.01;
    }
    ex.pin_write(PINX_SNS_LP_PWR, LOW);

    // get resistance from voltage divider with 56.8mA current source (LM317 current source with 22R) and 15k/15k
    // divider
    r = mv_filt * (0.001 / ((15.0 / (15 + 15) * (1.25 / 22))));

    // sensor is 0-90R with 90R being 100% full
    pct = 100 * r / 90;
    if(pct > 115) {
        values[F("lp")] = -1;
    } else if(pct > 100) {
        values[F("lp")] = 100;
    } else {
        values[F("lp")] = pct;
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
        pump_update = true;
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
    static StaticJsonDocument<512> doc;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch(event_id) {
        case MQTT_EVENT_CONNECTED:
            LOG_VERBOSE(TAG_CUSTOM, "MQTT connected");
            esp_mqtt_client_subscribe(mqttClient, (bms_mqtt_prefix + "#").c_str(), mqttQos);
            break; // fall through to original handler
        case MQTT_EVENT_DATA:
            String topic = String(event->topic).substring(0, event->topic_len);
            String msg = String(event->data).substring(0, event->data_len);
            String search = bms_mqtt_prefix + "bms";
            // LOG_VERBOSE(TAG_CUSTOM, "MQTT incoming: %s - %s\n", topic.c_str(), msg.c_str());
            if(!topic.startsWith(search)) {
                if(topic.startsWith(bms_mqtt_prefix)) return; // ignore all incoming with our prefix
                break;                                        // not our prefix so fall through to original handler
            }
            unsigned int bms = topic.substring(search.length()).toInt();
            if(bms > 3) return;
            deserializeJson(doc, msg);
            batt_cap_pct[bms] = doc["cap_remaining_pct"];
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
        obj = hasp_find_obj_from_page_id(HASP_GENMODAL_PG, HASP_GENMODAL_ID);
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

    if (range_sensor.InitSensor(0x29) == 0) {
        //range_sensor.VL53L1X_SetDistanceMode(2); // long distance mode
        range_sensor.VL53L1X_SetTimingBudgetInMs(200);
        range_sensor.VL53L1X_SetInterMeasurementInMs(200);
        range_sensor.VL53L1X_StartRanging();
    }

    als_sensor.begin();
    als_sensor.setIntegrationTime(VEML7700_INTEGRATION_400ms);

    I2CScanner scanner;
    scanner.Init();
    scanner.Scan();

    if (icm.begin(Wire, 1) == ICM_20948_Stat_Ok) {
        icm.initializeDMP();
          // Set up Digital Low-Pass Filter configuration
        ICM_20948_dlpcfg_t myDLPcfg;
        myDLPcfg.a = acc_d5bw7_n8bw3;
        myDLPcfg.g = gyr_d5bw7_n8bw9;
        icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr | ICM_20948_Internal_Mag), myDLPcfg);
        icm.enableDLPF(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr | ICM_20948_Internal_Mag, true);

        bool success = (icm.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
        success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
        success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);
        success &= (icm.setDMPODRrate(DMP_ODR_Reg_Accel, 10) == ICM_20948_Stat_Ok);
        success &= (icm.setDMPODRrate(DMP_ODR_Reg_Gyro, 10) == ICM_20948_Stat_Ok);
        success &= (icm.setDMPODRrate(DMP_ODR_Reg_Cpass, 10) == ICM_20948_Stat_Ok);

        success &= (icm.enableFIFO() == ICM_20948_Stat_Ok);
        success &= (icm.enableDMP() == ICM_20948_Stat_Ok);
        success &= (icm.resetDMP() == ICM_20948_Stat_Ok);
        success &= (icm.resetFIFO() == ICM_20948_Stat_Ok);

        if (success)
        {
            LOG_VERBOSE(TAG_CUSTOM, "ICM DMP init success.");
        }
        else
        {
            LOG_VERBOSE(TAG_CUSTOM, "ICM DMP init failed!");
        }

    } else {
        LOG_VERBOSE(TAG_CUSTOM, "ICM init failed!");
    }
    LOG_VERBOSE(TAG_CUSTOM, "ICM status: %d", icm.statusString());;

    sl_init(PIN_SNS_TANKS_IN, PIN_SNS_TANKS_OUT);

    if(owMaster.isConnected()) {
        LOG_VERBOSE(TAG_CUSTOM, "DS2482-100/800 present");
        owMaster.DS2482Reset();
        owMaster.writeConfig(0x01);
        owMaster.selectChannel(0);

        OwGetTempSensors();
        OwReadTemp(owAddress[0]);
        OwReadTemp(owAddress[1]);
    } else
        LOG_VERBOSE(TAG_CUSTOM, "No DS2482 present");

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

    nv_config_load();

    tank_update_millis = millis();

    custom_every_second();
    custom_every_5seconds();

    LOG_VERBOSE(TAG_CUSTOM, "Setup complete");
}

void custom_loop()
{
    static unsigned long fastmillis = 0;
    lp_loop();

    uint8_t range_ready = 0;
    range_sensor.VL53L1X_CheckForDataReady(&range_ready);
    if (range_ready) {
        uint16_t tmp = 0;
        char payload[128];
        range_sensor.VL53L1X_GetDistance(&tmp);
        tof[F("dist")] = tmp;
        range_sensor.VL53L1X_GetAmbientRate(&tmp);
        tof[F("amb")] = tmp;
        range_sensor.VL53L1X_GetRangeStatus((uint8_t *)&tmp);
        tof[F("stat")] = *(uint8_t *)&tmp;
        range_sensor.VL53L1X_ClearInterrupt();
        serializeJson(tof, payload, sizeof(payload));
        dispatch_state_subtopic("tof", payload);
    }

    if(millis() - fastmillis > 250) {
        fastmillis = millis();

        bool sent_accel = false, sent_gyro = false, sent_mag = false;
        icm_20948_DMP_data_t data;
        icm.readDMPdataFromFIFO(&data);

        while ((icm.status == ICM_20948_Stat_Ok) || (icm.status == ICM_20948_Stat_FIFOMoreDataAvail))
        {
            if ((data.header & DMP_header_bitmap_Accel) > 0 && !sent_accel)
            {
                float x = data.Raw_Accel.Data.Y * 0.0001221, y = -data.Raw_Accel.Data.Z * 0.0001221, z = data.Raw_Accel.Data.X * 0.0001221;
                imu[F("acc_x")] = (int)(1000.0 * (x - (nv_config[F("imu_acc_x_offs")] | 0.0))) * 0.001;
                imu[F("acc_y")] = (int)(1000.0 * (y - (nv_config[F("imu_acc_y_offs")] | 0.0))) * 0.001;
                imu[F("acc_z")] = (int)(1000.0 * (z - (nv_config[F("imu_acc_z_offs")] | 0.0))) * 0.001;
                float a = sqrt(x * x + z * z);
                float b = sqrt(y * y + z * z);
                static float roll_filt = 0, pitch_filt = 0;
                roll_filt += (atan2(x, b) * (360 / (2 * M_PI)) - roll_filt) * 0.4;
                pitch_filt += (atan2(y, a) * (360 / (2 * M_PI)) - pitch_filt) * 0.4;
                imu[F("roll")] = (int)(100.0 * (roll_filt - (nv_config[F("imu_roll_offs")] | 0.0))) * 0.01;
                imu[F("pitch")] = (int)(100.0 * (pitch_filt - (nv_config[F("imu_pitch_offs")] | 0.0))) * 0.01;
                sent_accel = true;
            }
            else if ((data.header & DMP_header_bitmap_Gyro_Calibr) > 0 && !sent_gyro)
            {
                imu[F("gyro_x")] = (int)((1000.0 * 0.00001526) * data.Gyro_Calibr.Data.Y) * 0.001;
                imu[F("gyro_y")] = (int)((1000.0 * 0.00001526) * -data.Gyro_Calibr.Data.Z) * 0.001;
                imu[F("gyro_z")] = (int)((1000.0 * 0.00001526) * data.Gyro_Calibr.Data.X) * 0.001;
                sent_gyro = true;
            }
            else if ((data.header & DMP_header_bitmap_Compass_Calibr) > 0 && !sent_mag)
            {
                imu[F("mag_x")] = (int)((1000.0 * 0.00001526) * data.Compass_Calibr.Data.Y) * 0.001;
                imu[F("mag_y")] = (int)((1000.0 * 0.00001526) * -data.Compass_Calibr.Data.Z) * 0.001;
                imu[F("mag_z")] = (int)((1000.0 * 0.00001526) * data.Compass_Calibr.Data.X) * 0.001;
                sent_mag = true;
            }
            if (sent_accel && sent_gyro && sent_mag) break;
            icm.readDMPdataFromFIFO(&data);
        }
        if (!sent_accel) icm.resetDMP();
        icm.resetFIFO();

        char payload[256];
        serializeJson(imu, payload, sizeof(payload));
        dispatch_state_subtopic("imu", payload);
    }

    if(tank_state < SL_TANKS << 1) {
        static int tries = SL_RETRY;
        if(!(tank_state & 1)) { // on even states do read
            if(millis() - tank_update_millis > 2000) {
                tank_update_millis = millis();
            } else if(millis() - tank_update_millis > 450) {
                sl_readtank(tank_state >> 1);
                tank_state++;
            }
        } else { // on odd states get result
            if(millis() - tank_update_millis > 500) {
                int result = sl_result();
                if(result >= 0 || --tries == 0) {
                    switch(tank_state) {
                        case 1:
                            values[F("fresh")] = result;
                            break;
                        case 3:
                            values[F("grey")] = result;
                            break;
                        case 5:
                            values[F("black")] = result;
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
    char str[12];
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
    char payload[256];
    status[F("als")] = (int)(100.0 * als_sensor.getLux()) * 0.01;

    if(!pump_update || millis() - pump_update_millis > 5000) { // only update pump after 5s from last change
        control[F("pump")] = ex.pin_read(PINX_PUMP_IN) ? "ON" : "OFF";

        lv_obj_t* button = hasp_find_obj_from_page_id(HASP_PUMPBUT_PG, HASP_PUMPBUT_ID);
        if(!button) return;
        if(control[F("pump")] == "ON") {
            lv_obj_add_state(button, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(button, LV_STATE_CHECKED);
        }

        pump_update_millis = millis() + 5000UL; // set so that it doesn't roll over eventually
        pump_update = false;
    }

    control[F("heater_lp_fault")] = ex.pin_read(PINX_HEATER_LP_FLT) ? "ON" : "OFF";
    lv_obj_t* obj = hasp_find_obj_from_page_id(HASP_HEATERLPBUT_PG, HASP_HEATERLPBUT_ID);
    if(!obj) return;
    if(control[F("heater_lp_fault")] == "ON") {
        lv_obj_set_style_local_outline_opa(obj, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_COVER);
    } else {
        lv_obj_set_style_local_outline_opa(obj, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    }

    serializeJson(control, payload, sizeof(payload));
    dispatch_state_subtopic("control", payload);
    serializeJson(status, payload, sizeof(payload));
    dispatch_state_subtopic("status", payload);
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
    values["therm_temp"][0] = (int)(10.0 * OwGetTempResult(owAddress[0])) * 0.1;
    values["therm_temp"][1] = (int)(10.0 * OwGetTempResult(owAddress[1])) * 0.1;
    OwReadTemp(owAddress[0]);
    OwReadTemp(owAddress[1]);

    if(tank_state == SL_TANKS << 1) { // only request a new tank update if we have completed the last one
        tank_state = 0;
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

    update_bar(HASP_FRESHBAR_PG, HASP_FRESHBAR_ID, values[F("fresh")]);
    update_bar(HASP_GREYBAR_PG, HASP_GREYBAR_ID, values[F("grey")]);
    update_bar(HASP_BLACKBAR_PG, HASP_BLACKBAR_ID, values[F("black")]);
    update_bar(HASP_LPBAR_PG, HASP_LPBAR_ID, values[F("lp")]);
    update_bar(HASP_BATTBAR_PG, HASP_BATTBAR_ID, batt_cap_avg);

    update_label(HASP_FRESHLBL_PG, HASP_FRESHLBL_ID, values[F("fresh")]);
    update_label(HASP_GREYLBL_PG, HASP_GREYLBL_ID, values[F("grey")]);
    update_label(HASP_BLACKLBL_PG, HASP_BLACKLBL_ID, values[F("black")]);
    update_label(HASP_LPLBL_PG, HASP_LPLBL_ID, values[F("lp")]);
    update_label(HASP_BATTLBL_PG, HASP_BATTLBL_ID, batt_cap_avg);

    char payload[256];
    serializeJson(values, payload, sizeof(payload));
    dispatch_state_subtopic("values", payload);
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
    JsonObject diag = doc.createNestedObject(F("diag"));
    diag[F("fresh_pwr")] = sl_power[0];
    diag[F("grey_pwr")] = sl_power[1];
    diag[F("black_pwr")] = sl_power[2];
}

void custom_topic_payload(const char* topic, const char* payload, uint8_t source)
{
    lv_obj_t* obj = nullptr;

    // calibrate VL53L10X crosstalk.  payload is the range in mm of the grey 17% reflectance target.
    if(!strcmp(topic, "range_cal")) {
        uint16_t xtalk;
        range_sensor.VL53L1X_CalibrateXtalk(atoi(payload), &xtalk);
        nv_config[F("vl53l1x_xtalk")] = xtalk;
        range_sensor.VL53L1X_SetXtalk(xtalk);
        range_sensor.VL53L1X_StartRanging();
        nv_config_save();
    }

    // before calibrating:
    // 1:Rotate the sensor around all three axes
    // 2:Hold the sensor stationary in all six orientations for a few seconds
    if(!strcmp(topic, "imu_cal")) {
        icm_get_bias();
        nv_config_save();
    }

    if(!strcmp(topic, "nv_clear")) {
        nv_config_clear();
    }

    if(!strcmp(topic, "nv_set")) {
        StaticJsonDocument<512> doc;
        deserializeJson(doc, payload);

        for (JsonPairConst kvp : doc.as<JsonObjectConst>()) // combine the new data into the old
        {
            nv_config[kvp.key()] = kvp.value();
        }
        nv_config_save();
        nv_config_load();
    }

    // All the rest are ON or OFF
    bool state = !strcasecmp(payload, "ON");

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

void custom_state_subtopic(const char* subtopic, const char* payload){
    // Not used
}

#endif // HASP_USE_CUSTOM