#include <Arduino.h>
#include <LoRaWan-RAK4630.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h> 
#include <SparkFun_SHTC3.h>

Adafruit_BME680 bme;
SHTC3 mySHTC3;


#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

bool doOTAA = true; 

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE 
#define SCHED_QUEUE_SIZE 60                     
#define LORAWAN_DATERATE DR_3                   
#define LORAWAN_TX_POWER TX_POWER_0             
#define JOINREQ_NBTRIALS 5                      

DeviceClass_t g_CurrentClass = CLASS_A;         
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_EU868;    
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;         
uint8_t gAppPort = LORAWAN_APP_PORT;                     

static lmh_param_t g_lora_param_init = {LORAWAN_ADR_OFF, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

static void lorawan_has_joined_handler(void);
void lorawan_join_fail(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

static lmh_callback_t g_lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_fail};

uint8_t nodeDeviceEUI[8] = {0xac, 0x1f, 0x09, 0xff, 0xfe, 0x08, 0xff, 0x8e};
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16] = {0x5D, 0xE6, 0xCD, 0x24, 0xDC, 0xB5, 0xA6, 0x5C, 0x2E, 0xD6, 0xAA, 0x57, 0x23, 0xAA, 0x43, 0xFB};
  
#define LORAWAN_APP_DATA_BUFF_SIZE 64                    
#define LORAWAN_APP_INTERVAL 20000                   
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];       
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; 

TimerEvent_t appTimer;
static uint32_t timers_init(void);
static uint32_t count = 0;
static uint32_t count_fail = 0;
//=========================================================================================================================================================================
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);

  time_t serial_timeout = millis();
  while (!Serial)
  {
    if ((millis() - serial_timeout) < 5000)
    {
      delay(100);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    else
    {
      break;
    }
  }

  Serial.println("=====================================");
  Serial.println("Welcome to RAK4630 LoRaWan!!!");
  Serial.println("Type: OTAA");
  Serial.println("Region: EU868");
  Serial.println("=====================================");
  
  lora_rak4630_init();
  
  init_bme680();

  Serial.println("shtc3 init");
  Serial.print("Beginning sensor. Result = "); 
  mySHTC3.begin();              
  Wire.setClock(400000);             
  Serial.println();

   if (mySHTC3.passIDcrc)
  {            
    Serial.print("ID Passed Checksum. ");
    Serial.print("Device ID: 0b");
    Serial.println(mySHTC3.ID, BIN);
  }
  else
  {
    Serial.println("ID Checksum Failed. ");
  }
  
  uint32_t err_code;

  err_code = timers_init();
  if (err_code != 0)
  {
    Serial.printf("timers_init failed - %d\n", err_code);
    return;
  }

    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);

  err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }
  
  lmh_join();
}
//=========================================================================================================================================================================
void loop()
{

}
//=========================================================================================================================================================================
void lorawan_join_fail(void)
{
  Serial.println("OTAA join failed!");
}

void lorawan_has_joined_handler(void)
{
  Serial.println("OTAA Mode, Network Joined!");

  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&appTimer);
  }
}

void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
                app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    return;
  }
  if (!bme.performReading()) {
    return;
  }
  sensor_data();

  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
}

void tx_lora_periodic_handler(void)
{
  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&appTimer);
  Serial.println("Sending frame now...");
  send_lora_frame();
}

uint32_t timers_init(void)
{
  TimerInit(&appTimer, tx_lora_periodic_handler);
  return 0;
}
//=========================================================================================================================================================================
void init_bme680(void)
{
  Wire.begin();

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    return;
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); 
}

String data_bme = "";
String data_SHTC3 = "";

void sensor_data()
{
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  mySHTC3.update();

  double temp_bme = bme.temperature;
  double pres_bme = bme.pressure / 100.0;
  double hum_bme = bme.humidity;
  uint32_t gas_bme = bme.gas_resistance;

  float temp_SHTC3 = mySHTC3.toDegC();
  float hum_SHTC3 = mySHTC3.toPercent();

  data_bme = "Tem:" + String(temp_bme) + "C " + "Hum:" + String(hum_bme) + "% " + "Pres:" + String(pres_bme) + "KPa " + "Gas:" + String(gas_bme) + "Ohms";
  Serial.println(data_bme);
  
  data_SHTC3 = "Tem:" + String(temp_SHTC3) + "C " + "Hum:" + String(hum_SHTC3) + "% " + "KPa ";
  Serial.println(data_SHTC3);
  
  uint16_t t_bme = temp_bme * 100;
  uint16_t h_bme = hum_bme * 100;
  uint32_t pre_bme = pres_bme * 100;

  m_lora_app_data.buffer[0] = 0x01;
  m_lora_app_data.buffer[1] = (uint8_t)(t_bme >> 8);
  m_lora_app_data.buffer[2] = (uint8_t)t_bme;
  m_lora_app_data.buffer[3] = (uint8_t)(h_bme >> 8);
  m_lora_app_data.buffer[4] = (uint8_t)h_bme;
  m_lora_app_data.buffer[5] = (uint8_t)((pre_bme & 0xFF000000) >> 24);
  m_lora_app_data.buffer[6] = (uint8_t)((pre_bme & 0x00FF0000) >> 16);
  m_lora_app_data.buffer[7] = (uint8_t)((pre_bme & 0x0000FF00) >> 8);
  m_lora_app_data.buffer[8] = (uint8_t)(pre_bme & 0x000000FF);
  m_lora_app_data.buffer[9] = (uint8_t)((gas_bme & 0xFF000000) >> 24);
  m_lora_app_data.buffer[10] = (uint8_t)((gas_bme & 0x00FF0000) >> 16);
  m_lora_app_data.buffer[11] = (uint8_t)((gas_bme & 0x0000FF00) >> 8);
  m_lora_app_data.buffer[12] = (uint8_t)(gas_bme & 0x000000FF);
  m_lora_app_data.buffsize = 13;
}
