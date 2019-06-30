
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AM2320.h>
#include <PubSubClient.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

#include <rom/rtc.h>
#include <soc/rtc_wdt.h>
#include <driver/adc.h>
#include <esp_wifi.h>

#include "local_config.h"

///////////////////////////////////////////////////////////////////
// Definitions

// I2C
#define CONFIG_I2C_SDA 21
#define CONFIG_I2C_SCL 22

// Status LED
#define CONFIG_STATUS_LED 2
// Secondary status LED
#define CONFIG_STATUS_LED_EX 17
// Analog input for VCC measurement (ADC2)
#define CONFIG_ADC2_PIN 26

// Total work timeout (s)
#define CONFIG_WORK_TIMEOUT 10
// Extra time to wait after CONFIG_WORK_TIMEOUT before triggering WDT reset (s)
#define CONFIG_TIMEOUT_TO_WDT_RESET 5

const EventBits_t EVENT_BIT_WORK_DONE = BIT0;

// Test mode: use test MQTT server
//#define TEST_MODE
// Test mode: override deep sleep timeout
//#define TEST_DEEPSLEEP 20


///////////////////////////////////////////////////////////////////
// Globals

Adafruit_AM2320 g_am2320; // 1-VDD 2-SDA 3-GND 4-SCL, (SDA,SCL 10K pullup)

WiFiClient g_wifiClient;
PubSubClient g_mqttClient(g_wifiClient);

EventGroupHandle_t g_events = 0;
SemaphoreHandle_t  g_ledMutex = 0;

bool g_testMode = false; // test mode: woken up not by deep sleep timer

volatile uint32_t g_measureOk = false; // measurement successful
volatile uint32_t g_wifiOk = false; // WiFi connection successful
volatile uint32_t g_sendOk = false; // MQTT sending successful

volatile uint32_t g_shutdown = 0; // flag: shutting down, disallow task LED access

uint32_t g_adc = 0;
float g_vcc = 0;
float g_temperature = 0;
float g_humidity = 0;

uint32_t g_resetReason = 0;

///////////////////////////////////////////////////////////////////
// Functions

void LedOn(bool f)
{
#ifdef CONFIG_STATUS_LED
    digitalWrite(CONFIG_STATUS_LED, f ? HIGH : LOW);
#endif
#ifdef CONFIG_STATUS_LED_EX
    digitalWrite(CONFIG_STATUS_LED_EX, f ? HIGH : LOW);
#endif
}

void TaskLedOn(bool f)
{
    xSemaphoreTake(g_ledMutex, portMAX_DELAY);
    if (!g_shutdown)
        LedOn(f);
    xSemaphoreGive(g_ledMutex);
}

void TaskLedBlink(int millisOn, int millisOff)
{
  TaskLedOn(true);
  delay(millisOn);
  TaskLedOn(false);
  delay(millisOff);
}

// Read analog multiple times and average results
uint32_t analogReadAvg(unsigned pin, unsigned n)
{
  uint32_t sum = 0;
  for (unsigned i = 0; i < n; ++i)
    sum += analogRead(pin);
  return sum/n;
}

float adcToVoltage(unsigned vadc)
{
    return (float)vadc * ((float)CONFIG_ADC_VREF_VCC / CONFIG_ADC_VREF_RAW);
}

uint32_t calcDeepSleepTime()
{
    // Calculate sleep time
    uint32_t sleepTime = CONFIG_SLEEP_TIME_NORM;
    if (g_vcc < CONFIG_VOLTAGE_NORM)
    {
        if (g_vcc <= CONFIG_VOLTAGE_MIN)
        {
            sleepTime = CONFIG_SLEEP_TIME_MAX;
        }
        else
        {
            sleepTime = CONFIG_SLEEP_TIME_MAX - (uint32_t)(
                (g_vcc - CONFIG_VOLTAGE_MIN) *
                (CONFIG_SLEEP_TIME_MAX - CONFIG_SLEEP_TIME_NORM) /
                (CONFIG_VOLTAGE_NORM - CONFIG_VOLTAGE_MIN));
        }
    }
    return sleepTime;
}

bool doMeasurement(float& temp, float& hum)
{
#ifdef CONFIG_I2C_SDA
    Wire.begin(CONFIG_I2C_SDA,CONFIG_I2C_SCL);
    g_am2320.begin();
    temp = g_am2320.readTemperature();
    hum = g_am2320.readHumidity();

    if (isnan(temp) || isnan(hum))
      return false;

    return true;
#else
    return false;
#endif
}

bool connectWiFi()
{
    printf("Connecting to WiFi...");
    fflush(stdout);
    WiFi.mode(WIFI_STA);
    WiFi.begin(CONFIG_WIFI_SSID, CONFIG_WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
      if (g_shutdown)
      {
          printf("FAIL\n");
          return false;
      }
      TaskLedBlink(100,50);
      printf(".");
      fflush(stdout);
    }
    printf("OK\n");
    return true;
}

void disconnectWiFi()
{
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    esp_wifi_stop();
    esp_wifi_deinit();
    adc_power_off();
}

void armRtcWatchdog(uint32_t msec)
{
    rtc_wdt_protect_off();
    rtc_wdt_disable();
    rtc_wdt_set_time(RTC_WDT_STAGE3, msec);
    rtc_wdt_set_stage(RTC_WDT_STAGE3, RTC_WDT_STAGE_ACTION_RESET_RTC);
    rtc_wdt_enable();
    rtc_wdt_protect_on();
}

bool sendMqtt()
{
    const char *srvName = g_testMode ? CONFIG_MQTT_HOST_TEST : CONFIG_MQTT_HOST;
    printf("Connecting to MQTT server at %s:%u\n", srvName, CONFIG_MQTT_PORT);
    
    g_mqttClient.setServer(srvName, CONFIG_MQTT_PORT);
    if (! g_mqttClient.connect(CONFIG_MQTT_CLIENT_ID, CONFIG_MQTT_USERNAME, CONFIG_MQTT_PASSWORD))
    {
        printf("Error connecting to MQTT server\n");
        return false;
    }

    printf("Publishing data\n");
    unsigned nSent = 0;

    char msgBuf[32];
    if (g_mqttClient.publish(CONFIG_MQTT_TOPIC "/status", "online"))
        ++nSent;

    snprintf(msgBuf, sizeof(msgBuf), "%.2f", g_vcc);
    if (g_mqttClient.publish(CONFIG_MQTT_TOPIC "/U1", msgBuf))
        ++nSent;

    snprintf(msgBuf, sizeof(msgBuf), "%u", g_resetReason);
    if (g_mqttClient.publish(CONFIG_MQTT_TOPIC "/RST", msgBuf))
        ++nSent;

    unsigned MSGCNT = 3;

    if (g_testMode)
    {
        snprintf(msgBuf, sizeof(msgBuf), "%u", g_adc);
        if (g_mqttClient.publish(CONFIG_MQTT_TOPIC "/ADC", msgBuf))
            ++nSent;

        MSGCNT += 1;
    }

    if (g_measureOk)
    {
        snprintf(msgBuf, sizeof(msgBuf), "%.2f", g_temperature);
        if (g_mqttClient.publish(CONFIG_MQTT_TOPIC "/T1", msgBuf))
            ++nSent;
    
        snprintf(msgBuf, sizeof(msgBuf), "%.0f", g_humidity);
        if (g_mqttClient.publish(CONFIG_MQTT_TOPIC "/H1", msgBuf))
            ++nSent;

        MSGCNT += 2;
    }

    g_mqttClient.disconnect();

    if (nSent != MSGCNT)
    {
        printf("Data sending failed (sent %u/%u)\n", nSent, MSGCNT);
        return false;
    }

    printf("Data sent successfully\n");
    return true;
}

///////////////////////////////////////////////////////////////////
// Tasks

void workTask(void *taskParm)
{
    printf("Work task started\n");
    TaskLedBlink(150,100);

    g_measureOk = doMeasurement(g_temperature, g_humidity);
    if (g_measureOk)
        printf("Am2320: Temperature=%.2f C  Humidity=%.1f %%\n", g_temperature, g_humidity);
    else
        printf("Am2320: Measurement failed\n");

    TaskLedBlink(150,100);

    if (connectWiFi());
        g_wifiOk = true;

    if (g_wifiOk)
    {
        TaskLedOn(true);
        g_sendOk = sendMqtt();
    }

    xEventGroupSetBits(g_events, EVENT_BIT_WORK_DONE);
    vTaskDelete(0);
}

///////////////////////////////////////////////////////////////////
// Main

void setup()
{
#ifdef CONFIG_STATUS_LED
    pinMode(CONFIG_STATUS_LED, OUTPUT);
#endif
#ifdef CONFIG_STATUS_LED_EX
    pinMode(CONFIG_STATUS_LED_EX, OUTPUT);
#endif
    Serial.begin(115200);

    g_resetReason = rtc_get_reset_reason(0);
#ifdef TEST_MODE
    g_testMode = 1;
#else
    g_testMode = (g_resetReason == POWERON_RESET); //(g_resetReason == DEEPSLEEP_RESET || g_resetReason == SW_CPU_RESET);
#endif
    if (g_testMode)
        printf("Test mode: manual reset or power on\n");
    else
        printf("Normal mode: woken by deep sleep or other reason\n");

    g_adc = analogReadAvg(CONFIG_ADC2_PIN, 8);
    g_vcc = adcToVoltage(g_adc);
    printf("Vadc=%u  Vcc=%.2f\n", g_adc, g_vcc);

    if (g_adc == 0xFFF && g_resetReason != RTCWDT_RTC_RESET)
    {
        // restart using RTC
        printf("Forcing RTC restart\n");
        fflush(stdout);

        armRtcWatchdog(100);
        delay(1000);

        printf("RTC restart failed!\n");
    }

    armRtcWatchdog((CONFIG_WORK_TIMEOUT+CONFIG_TIMEOUT_TO_WDT_RESET) * 1000);

    g_events = xEventGroupCreate();
    g_ledMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(workTask, "work_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    EventBits_t ev = xEventGroupWaitBits(g_events, EVENT_BIT_WORK_DONE, 
        pdFALSE, pdTRUE, CONFIG_WORK_TIMEOUT * 1000 / portTICK_PERIOD_MS);    

    if ((ev & EVENT_BIT_WORK_DONE) == 0)
        printf("Wait timeout (%d s)\n", CONFIG_WORK_TIMEOUT);
    else
        printf("Work done\n");

    // disallow led access for task (possibly continuing running)
    xSemaphoreTake(g_ledMutex, portMAX_DELAY);
    g_shutdown = true;
    xSemaphoreGive(g_ledMutex);

    LedOn(false);

    // report status using 1,2,3 blinks
    // 1 blink: measure done
    // 2 blinks: wifi ok
    // 3 blinks: data sent
    int nBlink = 0;
    if (g_sendOk)
        nBlink = g_testMode ? 3 : 4;
    else if (g_measureOk)
        nBlink = g_wifiOk ? 2 : 1;

    for (int i = 0; i < nBlink; ++i)
    {
        delay(200);
        LedOn(true);
        delay(200);
        LedOn(false);
    }

    printf("Disconnecting WiFi\n");
    
    disconnectWiFi();

    uint32_t deepSleepTime = calcDeepSleepTime();

#ifdef TEST_DEEPSLEEP
    deepSleepTime = TEST_DEEPSLEEP;
#endif

    printf("Going to deep sleep for %u sec\n", deepSleepTime);

    // go to deep sleep
    esp_sleep_enable_timer_wakeup(1000 * 1000 * deepSleepTime);
    esp_deep_sleep_start();
}

// loop() does nothing
void loop()
{
  delay(50);
}
