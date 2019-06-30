# SolarMeteo32

## License

MIT License

## General description

SolarMeteo32 is a (yet another) ESP32-based meteostation.
The design goal is to build station that can work unattended without
external power. It is powered by LiPo battery that is recharged by small solar
panel.

The working cycle is optimized for low power consumption. The station awakes
from deep sleep at 10 (or more) minutes, measures environmental conditions,
sends data to MQTT server and goes to sleep again.

### Power consumption

My experiments showed that stock ESP32 boards (Wemos ESP32) draws ~0.9mA
during deep sleep. An AM2320 sensor draws 0.1 mA (I also tested BME280, but 
it seemed to consume more).

I added some external circuitry: 10/20K voltage divisor for VCC
self-measurement and MCP100 reset control supervisory circuit.
All this stuff increased current consumption to 1.5mA.

### Data publishing

Measured data (temperature, humidity and self-monitoring data) is send to
NarodMon (http://www.narodmon.ru) using MQTT protocol.

### Indication

Single LED is used for indication. At measurement cycle it blinks fast when
connecting to WiFi, then goes on for MQTT sending, then makes 1 to 4 slow
blinks - 1 for successful measurement, 2 for successful WiFi connection, 3
or 4 for successful sending data to server.

### VCC self-measurement

For diagnostics purposes, device monitors its own power voltage.
Voltage is measured using internal ADC (ADC2, see below), input voltage is
converted to ESP-capable range using 20K/10K divisor. The divisor itself
draws 0.5mA.

### ADC2 usage

I decided to use ADC2 for measurement because initially planned to utilize
board with few pins (like ESP32 cam) where no ADC1 pins were available.

There is a known problem with ADC2 - after WiFi operations it becomes
unavailable - even if the device goes into deep sleep.
I have found the solution to force RTC restart of the board in this case.
This seemingly solved the problem.

