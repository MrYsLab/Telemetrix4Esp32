/*

  Copyright (c) 2023 Alan Yorinks All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
  Version 3 as published by the Free Software Foundation; either
  or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
  along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

/*
  This sketch is a telemetrix server for the esp32 BLE interface
*/

#include <Arduino.h>
#include "WiFi.h"
#include "Telemetrix4Esp32.h"
#include <Wire.h>
#include <dhtnew.h>
#include <ESP32Servo.h>
#include <Ultrasonic.h>
#include <SPI.h>
#include <OneWire.h>
#include <AccelStepper.h>

/* WIFI specific defines */
const char *ssid = "YOUR_NETWORK_SSID";
const char *password = "YOUR_NETWORK_PASSWORD";

uint16_t PORT = 31336;

WiFiServer wifiServer(PORT);

WiFiClient client;

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

static TimerHandle_t wifi_connection_timer = NULL;
#define CONNECTION_TIMEOUT_PERIOD 10000  // 10 seconds

// We define the following functions as extern
// to provide for forward referencing.

// If you need to add a new command, you must add the command handler
// here as well. When adding a new command, the command processing
// function address must also must be added to
// the command_table.

extern void serial_loopback();

extern void set_pin_mode();

extern void digital_write();

extern void analog_write();

extern void modify_reporting();

extern void get_firmware_version();

extern void servo_attach();

extern void servo_write();

extern void servo_detach();

extern void i2c_begin();

extern void i2c_read();

extern void i2c_write();

extern void sonar_new();

extern void dht_new();

extern void stop_all_reports();

extern void set_analog_scanning_interval();

extern void enable_all_reports();

extern void analog_out_attach();

extern void analog_out_detach();

extern void dac_write();

extern void dac_disable();

extern void reset_data();

extern void init_pin_structures();

extern void init_spi();

extern void write_blocking_spi();

extern void read_blocking_spi();

extern void set_format_spi();

extern void spi_cs_control();

extern void onewire_init();

extern void onewire_reset();

extern void onewire_select();

extern void onewire_skip();

extern void onewire_write();

extern void onewire_read();

extern void onewire_reset_search();

extern void onewire_search();

extern void onewire_crc8();

extern void set_pin_mode_stepper();

extern void stepper_move_to();

extern void stepper_move();

extern void stepper_run();

extern void stepper_run_speed();

extern void stepper_set_max_speed();

extern void stepper_set_acceleration();

extern void stepper_set_speed();

extern void stepper_get_distance_to_go();

extern void stepper_get_target_position();

extern void stepper_get_current_position();

extern void stepper_set_current_position();

extern void stepper_run_speed_to_position();

extern void stepper_stop();

extern void stepper_disable_outputs();

extern void stepper_enable_outputs();

extern void stepper_set_minimum_pulse_width();

extern void stepper_set_3_pins_inverted();

extern void stepper_set_4_pins_inverted();

extern void stepper_set_enable_pin();

extern void stepper_is_running();

extern void send_debug_info(uint8_t id, int value);

extern void rtos_fatal_error_report(char *report);

extern void get_next_command();

extern void send_report();

// Commands -received by this sketch
// Add commands retaining the sequential numbering.
// The order of commands here must be maintained in the command_table.
#define SERIAL_LOOP_BACK 0
#define SET_PIN_MODE 1
#define DIGITAL_WRITE 2
#define ANALOG_WRITE 3
#define MODIFY_REPORTING 4  // mode(all, analog, or digital), pin, enable or disable
#define GET_FIRMWARE_VERSION 5
#define SERVO_ATTACH 6
#define SERVO_WRITE 7
#define SERVO_DETACH 8
#define I2C_BEGIN 9
#define I2C_READ 10
#define I2C_WRITE 11
#define SONAR_NEW 12
#define DHT_NEW 13
#define STOP_ALL_REPORTS 14
#define SET_ANALOG_SCANNING_INTERVAL 15
#define ENABLE_ALL_REPORTS 16
#define ANALOG_OUT_ATTACH 17
#define ANALOG_OUT_DETACH 18
#define DAC_WRITE 19
#define RESET 20
#define DAC_DISABLE 21
#define SPI_INIT 22
#define SPI_WRITE_BLOCKING 23
#define SPI_READ_BLOCKING 24
#define SPI_SET_FORMAT 25
#define SPI_CS_CONTROL 26
#define ONE_WIRE_INIT 27
#define ONE_WIRE_RESET 28
#define ONE_WIRE_SELECT 29
#define ONE_WIRE_SKIP 30
#define ONE_WIRE_WRITE 31
#define ONE_WIRE_READ 32
#define ONE_WIRE_RESET_SEARCH 33
#define ONE_WIRE_SEARCH 34
#define ONE_WIRE_CRC8 35
#define SET_PIN_MODE_STEPPER 36
#define STEPPER_MOVE_TO 37
#define STEPPER_MOVE 38
#define STEPPER_RUN 30
#define STEPPER_RUN_SPEED 40
#define STEPPER_SET_MAX_SPEED 41
#define STEPPER_SET_ACCELERATION 42
#define STEPPER_SET_SPEED 43
#define STEPPER_SET_CURRENT_POSITION 44
#define STEPPER_RUN_SPEED_TO_POSITION 45
#define STEPPER_STOP 46
#define STEPPER_DISABLE_OUTPUTS 47
#define STEPPER_ENABLE_OUTPUTS 48
#define STEPPER_SET_MINIMUM_PULSE_WIDTH 49
#define STEPPER_SET_ENABLE_PIN 50
#define STEPPER_SET_3_PINS_INVERTED 51
#define STEPPER_SET_4_PINS_INVERTED 52
#define STEPPER_IS_RUNNING 53
#define STEPPER_GET_CURRENT_POSITION 54
#define STEPPER_GET_DISTANCE_TO_GO 55
#define STEPPER_GET_TARGET_POSITION 56




// When adding a new command update the command_table.
// The command length is the number of bytes that follow
// the command byte itself, and does not include the command
// byte in its length.
// The command_func is a pointer the command's function.
struct command_descriptor {
  // a pointer to the command processing function
  void (*command_func)(void);
};

// The following table is an array of pointers to the command functions

// Make sure to keep things in the proper order - command
// defines are indices into this table.
command_descriptor command_table[] = {
  { &serial_loopback },
  { &set_pin_mode },
  { &digital_write },
  { &analog_write },
  { &modify_reporting },
  { &get_firmware_version },
  { &servo_attach },
  { &servo_write },
  { &servo_detach },
  { &i2c_begin },
  { &i2c_read },
  { &i2c_write },
  { &sonar_new },
  { &dht_new },
  { &stop_all_reports },
  { &set_analog_scanning_interval },
  { &enable_all_reports },
  { &analog_out_attach },
  { &analog_out_detach },
  { &dac_write },
  { &reset_data },
  { &dac_disable },
  { &init_spi },
  { &write_blocking_spi },
  { &read_blocking_spi },
  { &set_format_spi },
  { &spi_cs_control },
  { &onewire_init },
  { &onewire_reset },
  { &onewire_select },
  { &onewire_skip },
  { &onewire_write },
  { &onewire_read },
  { &onewire_reset_search },
  { &onewire_search },
  { &onewire_crc8 },
  { &set_pin_mode_stepper },
  { &stepper_move_to },
  { &stepper_move },
  { &stepper_run },
  { &stepper_run_speed },
  { &stepper_set_max_speed },
  { &stepper_set_acceleration },
  { &stepper_set_speed },
  (&stepper_set_current_position),
  (&stepper_run_speed_to_position),
  (&stepper_stop),
  (&stepper_disable_outputs),
  (&stepper_enable_outputs),
  (&stepper_set_minimum_pulse_width),
  (&stepper_set_enable_pin),
  (&stepper_set_3_pins_inverted),
  (&stepper_set_4_pins_inverted),
  (&stepper_is_running),
  (&stepper_get_current_position),
  { &stepper_get_distance_to_go },
  (&stepper_get_target_position),
};

// Input pin reporting control sub commands (modify_reporting)
#define REPORTING_DISABLE_ALL 0
#define REPORTING_ANALOG_ENABLE 1
#define REPORTING_DIGITAL_ENABLE 2
#define REPORTING_ANALOG_DISABLE 3
#define REPORTING_DIGITAL_DISABLE 4

// maximum length of a command in bytes
#define MAX_COMMAND_LENGTH 32

#define MAX_REPORT_LENGTH 64

// FreeRTOS elements
// Data coming in from the transport is placed on this queue.
QueueHandle_t command_q = xQueueCreate(32, MAX_COMMAND_LENGTH);

// Report queue
QueueHandle_t report_q = xQueueCreate(64, MAX_REPORT_LENGTH);

// task handle
TaskHandle_t xHandle;


// buffer to hold data pulled off of the command queue
uint8_t command_buffer[MAX_COMMAND_LENGTH];

// Pin mode definitions

#define AT_INPUT 0
#define AT_OUTPUT 1
#define AT_INPUT_PULLUP 2
#define AT_ANALOG 3
#define AT_SERVO 4
#define AT_SONAR 5
#define AT_DHT 6
#define AT_TOUCH 7
#define AT_PWM_OUT 8
#define AT_INPUT_PULLDOWN 9

#define AT_MODE_NOT_SET 255

// maximum number of pins supported
#define MAX_PINS_SUPPORTED 40

// Reports - sent from this sketch
#define DIGITAL_REPORT DIGITAL_WRITE
#define ANALOG_REPORT ANALOG_WRITE
#define FIRMWARE_REPORT 5
#define SERVO_UNAVAILABLE 6
#define I2C_TOO_FEW_BYTES_RCVD 7
#define I2C_TOO_MANY_BYTES_RCVD 8
#define I2C_READ_REPORT 9
#define SONAR_DISTANCE 10
#define DHT_REPORT 11
#define TOUCH_REPORT 12
#define SPI_REPORT 13
#define ONE_WIRE_REPORT 14
#define STEPPER_DISTANCE_TO_GO 15
#define STEPPER_TARGET_POSITION 16
#define STEPPER_CURRENT_POSITION 17
#define STEPPER_RUNNING_REPORT 18
#define STEPPER_RUN_COMPLETE_REPORT 19
#define DEBUG_PRINT 99

// DHT Report sub-types
#define DHT_DATA 0
#define DHT_READ_ERROR 1

// firmware version - update this when bumping the version
#define FIRMWARE_MAJOR 1
#define FIRMWARE_MINOR 0
#define FIRMWARE_BUILD 2

// Scanning intervals

// analog scanning time management
uint8_t analog_sampling_interval = 19;
// touch pin scanning time management
uint8_t touch_sampling_interval = 19;
// sonar scanning time management
uint8_t sonar_scan_interval = 33;  // Milliseconds between sensor pings
// dht scanning time management
unsigned int dht_scan_interval = 2200;  // scan dht's every 2.2 seconds

// a flag to stop sending all report messages
bool stop_reports = false;

// a descriptor for digital pins
struct pin_descriptor {
  uint8_t pin_number;
  uint8_t pin_mode;
  bool reporting_enabled;  // If true, then send reports if an input pin
  int last_value;          // Last value read for input mode
};

// an array of digital_pin_descriptors
pin_descriptor the_digital_pins[MAX_PINS_SUPPORTED];

// a descriptor for digital pins
struct analog_pin_descriptor {
  uint8_t pin_number;
  uint8_t pin_mode;
  bool reporting_enabled;  // If true, then send reports if an input pin
  int last_value;          // Last value read for input mode
  int differential;        // difference between current and last value needed
                           // to generate a report
};

// an array of analog_pin_descriptors
analog_pin_descriptor the_analog_pins[MAX_PINS_SUPPORTED];

struct touch_pin_descriptor {
  uint8_t pin_number;
  bool reporting_enabled;
  int last_value;
  int differential;
};

// an array of touch_pin descriptors
touch_pin_descriptor the_touch_pins[MAX_PINS_SUPPORTED];

// servo management
Servo servos[MAX_SERVOS];

// this array allows us to retrieve the servo object
// associated with a specific pin number
uint8_t pin_to_servo_index_map[MAX_SERVOS];

// HC-SR04 Sonar Management
#define MAX_SONARS 6

struct Sonar {
  uint8_t trigger_pin;
  unsigned int last_value;
  Ultrasonic *usonic;
};

// DHT Management
#define MAX_DHTS 6                 // max number of devices
#define READ_FAILED_IN_SCANNER 0   // read request failed when scanning
#define READ_IN_FAILED_IN_SETUP 1  // read request failed when initially setting up

struct DHT {
  uint8_t pin;
  unsigned int last_value;
  DHTNEW *dht_sensor;
};


/* OneWire Object*/

// a pointer to a OneWire object
OneWire *ow = NULL;

#define MAX_NUMBER_OF_STEPPERS 4

// A class to store device objects

class Devices {
public:
  // stepper device storage and management
  AccelStepper *steppers[MAX_NUMBER_OF_STEPPERS];
  uint8_t stepper_run_modes[MAX_NUMBER_OF_STEPPERS];
  bool ok_to_run_motors = false;

  // DHT device storage
  DHT dhts[MAX_DHTS];

  // sonar device storage
  Sonar sonars[MAX_SONARS];

  // stepper variables
  int steppers_index = 0;

  // dht variables
  uint8_t dht_index = 0;  // index into dht struct

  // sonar variables
  uint8_t sonars_index = 0;  // index into sonars struct
  uint8_t last_sonar_visited = 0;

  // methods to devices
  void add_a_stepper(int interface, uint8_t pin1, uint8_t pin2,
                     uint8_t pin3, uint8_t pin4, bool enable) {

    if (this->steppers_index < MAX_NUMBER_OF_STEPPERS) {
      this->steppers[this->steppers_index] = new AccelStepper(interface,
                                                              pin1, pin2, pin3, pin4, enable);
      this->steppers_index++;
    }
  }

  void add_a_dht(uint8_t pin) {
    this->dhts[this->dht_index].dht_sensor = new DHTNEW(pin);
    this->dhts[this->dht_index].pin = pin;
  }

  void add_sonar(uint8_t trigger_pin, uint8_t echo_pin) {
    this->sonars[this->sonars_index].usonic = new Ultrasonic(trigger_pin, echo_pin, 80000UL);
    this->sonars[this->sonars_index].trigger_pin = trigger_pin;
    this->sonars_index++;
  }
};

// Instantiate the devices class
Devices devices = Devices();

// allow input scanning and motor running
bool can_scan = false;

// ip connection status
bool deviceConnected = false;

// this is a diagnostic to be called internally to dump the contents
// of a data buffer
void dump_buffer(uint8_t *buffer, int len) {
  Serial.print("start dump buffer: ");
  for (int i = 0; i < len; i++) {
    Serial.print(buffer[i]);
    Serial.print(" ");
  }
  Serial.println("end dump buffer");
}

// command functions

// A method to send debug data across the serial link
void send_debug_info(uint8_t id, int value) {

  uint8_t debug_buffer[64];
  memset(debug_buffer, 0, sizeof(debug_buffer));

  debug_buffer[0] = 4;
  debug_buffer[1] = (uint8_t)DEBUG_PRINT;
  debug_buffer[2] = id;
  debug_buffer[3] = highByte(value);
  debug_buffer[4] = lowByte(value);
  if (xQueueSend(report_q, (const void *)debug_buffer, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send reqport_q for send_debug_info failed");
  }
}

// command functions
void serial_loopback() {

  uint8_t loop_back_buffer[64];

  memset(loop_back_buffer, 0, sizeof(loop_back_buffer));

  loop_back_buffer[0] = 2;
  loop_back_buffer[1] = (uint8_t)SERIAL_LOOP_BACK;
  loop_back_buffer[2] = command_buffer[2];
  if (xQueueSend(report_q, (const void *)loop_back_buffer, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send reqport_q for serial_loopback failed");
  }
  vTaskDelay(14 / portTICK_PERIOD_MS);
}

void set_pin_mode() {
  uint8_t pin;
  uint8_t mode;
  uint8_t resolution;
  uint8_t channel;
  unsigned int fx;
  double frequency = 0.0;
  pin = command_buffer[0];
  mode = command_buffer[1];

  switch (mode) {
    case AT_INPUT:
      the_digital_pins[pin].pin_mode = AT_INPUT;
      the_digital_pins[pin].reporting_enabled = command_buffer[2];
      pinMode(pin, INPUT);
      break;
    case AT_INPUT_PULLUP:
      the_digital_pins[pin].pin_mode = AT_INPUT_PULLUP;
      the_digital_pins[pin].reporting_enabled = command_buffer[2];
      pinMode(pin, INPUT_PULLUP);
      break;
    case AT_INPUT_PULLDOWN:
      the_digital_pins[pin].pin_mode = AT_INPUT_PULLDOWN;
      the_digital_pins[pin].reporting_enabled = command_buffer[2];
      pinMode(pin, INPUT_PULLDOWN);
      break;
    case AT_TOUCH:
      the_touch_pins[pin].differential = (command_buffer[2] << 8) + command_buffer[3];
      the_touch_pins[pin].reporting_enabled = command_buffer[4];
      // Serial.print("TOUCH");
      // Serial.println(pin);
      break;
    case AT_OUTPUT:
      the_digital_pins[pin].pin_mode = mode;
      pinMode(pin, OUTPUT);
      break;
    case AT_ANALOG:
      the_analog_pins[pin].pin_mode = mode;
      the_analog_pins[pin].differential = (command_buffer[2] << 8) + command_buffer[3];
      the_analog_pins[pin].reporting_enabled = command_buffer[4];
      break;
    case AT_PWM_OUT:
      // command_buffer[2] = channel
      // command_buffer[3] = resolution
      // command_buffer[4] to command_buffer[11] = frequency

      channel = command_buffer[2];
      resolution = command_buffer[3];

      memcpy(&frequency, &command_buffer[4], sizeof(double));

      ledcSetup(channel, frequency, resolution);
      ledcAttachPin(pin, channel);
      break;
    default:
      break;
  }
}

void analog_out_attach() {
  // command_buffer[0] = pin number
  // command_buffer[1] = channel
  ledcAttachPin(command_buffer[0], command_buffer[1]);
}

void analog_out_detach() {
  ledcDetachPin(command_buffer[0]);
}

void set_analog_scanning_interval() {
  analog_sampling_interval = command_buffer[0];
}

void digital_write() {
  uint8_t pin;
  uint8_t value;
  pin = command_buffer[2];
  value = command_buffer[3];
  digitalWrite(pin, value);
}

void analog_write() {
  // command_buffer[0] = channel
  // command_buffer[1] = value_msb,
  // command_buffer[2] = value_lsb

  u_int value;

  value = (command_buffer[1] << 8) + command_buffer[2];
  ledcWrite(command_buffer[0], value);
}

void dac_write() {
  // command_buffer[0] = pin
  // command_buffer[1] = value

  dacWrite(command_buffer[0], command_buffer[1]);
}

void dac_disable() {
  dacDisable(command_buffer[0]);
}

void modify_reporting() {
  int pin = command_buffer[1];

  switch (command_buffer[0]) {
    case REPORTING_DISABLE_ALL:
      for (int i = 0; i < MAX_PINS_SUPPORTED; i++) {
        the_digital_pins[i].reporting_enabled = false;
      }
      for (int i = 0; i < MAX_PINS_SUPPORTED; i++) {
        the_analog_pins[i].reporting_enabled = false;
      }
      break;
    case REPORTING_ANALOG_ENABLE:
      if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET) {
        the_analog_pins[pin].reporting_enabled = true;
      }
      break;
    case REPORTING_ANALOG_DISABLE:
      if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET) {
        the_analog_pins[pin].reporting_enabled = false;
      }
      break;
    case REPORTING_DIGITAL_ENABLE:
      if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET) {
        the_digital_pins[pin].reporting_enabled = true;
      }
      break;
    case REPORTING_DIGITAL_DISABLE:
      if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET) {
        the_digital_pins[pin].reporting_enabled = false;
      }
      break;
    default:
      break;
  }
}

void get_firmware_version() {
  uint8_t report_message[64] = { 4, FIRMWARE_REPORT, FIRMWARE_MAJOR, FIRMWARE_MINOR,
                                 FIRMWARE_BUILD };

  if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send on report_q failed");
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

/***************************************************
   Servo Commands
 **************************************************/

// Find the first servo that is not attached to a pin
// This is a helper function not called directly via the API
int find_servo() {
  int index = -1;
  for (int i = 0; i < MAX_SERVOS; i++) {
    if (servos[i].attached() == false) {
      index = i;
      break;
    }
  }
  return index;
}

void servo_attach() {

  uint8_t pin = command_buffer[0];
  int servo_found = -1;

  int minpulse = (command_buffer[1] << 8) + command_buffer[2];
  int maxpulse = (command_buffer[3] << 8) + command_buffer[4];

  // find the first available open servo
  servo_found = find_servo();
  if (servo_found != -1) {
    pin_to_servo_index_map[servo_found] = pin;
    servos[servo_found].attach(pin, minpulse, maxpulse);
  } else {
    // no open servos available, send a report back to client
    byte report_message[64];
    memset(report_message, 0, sizeof(report_message));

    report_message[0] = SERVO_UNAVAILABLE;
    report_message[1] = pin;
    if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
      rtos_fatal_error_report((char *)"Send reqport_q for servo_attach failed");
    }
  }
}

// set a servo to a given angle
void servo_write() {
  uint8_t pin = command_buffer[0];
  int angle = command_buffer[1];
  servos[0].write(angle);
  // find the servo object for the pin
  for (int i = 0; i < MAX_SERVOS; i++) {
    if (pin_to_servo_index_map[i] == pin) {

      servos[i].write(angle);
      return;
    }
  }
}

// detach a servo and make it available for future use
void servo_detach() {
  uint8_t pin = command_buffer[0];

  // find the servo object for the pin
  for (int i = 0; i < MAX_SERVOS; i++) {
    if (pin_to_servo_index_map[i] == pin) {

      pin_to_servo_index_map[i] = -1;
      servos[i].detach();
    }
  }
}

/***********************************
   i2c functions
 **********************************/

void i2c_begin() {
  Wire.begin();
}

void i2c_read() {
  // data in the incoming message:
  // address, [0]
  // register, [1]
  // number of bytes, [2]
  // stop transmitting flag [3]

  int message_size = 0;
  uint8_t address = command_buffer[0];
  uint8_t the_register = command_buffer[1];
  uint8_t i2c_report_message[64];
  memset(i2c_report_message, 0, sizeof(i2c_report_message));

  Wire.beginTransmission(address);
  Wire.write((uint8_t)the_register);
  Wire.endTransmission(command_buffer[3]);       // default = true
  Wire.requestFrom(address, command_buffer[2]);  // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if (command_buffer[2] < Wire.available()) {

    i2c_report_message[0] = 3;
    i2c_report_message[1] = I2C_TOO_FEW_BYTES_RCVD;
    i2c_report_message[2] = 1;
    i2c_report_message[3] = address;
    if (xQueueSend(report_q, (const void *)i2c_report_message, 20) != pdTRUE) {
      rtos_fatal_error_report((char *)"Send reqport_q i2c_too_few_bytes failed");
    }
    return;
  } else if (command_buffer[2] > Wire.available()) {

    i2c_report_message[0] = 3;
    i2c_report_message[1] = I2C_TOO_MANY_BYTES_RCVD;
    i2c_report_message[2] = 1;
    i2c_report_message[3] = address;
    if (xQueueSend(report_q, (const void *)i2c_report_message, 20) != pdTRUE) {
      rtos_fatal_error_report((char *)"Send reqport_q i2c_too_many_bytes failed");
    }
    return;
  }

  // packet length
  i2c_report_message[0] = command_buffer[2] + 4;

  // report type
  i2c_report_message[1] = I2C_READ_REPORT;

  // number of bytes read
  i2c_report_message[2] = command_buffer[2];  // number of bytes

  // device address
  i2c_report_message[3] = address;

  // device register
  i2c_report_message[4] = the_register;

  // append the data that was read
  for (message_size = 0; message_size < command_buffer[2] && Wire.available(); message_size++) {
    i2c_report_message[5 + message_size] = Wire.read();
  }
  // send slave address, register and received bytes
  if (xQueueSend(report_q, (const void *)i2c_report_message, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send reqport_q i2c_read failed");
  }
}

void i2c_write() {
  // command_buffer[0] is the number of bytes to send
  // command_buffer[1] is the device address
  // additional bytes to write= command_buffer[3..];

  Wire.beginTransmission(command_buffer[1]);

  // write the data to the device
  for (int i = 0; i < command_buffer[0]; i++) {
    Wire.write(command_buffer[i + 2]);
  }
  Wire.endTransmission();
  delayMicroseconds(70);
}

/***********************************
   HC-SR04 adding a new device
 **********************************/

void sonar_new() {
  // command_buffer[0] = trigger pin,  command_buffer[1] = echo pin
  devices.add_sonar((uint8_t)command_buffer[0], (uint8_t)command_buffer[1]);
}

/***********************************
   DHT adding a new device
 **********************************/

void dht_new() {
  int d_read;
  // report consists of:
  // 0 - uint8_t count
  // 1 - report type
  // 2 - dht report subtype
  // 3 - pin number
  // 4 - error value

  // pre-build an error report in case of a read error
  uint8_t report_message[64];
  memset(report_message, 0, sizeof(report_message));
  report_message[0] = 4;
  report_message[1] = (uint8_t)DHT_REPORT;
  report_message[2] = (uint8_t)DHT_READ_ERROR;

  devices.add_a_dht((uint8_t)command_buffer[0]);

  d_read = devices.dhts[devices.dht_index].dht_sensor->read();

  // if read return == zero it means no errors.
  if (d_read == 0) {
    devices.dht_index++;

  } else {
    // error found
    // send report and release the dht object


    report_message[3] = command_buffer[0];  // pin number
    report_message[4] = d_read;
    if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
      rtos_fatal_error_report((char *)"Send reqport_q for DHT_READ_ERROR error failed");
    }
    delete (devices.dhts[devices.dht_index].dht_sensor);
  }
}

// initialize the SPI interface
void init_spi() {

  int cs_pin;

  // Serial.print(command_buffer[1]);
  //  initialize chip select GPIO pins
  for (int i = 0; i < command_buffer[0]; i++) {
    cs_pin = command_buffer[1 + i];
    // Chip select is active-low, so we'll initialise it to a driven-high state
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
  }
  SPI.begin();
}

// write a number of blocks to the SPI device
void write_blocking_spi() {
  int num_bytes = command_buffer[0];

  for (int i = 0; i < num_bytes; i++) {
    SPI.transfer(command_buffer[1 + i]);
  }
}

// read a number of bytes from the SPI device
void read_blocking_spi() {
  uint8_t spi_report_message[64];
  memset(spi_report_message, 0, sizeof(spi_report_message));

  // command_buffer[0] == number of bytes to read
  // command_buffer[1] == read register

  // spi_report_message[0] = length of message including this element
  // spi_report_message[1] = SPI_REPORT
  // spi_report_message[2] = register used for the read
  // spi_report_message[3] = number of bytes returned
  // spi_report_message[4..] = data read

  // configure the report message
  // calculate the packet length
  spi_report_message[0] = command_buffer[0] + 3;  // packet length
  spi_report_message[1] = SPI_REPORT;
  spi_report_message[2] = command_buffer[1];  // register
  spi_report_message[3] = command_buffer[0];  // number of bytes read

  // write the register out. OR it with 0x80 to indicate a read
  SPI.transfer(command_buffer[1] | 0x80);

  // now read the specified number of bytes and place
  // them in the report buffer
  for (int i = 0; i < command_buffer[0]; i++) {
    spi_report_message[i + 4] = SPI.transfer(0x00);
  }
  if (xQueueSend(report_q, (const void *)spi_report_message, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send reqport_q for read_blocking spi error failed");
  }
}

// modify the SPI format
void set_format_spi() {

  SPISettings(command_buffer[0], command_buffer[1], command_buffer[2]);
}

// set the SPI chip select line
void spi_cs_control() {
  int cs_pin = command_buffer[0];
  int cs_state = command_buffer[1];
  digitalWrite(cs_pin, cs_state);
}

// Initialize the OneWire interface
void onewire_init() {
  ow = new OneWire(command_buffer[0]);
}

// send a OneWire reset
void onewire_reset() {

  uint8_t reset_return = ow->reset();

  uint8_t onewire_report_message[64];
  memset(onewire_report_message, 0, sizeof(onewire_report_message));

  onewire_report_message[0] = 3;
  onewire_report_message[1] = ONE_WIRE_REPORT;
  onewire_report_message[2] = ONE_WIRE_RESET;
  onewire_report_message[3] = reset_return;

  if (xQueueSend(report_q, (const void *)onewire_report_message, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send reqport_q for one wire reset failed");
  }
}

// send a OneWire select
void onewire_select() {

  uint8_t dev_address[8];

  for (int i = 0; i < 8; i++) {
    dev_address[i] = command_buffer[i];
  }
  ow->select(dev_address);
}

// send a OneWire skip
void onewire_skip() {
  ow->skip();
}

// write 1 byte to the OneWire device
void onewire_write() {

  // write data and power values
  ow->write(command_buffer[0], command_buffer[1]);
}

// read one byte from the OneWire device
void onewire_read() {

  // onewire_report_message[0] = length of message including this element
  // onewire_report_message[1] = ONEWIRE_REPORT
  // onewire_report_message[2] = message subtype = 29
  // onewire_report_message[3] = data read

  uint8_t data = ow->read();
  uint8_t onewire_report_message[64];
  memset(onewire_report_message, 0, sizeof(onewire_report_message));

  onewire_report_message[0] = 3;
  onewire_report_message[1] = ONE_WIRE_REPORT;
  onewire_report_message[2] = ONE_WIRE_READ;
  onewire_report_message[3] = data;

  if (xQueueSend(report_q, (const void *)onewire_report_message, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send reqport_q for one wire read failed");
  }
}

// Send a OneWire reset search command
void onewire_reset_search() {

  ow->reset_search();
}

// Send a OneWire search command
void onewire_search() {
  uint8_t onewire_report_message[64];

  memset(onewire_report_message, 0, sizeof(onewire_report_message));

  onewire_report_message[0] = 10;
  onewire_report_message[1] = ONE_WIRE_REPORT;
  onewire_report_message[2] = ONE_WIRE_SEARCH;
  onewire_report_message[3] = 0xff;
  onewire_report_message[4] = 0xff;
  onewire_report_message[5] = 0xff;
  onewire_report_message[6] = 0xff;
  onewire_report_message[7] = 0xff;
  onewire_report_message[8] = 0xff;
  onewire_report_message[9] = 0xff;
  onewire_report_message[10] = 0xff;

  ow->search(&onewire_report_message[3]);

  if (xQueueSend(report_q, (const void *)onewire_report_message, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send reqport_q for one wire search failed");
  }
}

// Calculate a OneWire CRC8 on a buffer containing a specified number of bytes
void onewire_crc8() {
  uint8_t onewire_report_message[64];

  memset(onewire_report_message, 0, sizeof(onewire_report_message));

  onewire_report_message[0] = 3;
  onewire_report_message[1] = ONE_WIRE_REPORT;
  onewire_report_message[2] = ONE_WIRE_CRC8;
  onewire_report_message[3] = ow->crc8(&command_buffer[1], command_buffer[0]);

  if (xQueueSend(report_q, (const void *)onewire_report_message, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send reqport_q for one wire crc8 failed");
  }
}

// Stepper Motor supported
// Stepper Motor supported
void set_pin_mode_stepper() {
  // motor_id = command_buffer[0]
  // interface = command_buffer[1]
  // pin1 = command_buffer[2]
  // pin2 = command_buffer[3]
  // pin3 = command_buffer[4]
  // pin4 = command_buffer[5]
  // enable = command_buffer[6]

  devices.add_a_stepper(command_buffer[1], command_buffer[2],
                        command_buffer[3], command_buffer[4],
                        command_buffer[5], command_buffer[6]);
}

void stepper_move_to() {
  // motor_id = command_buffer[0]
  // position MSB = command_buffer[1]
  // position MSB-1 = command_buffer[2]
  // position MSB-2 = command_buffer[3]
  // position LSB = command_buffer[4]
  // polarity = command_buffer[5]

  // convert the 4 position bytes to a long
  long position = (long)(command_buffer[1]) << 24;
  position += (long)(command_buffer[2]) << 16;
  position += command_buffer[3] << 8;
  position += command_buffer[4];
  if (command_buffer[5]) {
    position *= -1;
  }

  devices.steppers[command_buffer[0]]->moveTo(position);
}

void stepper_move() {

  // motor_id = command_buffer[0]
  // position MSB = command_buffer[1]
  // position MSB-1 = command_buffer[2]
  // position MSB-2 = command_buffer[3]
  // position LSB = command_buffer[4]
  // polarity = command_buffer[5]

  // convert the 4 position bytes to a long
  long position = (long)(command_buffer[1]) << 24;
  position += (long)(command_buffer[2]) << 16;
  position += command_buffer[3] << 8;
  position += command_buffer[4];
  if (command_buffer[5]) {
    position *= -1;
  }
  devices.steppers[command_buffer[0]]->move(position);
}

void stepper_run() {
  devices.stepper_run_modes[command_buffer[0]] = STEPPER_RUN;
  devices.ok_to_run_motors = true;
}

void stepper_run_speed() {
  // motor_id = command_buffer[0]
  devices.stepper_run_modes[command_buffer[0]] = STEPPER_RUN_SPEED;
  devices.ok_to_run_motors = true;
}

void stepper_set_max_speed() {
  // motor_id = command_buffer[0]
  // speed_msb = command_buffer[1]
  // speed_lsb = command_buffer[2]

  float max_speed = (float)((command_buffer[1] << 8) + command_buffer[2]);
  devices.steppers[command_buffer[0]]->setMaxSpeed(max_speed);
}

void stepper_set_acceleration() {
  // motor_id = command_buffer[0]
  // accel_msb = command_buffer[1]
  // accel = command_buffer[2]

  float acceleration = (float)((command_buffer[1] << 8) + command_buffer[2]);
  devices.steppers[command_buffer[0]]->setAcceleration(acceleration);
}

void stepper_set_speed() {

  // motor_id = command_buffer[0]
  // speed_msb = command_buffer[1]
  // speed_lsb = command_buffer[2]
  // polarity = command_buffer[3]

  float speed = (float)((command_buffer[1] << 8) + command_buffer[2]);
  if (command_buffer[3] == 1) {
    speed = speed * -1.0;
  }
  devices.steppers[command_buffer[0]]->setSpeed(speed);
}

void stepper_get_distance_to_go() {
  // motor_id = command_buffer[0]
  // report = STEPPER_DISTANCE_TO_GO, motor_id, distance(8 bytes)
  // motor_id = command_buffer[0]

  uint8_t report_message[64];
  memset(report_message, 0, sizeof(report_message));

  long dtg = devices.steppers[command_buffer[0]]->distanceToGo();

  report_message[0] = 6;
  report_message[1] = STEPPER_DISTANCE_TO_GO;
  report_message[2] = command_buffer[0];
  report_message[3] = (uint8_t)((dtg & 0xFF000000) >> 24);
  report_message[4] = (uint8_t)((dtg & 0x00FF0000) >> 16);
  report_message[5] = (uint8_t)((dtg & 0x0000FF00) >> 8);
  report_message[6] = (uint8_t)((dtg & 0x000000FF));

  if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send reqport_q for stepper_get_distance_to_go failed");
  }
}

void stepper_get_target_position() {
  // motor_id = command_buffer[0]

  // report = STEPPER_TARGET_POSITION, motor_id, distance(8 uint8_ts)

  uint8_t report_message[64];
  memset(report_message, 0, sizeof(report_message));

  long target = devices.steppers[command_buffer[0]]->targetPosition();
  report_message[0] = 6;
  report_message[1] = STEPPER_TARGET_POSITION;
  report_message[2] = command_buffer[0];
  report_message[3] = (uint8_t)((target & 0xFF000000) >> 24);
  report_message[4] = (uint8_t)((target & 0x00FF0000) >> 16);
  report_message[5] = (uint8_t)((target & 0x0000FF00) >> 8);
  report_message[6] = (uint8_t)((target & 0x000000FF));

  // motor_id = command_buffer[0]
  if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send reqport_q for stepper_get_target_position failed");
  }
}

void stepper_get_current_position() {
  // motor_id = command_buffer[0]

  // report = STEPPER_CURRENT_POSITION, motor_id, distance(8 uint8_ts)

  uint8_t report_message[64];
  memset(report_message, 0, sizeof(report_message));

  long position = devices.steppers[command_buffer[0]]->targetPosition();

  report_message[0] = 6;
  report_message[1] = STEPPER_CURRENT_POSITION;
  report_message[2] = command_buffer[0];
  report_message[3] = (uint8_t)((position & 0xFF000000) >> 24);
  report_message[4] = (uint8_t)((position & 0x00FF0000) >> 16);
  report_message[5] = (uint8_t)((position & 0x0000FF00) >> 8);
  report_message[6] = (uint8_t)((position & 0x000000FF));

  // motor_id = command_buffer[0]
  if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send reqport_q for stepper_get_current_position failed");
  }
}

void stepper_set_current_position() {
  // motor_id = command_buffer[0]
  // position MSB = command_buffer[1]
  // position MSB-1 = command_buffer[2]
  // position MSB-2 = command_buffer[3]
  // position LSB = command_buffer[4]

  // convert the 4 position bytes to a long
  long position = (long)(command_buffer[2]) << 24;
  position += (long)(command_buffer[2]) << 16;
  position += command_buffer[3] << 8;
  position += command_buffer[4];

  devices.steppers[command_buffer[0]]->setCurrentPosition(position);
}

void stepper_run_speed_to_position() {
  devices.stepper_run_modes[command_buffer[0]] = STEPPER_RUN_SPEED_TO_POSITION;
  devices.ok_to_run_motors = true;
}

void stepper_stop() {
  devices.steppers[command_buffer[0]]->stop();
  devices.steppers[command_buffer[0]]->disableOutputs();
  devices.stepper_run_modes[command_buffer[0]] = STEPPER_STOP;
}

void stepper_disable_outputs() {
  devices.steppers[command_buffer[0]]->disableOutputs();
}

void stepper_enable_outputs() {
  devices.steppers[command_buffer[0]]->enableOutputs();
}

void stepper_set_minimum_pulse_width() {
  unsigned int pulse_width = (command_buffer[1] << 8) + command_buffer[2];
  devices.steppers[command_buffer[0]]->setMinPulseWidth(pulse_width);
}

void stepper_set_enable_pin() {
  devices.steppers[command_buffer[0]]->setEnablePin((uint8_t)command_buffer[1]);
}

void stepper_set_3_pins_inverted() {
  // command_buffer[1] = directionInvert
  // command_buffer[2] = stepInvert
  // command_buffer[3] = enableInvert
  devices.steppers[command_buffer[0]]->setPinsInverted((bool)command_buffer[1],
                                                       (bool)command_buffer[2],
                                                       (bool)command_buffer[3]);
}

void stepper_set_4_pins_inverted() {
  // command_buffer[1] = pin1
  // command_buffer[2] = pin2
  // command_buffer[3] = pin3
  // command_buffer[4] = pin4
  // command_buffer[5] = enable
  devices.steppers[command_buffer[0]]->setPinsInverted((bool)command_buffer[1],
                                                       (bool)command_buffer[2],
                                                       (bool)command_buffer[3],
                                                       (bool)command_buffer[4],
                                                       (bool)command_buffer[5]);
}

void stepper_is_running() {
  // motor_id = command_buffer[0]

  // report = STEPPER_IS_RUNNING, motor_id, distance(8 bytes)

  uint8_t report_message[64];
  memset(report_message, 0, sizeof(report_message));

  report_message[0] = 2;
  report_message[1] = STEPPER_RUNNING_REPORT;
  report_message[2] = command_buffer[0];
  report_message[3] = devices.steppers[command_buffer[0]]->isRunning();

  if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
    rtos_fatal_error_report((char *)"Send reqport_q for stepper_is_running failed");
  }
}

void stop_all_reports() {
  stop_reports = true;
  vTaskDelay(20 / portTICK_PERIOD_MS);
}

void enable_all_reports() {
  stop_reports = false;
  vTaskDelay(20 / portTICK_PERIOD_MS);
}

void scan_digital_inputs(void *parameter) {
  uint8_t value;

  // report message

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = pin number
  // byte 3 = value

  uint8_t report_message[64];

  while (1) {

    if (can_scan) {
      for (int i = 0; i < MAX_PINS_SUPPORTED; i++) {
        if (the_digital_pins[i].pin_mode == AT_INPUT || the_digital_pins[i].pin_mode == AT_INPUT_PULLUP || the_digital_pins[i].pin_mode == AT_INPUT_PULLDOWN) {
          if (the_digital_pins[i].reporting_enabled) {
            // if the value changed since last read
            value = (uint8_t)digitalRead(the_digital_pins[i].pin_number);
            if (value != the_digital_pins[i].last_value) {
              memset(report_message, 0, sizeof(report_message));
              report_message[0] = 3;
              report_message[1] = DIGITAL_REPORT;
              the_digital_pins[i].last_value = value;
              report_message[2] = (uint8_t)i;
              report_message[3] = value;
              if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
                rtos_fatal_error_report((char *)"Send reqport_q for scan_digital_inputs failed");
              }
            }
            vTaskDelay(1 / portTICK_PERIOD_MS);
          }
        }
      }
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
}

void scan_analog_inputs(void *parameters) {

  // report message

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = pin number
  // byte 3 = high order byte of value
  // byte 4 = low order byte of value

  int value;
  uint8_t report_message[64];
  int differential;

  while (1) {
    if (can_scan) {
      for (int i = 0; i < MAX_PINS_SUPPORTED; i++) {
        if (the_analog_pins[i].pin_mode == AT_ANALOG) {
          if (the_analog_pins[i].reporting_enabled) {
            value = analogRead(i);
            //Serial.println(value);

            differential = abs(value - the_analog_pins[i].last_value);
            if (differential >= the_analog_pins[i].differential) {
              memset(report_message, 0, sizeof(report_message));

              report_message[0] = 4;
              report_message[1] = ANALOG_REPORT;
              // trigger value achieved, send out the report
              the_analog_pins[i].last_value = value;
              // input_message[1] = the_analog_pins[i].pin_number;
              report_message[2] = (uint8_t)i;
              report_message[3] = highByte(value);  // get high order byte
              report_message[4] = lowByte(value);
              // report_message[3] = value;
              if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
                rtos_fatal_error_report((char *)"Send reqport_q for scan_analog_inputs failed");
              }
              vTaskDelay(analog_sampling_interval / portTICK_PERIOD_MS);
            }
          }
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void scan_sonars(void *parameter) {
  unsigned int distance;
  uint8_t report_message[64];

  while (1) {
    if (can_scan) {
      // Serial.print("can scan sonars");
      if (devices.sonars_index) {
        //Serial.println(devices.sonars_index);
        distance = devices.sonars[devices.last_sonar_visited].usonic->read();
        if (distance != devices.sonars[devices.last_sonar_visited].last_value) {
          devices.sonars[devices.last_sonar_visited].last_value = distance;

          // byte 0 = packet length
          // byte 1 = report type
          // byte 2 = trigger pin number
          // byte 3 = distance high order byte
          // byte 4 = distance low order byte
          memset(report_message, 0, sizeof(report_message));

          report_message[0] = 4;
          report_message[1] = SONAR_DISTANCE;
          report_message[2] = devices.sonars[devices.last_sonar_visited].trigger_pin;
          report_message[3] = (uint8_t)(distance >> 8);
          report_message[4] = (uint8_t)(distance & 0xff);
          if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
            rtos_fatal_error_report((char *)"Send reqport_q for scan_sonar failed");
          }
          vTaskDelay(sonar_scan_interval / portTICK_PERIOD_MS);
        }
        devices.last_sonar_visited++;
        if (devices.last_sonar_visited == devices.sonars_index) {
          devices.last_sonar_visited = 0;
        }
        vTaskDelay(sonar_scan_interval / portTICK_PERIOD_MS);
      }
    }
    vTaskDelay(sonar_scan_interval / portTICK_PERIOD_MS);
  }
}

void scan_dhts(void *parameter) {
  // prebuild report for valid data
  // reuse the report if a read command fails

  // data returned is in floating point form - 4 bytes
  // each for humidity and temperature

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = report sub type - DHT_DATA or DHT_ERROR
  // byte 3 = pin number
  // byte 4 = humidity high order byte for data or error value
  // byte 5 = humidity byte 2
  // byte 6 = humidity byte 3
  // byte 7 = humidity byte 4
  // byte 8 = temperature high order byte for data or
  // byte 9 = temperature byte 2
  // byte 10 = temperature byte 3
  // byte 11 = temperature byte 4

  uint8_t report_message[64];

  uint8_t d_read;

  float dht_data;

  while (1) {
    if (can_scan) {
      // are there any dhts to read?
      //Serial.println(devices.dht_index);
      if (devices.dht_index) {
        // is it time to do the read? This should occur every 2 seconds

        // read and report all the dht sensors
        for (int i = 0; i < devices.dht_index; i++) {
          memset(report_message, 0, sizeof(report_message));
          report_message[0] = 11;
          report_message[1] = DHT_REPORT;
          report_message[2] = DHT_DATA;
          report_message[3] = devices.dhts[i].pin;
          // get humidity
          dht_data = devices.dhts[i].dht_sensor->getHumidity();
          memcpy(&report_message[4], &dht_data, sizeof dht_data);

          // get temperature
          dht_data = devices.dhts[i].dht_sensor->getTemperature();
          memcpy(&report_message[8], &dht_data, sizeof dht_data);

          if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
            rtos_fatal_error_report((char *)"Send reqport_q for scan_dhts failed");
          }

          // now read do a read for this device for next go around
          d_read = devices.dhts[i].dht_sensor->read();

          if (d_read) {
            // error found
            // send report
            report_message[0] = 4;
            report_message[1] = DHT_REPORT;
            report_message[2] = DHT_READ_ERROR;
            report_message[3] = devices.dhts[i].pin;  // pin number
            report_message[4] = d_read;

            if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
              rtos_fatal_error_report((char *)"Send reqport_q for scan_dhts(2) failed");
            }
            vTaskDelay(dht_scan_interval / portTICK_PERIOD_MS);
          }
        }
      }
      vTaskDelay(dht_scan_interval / portTICK_PERIOD_MS);
    }
  }
}

void scan_touch(void *parameter) {
  int value;

  // report message

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = pin number
  // byte 3 = high order byte of value
  // byte 4 = low order byte of value

  uint8_t report_message[64];

  while (1) {
    if (can_scan) {
      memset(report_message, 0, sizeof(report_message));
      report_message[0] = 4;
      report_message[1] = TOUCH_REPORT;

      int differential;

      for (int i = 0; i < MAX_PINS_SUPPORTED; i++) {
        if (the_touch_pins[i].reporting_enabled) {
          value = touchRead(i);
          Serial.println(value);

          differential = abs(value - the_touch_pins[i].last_value);
          if (differential >= the_touch_pins[i].differential) {
            // trigger value achieved, send out the report
            the_touch_pins[i].last_value = value;
            // input_message[1] = the_analog_pins[i].pin_number;
            report_message[2] = (uint8_t)i;
            report_message[3] = highByte(value);  // get high order byte
            report_message[4] = lowByte(value);

            if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
              rtos_fatal_error_report((char *)"Send reqport_q for scan_touch failed");
            }
          }
        }
      }
      vTaskDelay(touch_sampling_interval / portTICK_PERIOD_MS);
    }
  }
}

void reset_data() {
  WiFi.disconnect();
  ESP.restart();
}

void init_pin_structures() {
  for (uint8_t i = 0; i < MAX_PINS_SUPPORTED; i++) {
    the_digital_pins[i].pin_number = i;
    the_digital_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_digital_pins[i].reporting_enabled = false;
    the_digital_pins[i].last_value = 0;

    the_analog_pins[i].pin_number = i;
    the_analog_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_analog_pins[i].reporting_enabled = false;
    the_analog_pins[i].last_value = 0;
    the_analog_pins[i].differential = 0;

    the_touch_pins[i].reporting_enabled = false;
    the_touch_pins[i].last_value = 0;
    the_touch_pins[i].differential = 0;
  }

  // establish the analog pin array
  for (uint8_t i = 0; i < MAX_PINS_SUPPORTED; i++) {
    the_analog_pins[i].pin_number = i;
    the_analog_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_analog_pins[i].reporting_enabled = false;
    the_analog_pins[i].last_value = 0;
    the_analog_pins[i].differential = 0;
  }
}

void run_steppers(void *parameter) {
  boolean running;
  long current_position;
  long target_position;
  uint8_t report_message[64];

  while (1) {
    if (devices.ok_to_run_motors) {
      for (int i = 0; i < devices.steppers_index; i++) {
        if (devices.stepper_run_modes[i] == STEPPER_STOP) {
          continue;
        } else {
          devices.steppers[i]->enableOutputs();
          switch (devices.stepper_run_modes[i]) {
            case STEPPER_RUN:
              devices.steppers[i]->run();
              running = devices.steppers[i]->isRunning();
              if (!running) {
                memset(report_message, 0, sizeof(report_message));
                report_message[0] = 2;
                report_message[1] = STEPPER_RUN_COMPLETE_REPORT;
                report_message[2] = (uint8_t)i;

                if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
                  rtos_fatal_error_report((char *)"Send reqport_q for stepper run complete failed");
                }
                devices.stepper_run_modes[i] = STEPPER_STOP;
              }
              break;
            case STEPPER_RUN_SPEED:
              devices.steppers[i]->runSpeed();
              break;
            case STEPPER_RUN_SPEED_TO_POSITION:
              running = devices.steppers[i]->runSpeedToPosition();
              target_position = devices.steppers[i]->targetPosition();
              if (target_position == devices.steppers[i]->currentPosition()) {
                memset(report_message, 0, sizeof(report_message));

                uint8_t report_message[3] = { 2, STEPPER_RUN_COMPLETE_REPORT, (uint8_t)i };
                report_message[0] = 2;
                report_message[1] = STEPPER_RUN_COMPLETE_REPORT;
                report_message[2] = (uint8_t)i;

                if (xQueueSend(report_q, (const void *)report_message, 20) != pdTRUE) {
                  rtos_fatal_error_report((char *)"Send reqport_q for stepper run complete failed");
                }
                devices.stepper_run_modes[i] = STEPPER_STOP;
              }
              break;
            default:
              vTaskDelay(1 / portTICK_PERIOD_MS);
              break;
          }
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// print the error and then blink board led forever
void rtos_fatal_error_report(char *report) {
  Serial.println(report);
  while (1) {
    digitalWrite(BUILTIN_LED, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(BUILTIN_LED, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// rtos tasks
void get_next_command(void *parameter) {
  command_descriptor command_entry;
  uint8_t command, length;

  memset(command_buffer, 0, sizeof(command_buffer));

  // get the next entry on commmand_q
  while (1) {
    if (deviceConnected == true) {
      if (xQueueReceive(command_q, command_buffer, 10) == pdTRUE) {
        Serial.println("Before");
        //dump_buffer(command_buffer, 32);
        length = command_buffer[0];
        command = command_buffer[1];


        if (length > 1) {
          for (int i = 0; i < length - 1; i++) {
            command_buffer[i] = command_buffer[i + 2];
          }
        }
        // Serial.println("after");

        //dump_buffer(command_buffer, 32);

        // get function pointer to command
        command_entry = command_table[command];

        //  execute the command
        command_entry.command_func();
      }
    } else {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    memset(command_buffer, 0, sizeof(command_buffer));
  }
}

void send_report(void *parameter) {
  while (1) {
    if (deviceConnected) {

      // byte bx[] = {1, 2, 3, 4, 0, 0, 0};
      uint8_t xreport_buffer[64];
      memset(xreport_buffer, 0, sizeof(xreport_buffer));

      if (xQueueReceive(report_q, xreport_buffer, 10) == pdTRUE) {

        client.write(xreport_buffer, xreport_buffer[0] + 1);

        vTaskDelay(1 / portTICK_PERIOD_MS);
      } else {
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
    }

    else {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
}

void retrieve_data_from_network(void *parameters) {
  memset(command_buffer, 0, sizeof(command_buffer));
  byte packet_length;

  while (1) {
    client = wifiServer.available();

    if (client) {
      Serial.print("Client Connected to address: ");
      Serial.println(client.remoteIP());

      while (client.connected()) {
        if (not client.available()) {
          vTaskDelay(1 / portTICK_PERIOD_MS);
          continue;
        }
        // get the packet length
        packet_length = (byte)client.read();

        command_buffer[0] = packet_length;
        for (int i = 1; i <= packet_length; i++) {
          // need this delay or data read is not correct
          while (not client.available()) {
            vTaskDelay(1 / portTICK_PERIOD_MS);
          }
          command_buffer[i] = (byte)client.read();
        }

        //dump_buffer(command_q_buffer, 32);

        if (xQueueSend(command_q, (const void *)command_buffer, 2000) != pdTRUE) {
          rtos_fatal_error_report((char *)"Send on command_q failed");
        }
        memset(command_buffer, 0, sizeof(command_buffer));
      }
    }
  }
}

void myTimerCallback(TimerHandle_t xTimer) {
  Serial.println("Could not connect - resetting the device");
  reset_data();
}

void setup() {
  Serial.begin(115200);

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  delay(100);

  pinMode(LED_BUILTIN, OUTPUT);

  // turn on LED
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("\nAllow 15 seconds for connection to complete..");

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println();

  Serial.print("Connected to WiFi. IP Address: ");
  Serial.print(WiFi.localIP());

  Serial.print("  IP Port: ");
  Serial.println(PORT);
  Serial.println();

  digitalWrite(LED_BUILTIN, LOW);
  deviceConnected = true;
  wifiServer.begin();
  delay(1000);



  can_scan = true;
  xTaskCreatePinnedToCore(retrieve_data_from_network, "retrieve_data_from_network", 2048, NULL, 2, NULL, app_cpu);
  xTaskCreatePinnedToCore(get_next_command, "get next command", 2048, NULL, 5, NULL, app_cpu);
  xTaskCreatePinnedToCore(send_report, "send report", 2048, NULL, 2, NULL, app_cpu);
  xTaskCreatePinnedToCore(scan_digital_inputs, "scan_digital_inputs", 2048, NULL, 2, NULL, app_cpu);
  xTaskCreatePinnedToCore(scan_analog_inputs, "scan_analog_inputs", 2048, NULL, 2, NULL, app_cpu);
  xTaskCreatePinnedToCore(scan_touch, "scan_touch", 2048, NULL, 4, NULL, app_cpu);
  xTaskCreatePinnedToCore(scan_sonars, "scan_sonars", 2048, NULL, 2, NULL, app_cpu);
  xTaskCreatePinnedToCore(scan_dhts, "scan_dhts", 2048, NULL, 5, NULL, app_cpu);
  xTaskCreatePinnedToCore(run_steppers, "run_steppers", 2048, NULL, 2, NULL, app_cpu);

  //delay(1000);

  vTaskDelete(NULL);
}

void loop() {
  // Nothing to do here
}
