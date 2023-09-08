
//////////////SETTINGS/////////////////////
#define TRANSPORT_SERIAL // remove to use wifi - also uncomment board_microros_transport from ini file
#define SPI
// #define I2C

#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <Wire.h>

#include <std_msgs/msg/int32.h>

#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include <Adafruit_NeoPixel.h>

#ifdef TRANSPORT_SERIAL
#else
#include <WiFi.h>
#endif

// LED
uint32_t disconnected_led_color = 0xFFFF00;
int led_brightness = 20;
#define NUMPIXELS 1
#define LED_PIN 38
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRBW + NEO_KHZ800);

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      return false;              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t mag_publisher;
sensor_msgs__msg__MagneticField mag_msg;
bool micro_ros_init_successful;

#ifdef SPI
SPIClass *vspi = NULL;
#define VSPI FSPI
#endif

#define VSPI_MISO_SDA 1
#define VSPI_SCLK_SCL 13
#define VSPI_MOSI_I2C_ADD 14
#define CS 11
#define INT 2
#define BNO08X_RESET 12
#define BNO08X_PS0 42
#define BNO08X_PS1 41

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

bool new_imu_data = false;

byte tot_rep = 0;
// internal copies of the IMU data
float ax, ay, az, gx, gy, gz, qx, qy, qz, qw, mx, my, mz; // (qx, qy, qz, qw = i,j,k, real)
byte linAccuracy = 0;
byte gyroAccuracy = 0;
byte magAccuracy = 0;
float quatRadianAccuracy = 0;
byte quatAccuracy = 0;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer != NULL)
  {
    rcl_publish(&publisher, &msg, NULL);
  }
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "uROS_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msg_Int32"));
  RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "uROS_BNO086"));
  RCCHECK(rclc_publisher_init_default(&mag_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "uROS_BNO086_mag"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&mag_publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void error_loop()
{
  pixels.clear();
  pixels.show();
  while (1)
  {
    pixels.setPixelColor(0, 0xFF0000);
    pixels.show();
    delay(500);
    pixels.setPixelColor(0, 0x000000);
    pixels.show();
    delay(500);
  }
}

void ARDUINO_ISR_ATTR isr()
{
  new_imu_data = true;
}

void setup()
{
  delay(1000);
  Serial.begin(115200);
  delay(200);
  Serial.println("setup..");

  /*PS1 PS0 MODE
    0   0   I2C
    0   1   UART-RVC
    1   0   UART
    1   1   SPI*/
  pinMode(BNO08X_PS1, OUTPUT);
  pinMode(BNO08X_PS0, OUTPUT);
#ifdef SPI
  digitalWrite(BNO08X_PS1, HIGH);
  digitalWrite(BNO08X_PS0, HIGH);
#endif
#ifdef I2C
  digitalWrite(BNO08X_PS1, LOW);
  digitalWrite(BNO08X_PS0, LOW);
  // with i2c define address 04B or 04A with mosi pin/ pull down present (04A)
  pinMode(VSPI_MOSI_I2C_ADD, OUTPUT);
  digitalWrite(VSPI_MOSI_I2C_ADD, HIGH);
#endif

  pinMode(INT, INPUT_PULLUP);
  attachInterrupt(INT, isr, FALLING);
  delay(1000);

  // BNO086 hardware reset after settings
  pinMode(BNO08X_RESET, OUTPUT);
  digitalWrite(BNO08X_RESET, HIGH);
  delay(10);
  digitalWrite(BNO08X_RESET, LOW);
  delay(10);
  digitalWrite(BNO08X_RESET, HIGH);
  delay(10);

  delay(100);
  pixels.begin();
  pixels.clear();
  pixels.setBrightness(led_brightness);
  for (size_t i = 0; i < 5; i++)
  {
    pixels.setPixelColor(0, 0xAA00FF);
    pixels.show();
    delay(500);
    pixels.setPixelColor(0, 0x000000);
    pixels.show();
    delay(500);
  }
  pixels.clear();
  pixels.show();

  state = WAITING_AGENT;

#ifdef TRANSPORT_SERIAL
  set_microros_serial_transports(Serial);
#else
  IPAddress agent_ip(192, 168, 1, 114);
  size_t agent_port = 12345;

  char ssid[] = "SSID";
  char psk[] = "WIFI PASS";
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
#endif

#ifdef I2C
  Wire.setPins(VSPI_MISO_SDA, VSPI_SCLK_SCL);
  // Try to initialize!
  if (!bno08x.begin_I2C(0x4B))
  {
    Wire.end();
    delay(200);
    if (!bno08x.begin_I2C(0x4A))
    {
      // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
      // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
      Serial.println("Failed to find BNO08x chip");
      error_loop();
    }
  }
#endif
#ifdef SPI
  vspi = new SPIClass(VSPI);
  vspi->setFrequency(40000000);
  vspi->begin(VSPI_SCLK_SCL, VSPI_MISO_SDA, VSPI_MOSI_I2C_ADD, CS); // SCLK, MISO, MOSI, SS
  pinMode(vspi->pinSS(), OUTPUT);                               // VSPI SS
  if (!bno08x.begin_SPI(CS, INT, vspi, 0))
    while (1)
    {
      Serial.println("Failed to find BNO08x chip");
      error_loop();
    }
#endif
  Serial.println("BNO08x Found!");
  for (int n = 0; n < bno08x.prodIds.numEntries; n++)
  {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  delay(10);
  bno08x.enableReport(SH2_ACCELEROMETER, 9000);
  delay(10);
  bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 9000);
  delay(10);
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 9000);
  delay(10);
  bno08x.enableReport(SH2_ROTATION_VECTOR, 9000);

  msg.data = 0;

  ////NEW NEW
  micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), &imu_msg, (micro_ros_utilities_memory_conf_t){});
  imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu");

  micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), &mag_msg, (micro_ros_utilities_memory_conf_t){});
  mag_msg.header.frame_id = micro_ros_string_utilities_set(mag_msg.header.frame_id, "imu");

  imu_msg.angular_velocity.x = 0;
  imu_msg.angular_velocity.y = 0;
  imu_msg.angular_velocity.z = 0;
  imu_msg.linear_acceleration.x = 0;
  imu_msg.linear_acceleration.y = 0;
  imu_msg.linear_acceleration.z = 0;
  imu_msg.orientation.w = 0;
  imu_msg.orientation.x = 0;
  imu_msg.orientation.y = 0;
  imu_msg.orientation.z = 0;
  mag_msg.magnetic_field.x = 0;
  mag_msg.magnetic_field.y = 0;
  mag_msg.magnetic_field.z = 0;
  for (size_t i = 0; i < 9; i++)
    imu_msg.orientation_covariance[i] = -1;
  for (size_t i = 0; i < 9; i++)
    imu_msg.linear_acceleration_covariance[i] = -1;
  for (size_t i = 0; i < 9; i++)
    imu_msg.angular_velocity_covariance[i] = -1;
  for (size_t i = 0; i < 9; i++)
    mag_msg.magnetic_field_covariance[i] = -1;
}

uint32_t color = 0x000000;
uint32_t current_color = 0x000000;
int bno_reset_count = 0;

void loop()
{

  if (bno08x.wasReset())
  {
    delay(10);
    bno08x.enableReport(SH2_ACCELEROMETER, 9000);
    delay(10);
    bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 9000);
    delay(10);
    bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 9000);
    delay(10);
    bno08x.enableReport(SH2_ROTATION_VECTOR, 9000);
    bno_reset_count++;
  }

  if (!bno08x.getSensorEvent(&sensorValue))
  {
    return;
  }
  else
  {

    switch (sensorValue.sensorId)
    {
    case SH2_ACCELEROMETER:
      imu_msg.linear_acceleration.x = sensorValue.un.accelerometer.x;
      imu_msg.linear_acceleration.y = sensorValue.un.accelerometer.y;
      imu_msg.linear_acceleration.z = sensorValue.un.accelerometer.z;
      linAccuracy = sensorValue.status;
      tot_rep++;
      break;
    case SH2_GYROSCOPE_UNCALIBRATED:
      imu_msg.angular_velocity.x = sensorValue.un.gyroscopeUncal.x;
      imu_msg.angular_velocity.y = sensorValue.un.gyroscopeUncal.y;
      imu_msg.angular_velocity.z = sensorValue.un.gyroscopeUncal.z;
      gyroAccuracy = sensorValue.status;
      tot_rep++;
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      mag_msg.magnetic_field.x = sensorValue.un.magneticField.x;
      mag_msg.magnetic_field.y = sensorValue.un.magneticField.y;
      mag_msg.magnetic_field.z = sensorValue.un.magneticField.z;
      break;
    case SH2_ROTATION_VECTOR:
      imu_msg.orientation.w = sensorValue.un.rotationVector.real;
      imu_msg.orientation.x = sensorValue.un.rotationVector.i;
      imu_msg.orientation.y = sensorValue.un.rotationVector.j;
      imu_msg.orientation.z = sensorValue.un.rotationVector.k;
      quatAccuracy = sensorValue.un.rotationVector.accuracy;
      tot_rep++;
      break;
    default:
      break;
    }

    new_imu_data = false;
    msg.data = (1 + bno_reset_count * 1000 + 1 + gyroAccuracy * 100 + 1 + linAccuracy * 10 + 1 + quatAccuracy);
  }

  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    color = 0xFFDD00;
    break;
  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    color = 0xFFDD00;
    if (state == WAITING_AGENT)
    {
      destroy_entities();
    };
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      color = 0x00FF00;
    }
    break;
  case AGENT_DISCONNECTED:
    color = 0xFFDD00;
    destroy_entities();
    state = WAITING_AGENT;
    break;
  default:
    break;
  }

  if (state == AGENT_CONNECTED)
  {
    if (tot_rep >= 3)
    {
      tot_rep = 0;
      RCSOFTCHECK(rmw_uros_sync_session(1000));
      imu_msg.header.stamp.sec = rmw_uros_epoch_millis();
      imu_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
      mag_msg.header.stamp.sec = rmw_uros_epoch_millis();
      mag_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
      RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
      RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
    }
  }

  if (current_color != color)
  {
    current_color = color;
    pixels.setPixelColor(0, current_color);
    pixels.show();
  }
}
