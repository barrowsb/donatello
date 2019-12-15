/*
 * DemoBot code
 */

// ========================= includes =============================
// add any libraries here


// =================================================================
// ========================= I2C start =============================
// =================================================================
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <WiFi.h>
#include <WiFiUDP.h>

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 128                             // data buffer length of test buffer
#define W_LENGTH 1                                  // data length for w, [0,DATA_LENGTH]
#define R_LENGTH 16                                 // data length for r, [0,DATA_LENGTH]

#define I2C_MASTER_SCL_IO (gpio_num_t)33            // gpio number for I2C master clock
#define I2C_MASTER_SDA_IO (gpio_num_t)25            // gpio number for I2C master data
#define I2C_MASTER_NUM I2C_NUMBER(1)                // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 40000                    // I2C master clock frequency (Hz)
#define I2C_MASTER_TX_BUF_DISABLE 0                 // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0                 // I2C master doesn't need buffer

#define CONFIG_I2C_SLAVE_ADDRESS 0x28
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS     // ESP32 slave address, you can set any 7bit value
#define WRITE_BIT I2C_MASTER_WRITE                  // I2C master write
#define READ_BIT I2C_MASTER_READ                    // I2C master read
#define ACK_CHECK_EN 0x1                            // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0                           // I2C master will not check ack from slave
#define ACK_VAL I2C_MASTER_ACK                      // I2C ack value
#define NACK_VAL I2C_MASTER_NACK                    // I2C nack value

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t nsize)
{
    if (nsize == 0) 
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN); 
    if (nsize > 1) 
    {
        i2c_master_read(cmd, data_rd, nsize - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + nsize - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS); // send all queued commands
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t nsize)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, nsize, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) 
    {
        Serial.printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) 
        {
            Serial.printf("\n");
        }
    }
    Serial.printf("\n");
}

uint8_t data_wr[DATA_LENGTH];
uint8_t data_rd[DATA_LENGTH];

static void i2c_read_test()
{
    int ret;

    ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, DATA_LENGTH);

    if (ret == ESP_ERR_TIMEOUT) 
    {
        ESP_LOGE(TAG, "I2C Timeout");
        Serial.println("I2C Timeout");
    } 
    else if (ret == ESP_OK) 
    {
        // uncomment the following 2 lines if you want to display information read from I2C
        //Serial.printf(" MASTER READ FROM SLAVE ******\n");
        //disp_buf(data_rd, DATA_LENGTH);
    } 
    else 
    {
        ESP_LOGW(TAG, " %s: Master read slave error, IO not connected...\n",
            esp_err_to_name(ret));
    }
}

static void i2c_write_test()
{ 
    int ret;
                                                                             
    ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, W_LENGTH);
    if (ret == ESP_ERR_TIMEOUT) 
    {
        ESP_LOGE(TAG, "I2C Timeout");
    } 
    else if (ret == ESP_OK) 
    {
        // uncomment the following 2 lines if you want to display information being send over I2C
        //Serial.printf(" MASTER WRITE TO SLAVE\n");
        //disp_buf(data_wr, W_LENGTH);
    } 
    else 
    {
        ESP_LOGW(TAG, "%s: Master write slave error, IO not connected....\n",
            esp_err_to_name(ret));
    }
}
// =================================================================
// ========================== I2C end ==============================
// =================================================================


// =================================================================
// ====================== Interrupt start ==========================
// =================================================================
// Timer + Interrupt for reading I2C
hw_timer_t* timer = NULL;                               // initialize a timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;   // needed to sync between main loop and ISR when modifying shared variable
volatile bool readI2C = 0;                              // should we read data from I2C?

void IRAM_ATTR readI2COnTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    readI2C = 1;                        // need to read I2C next loop
    portEXIT_CRITICAL_ISR(&timerMux);
}
// =================================================================
// ======================= Interrupt end ===========================
// =================================================================


// =================================================================
// ========================= LED start =============================
// =================================================================
#include "FastLED.h"
FASTLED_USING_NAMESPACE

#define RED             0xFF0000    // color for the red team
#define BLUE            0x0000FF    // color for the blue team
#define HEALTHCOLOR     0x00FF00    // color for the health LEDs   

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

int MAX_HEALTH = -1;
int RESPAWN_TIME = 15;

// ===== GAME VARIABLES =====
// change ROBOTNUM (1-4) and TEAMCOLOR (BLUE or RED) as necessary
#define ROBOTNUM    1               // robot number on meta team (1-4)
#define TEAMCOLOR   BLUE            // color for the robot team, either RED or BLUE
// ==========================

#define NEO_LED_PIN 12              // pin attached to LED ring
#define LED_TYPE    WS2812          // APA102
#define COLOR_ORDER GRB             // changes the order so we can use standard RGB for the values
#define NUM_LEDS    24              // number of LEDs in the ring
CRGB leds[NUM_LEDS];                // set value of LED, each LED is 24 bits

#define BRIGHTNESS          60      // lower the brightness a bit

// core to run FastLED.show()
#define FASTLED_SHOW_CORE 0

// task handles for use in the notifications
static TaskHandle_t FastLEDshowTaskHandle = 0;
static TaskHandle_t userTaskHandle = 0;

/** show() for ESP32
 *  Call this function instead of FastLED.show(). It signals core 0 to issue a show, 
 *  then waits for a notification that it is done.
 */
void FastLEDshowESP32()
{
    if (userTaskHandle == 0) 
    {
        // -- Store the handle of the current task, so that the show task can
        //    notify it when it's done
        userTaskHandle = xTaskGetCurrentTaskHandle();

        // -- Trigger the show task
        xTaskNotifyGive(FastLEDshowTaskHandle);

        // -- Wait to be notified that it's done
        const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );
        ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
        userTaskHandle = 0;
    }
}

/** show Task
 *  This function runs on core 0 and just waits for requests to call FastLED.show()
 */
void FastLEDshowTask(void *pvParameters)
{
    // -- Run forever...
    for(;;) 
    {
        // -- Wait for the trigger
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // -- Do the show (synchronously)
        FastLED.show();

        // -- Notify the calling task
        xTaskNotifyGive(userTaskHandle);
    }
}

void SetupFastLED(void)
{
    // tell FastLED about the LED strip configuration
    FastLED.addLeds<LED_TYPE,NEO_LED_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

    // set master brightness control
    FastLED.setBrightness(BRIGHTNESS);

    int core = xPortGetCoreID();
    Serial.print("FastLED: Main code running on core ");
    Serial.println(core);

    // -- Create the FastLED show task
    xTaskCreatePinnedToCore(FastLEDshowTask, "FastLEDshowTask", 2048, NULL, 2, &FastLEDshowTaskHandle, FASTLED_SHOW_CORE);
}

void ShowRobotNum(void)
{
    int robotLeds[] = {0,6,12,18};      // location of the LEDs used to display the robot number

    // change the LEDs based on the robot number
    leds[robotLeds[0]] = TEAMCOLOR;     // The first LED is always displayed with the robot number

    switch (ROBOTNUM)
    {
        case 1:
            leds[robotLeds[1]] = 0;
            leds[robotLeds[2]] = 0;
            leds[robotLeds[3]] = 0;
            break;
        case 2:
            leds[robotLeds[1]] = 0;
            leds[robotLeds[2]] = TEAMCOLOR;
            leds[robotLeds[3]] = 0;
            break;
        case 3:
            leds[robotLeds[1]] = TEAMCOLOR;
            leds[robotLeds[2]] = 0;
            leds[robotLeds[3]] = TEAMCOLOR;
            break;
        case 4:
            leds[robotLeds[1]] = TEAMCOLOR;
            leds[robotLeds[2]] = TEAMCOLOR;
            leds[robotLeds[3]] = TEAMCOLOR;
            break;
    }
}

void ShowHealth(int health)
{   
    int health_leds[] = {1,2,3,4,5,7,8,9,10,11,13,14,15,16,17,19,20,21,22,23};  //LED number for health LEDS
    int num_health_leds = 20; //number of LEDS for health
    
    if(MAX_HEALTH == -1 && health > 0)  //max health set on first iteration
      MAX_HEALTH = health;            
      
    //float health_ratio = health/(MAX_HEALTH * 1.0);
    //int num_health_led_on = health_ratio * num_health_leds;

    int num_health_led_on = map(health,0,MAX_HEALTH,0,num_health_leds);

   
    
    for(int i=0; i<num_health_leds; i++)
    {
        if( i < num_health_led_on)
          leds[health_leds[i]] = HEALTHCOLOR; 
        else
          leds[health_leds[i]] = 0;
    }
    
//    for(int i = 0; i<24 ;i++)
//      leds[i]= RED;
}

void clearLEDs(void)
{
    for(int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = 0;
    }
}

void ShowRespawnTimer(int respawnTime)
{
    int respawn_leds = map(respawnTime, 0, RESPAWN_TIME, 0, NUM_LEDS);
    
    for(int i=0; i<NUM_LEDS; i++)
     {
      if(i<respawn_leds)
        leds[i] = RED;
      else
        leds[i] = 0;
      }
}
// =================================================================
// ========================== LED end ==============================
// =================================================================

// =================================================================
// ========================== Motor start ==========================
// =================================================================

// servo

int servo_pin = 19;
int servo_channel = 4;
int servo_pwm_freq = 50;
int servo_pwm_resolution = 16;


int servo_low  = 1600;
int servo_high = 8100;
int servo_direction = 0;
int servo_position = servo_high;
int servo_speed = 500;

// motor A
int mA1_pin = 22;
int mA2_pin = 23;
int mA1_channel = 2;
int mA2_channel = 3;

// motor B
int mB1_pin = 27;
int mB2_pin = 26 ;
int mB1_channel = 0;
int mB2_channel = 1;

// pwm parameters
int pwm_freq = 500;
int pwm_resolution = 6;

// control left motor
void control_left_motor(int spood, int dir)
{
  if(dir == 1)
  {
    ledcWrite(mA2_channel, 0);
    ledcWrite(mA1_channel, spood);
    
    }
   else if(dir == -1)
   {
    ledcWrite(mA1_channel, 0);
    ledcWrite(mA2_channel, spood);
    }
} 

void control_right_motor(int spood, int dir)
{
  if(dir == 1)
  {
    ledcWrite(mB2_channel, 0);
    ledcWrite(mB1_channel, spood);
    
    }
   else if(dir == -1)
   {
    ledcWrite(mB1_channel, 0);
    ledcWrite(mB2_channel, spood);
    }
}  

// control motors with command
void control_motors(int udp_data)
{
  // variable for adjusted motor command
  int d;

  // if command for right motor
  if(udp_data > 127)
  { 
    // subtract off first bit (which controls motor selection)
    d = udp_data - 128;
    
    // print right
    Serial.print("Right\t");

    // if command for forward rotation
    if(d > 63)
    {
      // subtract off second bit (which controls motor direction)
      d = d - 63;
      //if 64 serco stop command!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      if(d==64){
        ServoStop();
        Serial.println(udp_data);
        return;
      }

      // print motor command
      Serial.print(d);
      Serial.print("\t forward\t");

      // control right motor in forward direction
      control_right_motor(d,1);       
    }
    
    // if command for backward rotation
    else
    {
      //if 63 then Right servo command!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      if(d==63){
        ServoRight();
        Serial.println(udp_data);
        
        return; 
      }

      //
      
      
      // print motor command
      Serial.print(d);
      Serial.print("\t backward\t");

      // control right motor in backward direction
      control_right_motor(d,-1);        
    }
  }

  // if command for left motor
  else
  {
    d = udp_data;
    //if 127 then servo stop !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if(d==127){
      ServoStop();
      Serial.println(udp_data);

      return;
    }
    // first bit is zero, assign upd_data to d
    

    // print left
    Serial.print("Left\t");

    // if command for forward rotation
    if(d > 63)
    {
      // subtract off second bit (which controls motor direction)
      d = d - 63;

      // print motor command
      Serial.print(d);
      Serial.println("\t forward\t");

      // control left motor in forward direction
      control_left_motor(d,1);       
    }

    // if command for backward direction
    else
    {
      //if 63 then Left servo command !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      if(d == 63){
        ServoLeft();
        Serial.println(udp_data);

        return;
      }

      // print motor command
      Serial.print(d);
      Serial.println("\t backward\t");

      // control left motor in backward direction
      control_left_motor(d,-1);      
    }
  }                     
}

void ServoLeft(){
  Serial.println("Servo Left Command");
  servo_direction = 1;
}

void ServoRight(){
  Serial.println("Servo Right Command");
  servo_direction = -1;
  
}

void ServoStop(){
  Serial.println("Servo Stop Command");
  servo_direction = 0;
}

void update_servo(){
  if(servo_direction == 1){
      servo_position = min(servo_high, servo_direction*servo_speed + servo_position);
  }
  else if(servo_direction == -1){
      servo_position = max(servo_low, servo_direction*servo_speed + servo_position);
  }
  ledcWrite(servo_channel, servo_position);
  delay(5);
  
  Serial.printf("Servo dir:");
  Serial.print(servo_direction);
  
  Serial.printf("Servo pos:");
  Serial.print(servo_position);
  
  
}

// =================================================================
// ========================== Motor end ============================
// =================================================================


// =================================================================
// ========================== Auto Start ============================
// =================================================================

int RXD2  = 16;
int TXD2 = 17;

int X_VAL_L; //from timer 3
int Y_VAL_L; //from timer 3
int X_VAL_R; //from timer 1
int Y_VAL_R; //from timer 1

int TARGET_X = 3850;
int TARGET_Y = 4500;

int success_area = 50;


float error_angle;
int heading_vec_x;
int heading_vec_y;

int motor_speed = 12;

float motor_p = 3;

byte temp_read[3];



void Read_Serial(){
  int count = 0;
  while(Serial2.available()){
    
    count++;
    add_to_temp(Serial2.read());
    if(count > 12) return;
  }


  
}

void add_to_temp(byte in){
  temp_read[2] = temp_read[1];
  temp_read[1] = temp_read[0];
  temp_read[0] = in;

  if(temp_read[0] == '!'){
    int out = temp_read[2]|(temp_read[1] << 8);
    Y_VAL_L = out;
  }
  else if(temp_read[0] == '@'){
    int out = temp_read[2]|(temp_read[1] << 8);
    X_VAL_L = out;
  }
  else if(temp_read[0] == '#'){
    int out = temp_read[2]|(temp_read[1] << 8);
    Y_VAL_R = out;
  }
  else if(temp_read[0] == '$'){
    int out = temp_read[2]|(temp_read[1] << 8);
    X_VAL_R = out;
  }
  else if(temp_read[0] == '~'){
    //calculate values
    update_auto_vals();
  }
  
}

void update_auto_vals(){
  int dif_vec_x = X_VAL_L - X_VAL_R;
  int dif_vec_y = Y_VAL_L - Y_VAL_R;

  int mid_vec_x = (X_VAL_L + X_VAL_R)/2;
  int mid_vec_y = (Y_VAL_L + Y_VAL_R)/2;

  heading_vec_x = TARGET_X - mid_vec_x;
  heading_vec_y = TARGET_Y - mid_vec_y;

  float target_relative_angle = atan2(heading_vec_y, heading_vec_x);
  float field_relative_angle = atan2(dif_vec_y,dif_vec_x) + PI/2;

  error_angle = target_relative_angle - field_relative_angle;
  if (field_relative_angle < 0){
    field_relative_angle+=2*PI;
  }
  if(target_relative_angle < 0){
    target_relative_angle +=2*PI;
  }
  if(error_angle < -PI){
    error_angle+=2*PI;
  }

  
  Serial.print("X Mid: ");
  Serial.println(mid_vec_x);
  Serial.print("Y Mid: ");
  Serial.println(mid_vec_y);

  Serial.print("target relative angle: ");
  Serial.println(target_relative_angle);
  Serial.print("field relative angle: ");
  Serial.println(field_relative_angle);
  Serial.print("error angle: ");
  Serial.println(error_angle);

  
  
}

void controlMotors(){
  
  int left_speed;
  int right_speed;
  if(check_for_goal()){
    Serial.print("Right Motor :");
    Serial.println(0);
    right_speed = 0;
    Serial.print("Left Motor :");
    Serial.println(0);
    left_speed = 0;
  }
  else if(error_angle < -PI/4){
    Serial.print("Right Motor :");
    Serial.println(-motor_speed - (PI/4)*motor_p);
    right_speed =-motor_speed - (PI/4)*motor_p;
    Serial.print("Left Motor :");
    Serial.println(motor_speed + (PI/4)*motor_p);
    left_speed = motor_speed + (PI/4)*motor_p;
  }
  else if(error_angle > PI/4){
    Serial.print("Right Motor :");
    Serial.println(motor_speed + (PI/4)*motor_p);
    right_speed = motor_speed + (PI/4)*motor_p;
    Serial.print("Left Motor :");
    Serial.println(-motor_speed - (PI/4)*motor_p);
    left_speed = -motor_speed - (PI/4)*motor_p;
  }
  else{
    Serial.print("Right Motor :");
    Serial.println(motor_speed + error_angle*motor_p);
    right_speed = motor_speed + error_angle*motor_p;
    Serial.print("Left Motor :");
    Serial.println(motor_speed - error_angle*motor_p);
    left_speed = motor_speed - error_angle*motor_p;
  }
  if(right_speed >= 0){
    control_right_motor(right_speed, 1);
  }
  else{
    control_right_motor(-right_speed, -1);
  }
  if(left_speed >= 0){
    control_right_motor(left_speed, 1);
  }
  else{
    control_left_motor(-left_speed, -1);
  }
}

float dot_prod(int x1, int y1, int x2, int y2){
  return (x1*x2+y1*y2)/(mag(x1,y1)*mag(x2,y2));
}

float mag(int x, int y){
  return pow(pow(x,2)+pow(y,2),0.5);
}

bool check_for_goal(){
  if(mag(heading_vec_x,heading_vec_y) < success_area){
    Serial.println("Goal Reached");
    return true;
  }
  return false;
}



// =================================================================
// ========================== Auto end ============================
// =================================================================




// =================================================================
// ========================== WiFi start ============================
// =================================================================

// WiFi name
const char* ssid = "Vishnu";
const char* pass = "12345678";

// IP Addresses
IPAddress myIP(192,168,4,113);
IPAddress targetIP(192,168,4,1);

// variables for UDP
WiFiUDP udp;                        // rename reference to WiFiUDP header file
unsigned int UDPport = 2808;        // UDP port number for target ESP
const int packetSize = 1;           // define packetSize (length of message)
byte receiveBuffer[packetSize+1];   // create the receiveBuffer array
int udp_data;

// receive upd
void UDPreceiveData()
{ Serial.println("check for data");
Serial.print("Connected: ");
Serial.println(WL_CONNECTED);
  // if there's a message packet
  if (udp.parsePacket())
  {  
    // read packet
    udp.read(receiveBuffer, packetSize);
    udp_data = receiveBuffer[0];
    Serial.println(udp_data);

    // control motors with command
    control_motors(udp_data);
  }
}

// =================================================================
// ========================== WiFi end ============================
// =================================================================


// =====================================================================
// ============================= SETUP =================================
// =====================================================================
void setup()
{
    Serial.begin(115200);
    
    // ========================= I2C start =============================
    ESP_ERROR_CHECK(i2c_master_init()); // initialize the i2c
    // ========================== I2C end ==============================

    // ===================== Interrupts start ==========================
    // default clock speed is 240MHz
    // 240MHz / 240 = 1MHz      1 000 000 timer increments per second
    // 1 000 000 / 20 = 50 000  timer value to count up to before calling interrupt (call 20 times per second)
    timer = timerBegin(0, 240, true);                       // initialize timer with pre-scaler of 240
    timerAttachInterrupt(timer, &readI2COnTimer, true);     // attach function to timer interrupt
    timerAlarmWrite(timer, 50000, true);                    // set count value before interrupt occurs
    timerAlarmEnable(timer);                                // start the timer
    // ====================== Interrupts end ===========================

    // ========================= LED start =============================
    SetupFastLED(); // set the LEDs
    // ========================== LED end ==============================

    // ========================== Motor start ==============================

    //motor A setup
    ledcSetup(mA1_channel, pwm_freq, pwm_resolution);
    ledcAttachPin(mA1_pin, mA1_channel);
    pinMode(mA1_pin,OUTPUT);
    ledcSetup(mA2_channel, pwm_freq, pwm_resolution);
    ledcAttachPin(mA2_pin, mA2_channel);
    pinMode(mA2_pin,OUTPUT);
  
    //motor B setup
    ledcSetup(mB1_channel, pwm_freq, pwm_resolution);
    ledcAttachPin(mB1_pin, mB1_channel);
    pinMode(mB1_pin,OUTPUT);
    ledcSetup(mB2_channel, pwm_freq, pwm_resolution);
    ledcAttachPin(mB2_pin, mB2_channel);
    pinMode(mB2_pin,OUTPUT);

    //servo setup
    ledcSetup(servo_channel, servo_pwm_freq, servo_pwm_resolution);
    ledcAttachPin(servo_pin, servo_channel);   
      
    // ========================== Motor end ==============================
    
    // ========================== WiFi start ============================

    // connect to WiFi
    Serial.print("Connecting to: "); Serial.println(ssid);
    WiFi.config(myIP,targetIP,IPAddress(255,255,255,0));
    WiFi.begin(ssid, pass);
    WiFi.setSleep(false);
  
    // configure udp comm port
    udp.begin(UDPport);
  
    // hold the code here and wait until the WiFi is connected
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("WiFi connected!");
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    // null terminate at end of receiveBuffer
    receiveBuffer[packetSize] = 0;
    
    // ========================== WiFi end ============================  
}
// =====================================================================
// ========================== END OF SETUP =============================
// =====================================================================


// =====================================================================
// ============================== LOOP =================================
// =====================================================================
void loop()
{
    // ========================= I2C start =============================
    // static variables
    // static variables only get initialized once, when the loop function is first called
    // values of static variables persist through function calls
    static bool gameStatus = 0;     // game on: 1, game off: 0
    static bool reset = 0;          // 1 for resetting
    static bool autoMode = 0;       // not autonomous mode: 0, is auto mode: 1
    static bool syncStatus = 0;     // 1 for sync

    static int health;              // robot's health

    static int respawnTimer;        // amount of time remaining on respawn

    if (readI2C)
    {
        readI2C = 0;                // set readI2C to be false
        i2c_write_test();           // need this write for some reason in order to get read to work?
        i2c_read_test();            // read information from slave (top hat)  

        // read information
        gameStatus  = 1 & (data_rd[0] >> 0);
        reset       = 1 & (data_rd[0] >> 1);
        autoMode    = 1 & (data_rd[0] >> 2);
        syncStatus  = 1 & (data_rd[0] >> 3);
    
        if (data_rd[1] != 0xFF)
        {
            // make sure the data isn't 0xFF (0xFF means that something is wrong)
            health = data_rd[1];
        }

        if (data_rd[2] != 0xFF)
        {
            // make sure the data isn't 0xFF (0xFF means that something is wrong)
            respawnTimer = data_rd[2];
        }
    }
    // ========================== I2C end ==============================

    // ========================= LED start =============================
    
    ShowRobotNum();         // set the LEDs for the robot number
    ShowHealth(health);     // set the LEDs for the health
    
    if(gameStatus == 0)
    {
      // game off
      Serial.println("Game status off");
      FastLEDshowESP32();
      
      control_right_motor(0,1);
      control_left_motor(0,1);
      
      return;
    }
    else if (health == 0)
    {
        Serial.println("Robot dead");
        clearLEDs();
        ShowRespawnTimer(respawnTimer);
        FastLEDshowESP32();
        
        control_right_motor(0,1);
        control_left_motor(0,1);
        
        return;
    }
    
    FastLEDshowESP32();
    
    if(autoMode == 1)
    {
      Read_Serial();
      delay(10);
      controlMotors();
        
      Serial.println("Autonomous mode");
      
      return;
    }
    
    //Serial.println("Normal mode");
    // ========================== LED end ==============================

    // ========================== Motor/UDP start ==============================

        //read UDP and control motors

      UDPreceiveData();
      update_servo();
    // ========================== Motor end ==============================
    delay(10);
}
// =====================================================================
// ========================== END OF LOOP ==============================
// =====================================================================
