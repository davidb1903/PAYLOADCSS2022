#include <BMP280_DEV.h>
#include <Device.h>
#include <MechaQMC5883.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <EEPROM.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
void(* resetFunc) (void) = 0;
Servo servoMotor;
//
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int mx, my, mz, a, a1;
float rtemperature, raltitude, pressure;
int angulo = 0;
int dirRef = 0; //Sur
char ComandoON = 'A'; //0x41
char ComandoOFF = 'B'; //0x42
MechaQMC5883 qmc; //Create an object name for the snsor, I have named it as qmc
BMP280_DEV bmp280;
// Define a Array
float sensorRead[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
String state_f="RELEASED";
//String states[10]={"RELEASED","TARGET_POINTING","LANDED"};
//
TaskHandle_t TaskHandle_Sensor; // handler for TasksSensors
TaskHandle_t TaskHandle_Serial; // handler for TasksSensors
//
void TaskSerial(void *pvParameters);
void TaskSensors(void *pvParameters);
void TaskComOFF(void *pvParameters);
void init_Interfaces();
void validation_comand();
int dirSouth_heading(int a);
//
QueueHandle_t arrayQueue;

void setup() {
  init_Interfaces();
    arrayQueue = xQueueCreate(10, //Queue length
                            sizeof(int)); //Queue item size
  validation_comand();
  
  accelgyro.initialize();
  //
  if (arrayQueue != NULL) {
    // Create task that consumes the queue if it was created.
    // Create task that publish data in the queue if it was created.
    // Create task that publish data in the queue if it was created.
    xTaskCreate(TaskComOFF, // Task function
                "TaskComOFF",// Task name
                128,// Stack size
                NULL,
                1,// Priority
                NULL);
  }
    accelgyro.initialize();
}

void loop() {}

void TaskComOFF(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    if (Serial.available() > 1 ) {
      char Dato = Serial.read();
      if (Dato == ComandoOFF) {
        state_f="LANDED";
        digitalWrite(8, HIGH);
        digitalWrite(9, HIGH);
        EEPROM.update(0, 0);
        digitalWrite(6, HIGH);
        delay(3000);
        digitalWrite(6, LOW);
        EEPROM.update(1, 0);
        vTaskDelete(TaskHandle_Sensor);
        vTaskDelete(TaskHandle_Serial);
        vTaskDelete(NULL);
      }
      if (Dato=='C'){
        state_f="TARGET_POINTING";
        }
    }
  }
}

void TaskSensors(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    bmp280.startForcedConversion();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bmp280.getMeasurements(rtemperature, pressure, raltitude);
    sensorRead[1] = raltitude;
    sensorRead[1] = raltitude;
    sensorRead[2] = rtemperature;
    sensorRead[3] = float(analogRead(A0) * 0.0098);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    qmc.read(&mx, &my, &mz, &a);
    sensorRead[4] = gx* (9.81/2048);;
    sensorRead[5] = gy* (9.81/2048);;
    sensorRead[6] = gz* (9.81/2048);;
    sensorRead[7] = ax*(250.0/32768.0);
    sensorRead[8] = ay*(250.0/32768.0);
    sensorRead[9] = az*(250.0/32768.0);
    sensorRead[10] = mx;
    sensorRead[11] = my;
    sensorRead[12] = mz;
    float h=atan2(mz,mx);
    sensorRead[13] = h;
    if (abs(sensorRead[13]) > 10) {
      angulo = int(float(angulo) - (float(sensorRead[13] * 0.5)));
      servoMotor.write(angulo);
    }
    xQueueSend(arrayQueue, &sensorRead, portMAX_DELAY);
  }
}

void TaskSerial(void *pvParameters) {
  (void) pvParameters;
  Serial.begin(9600);
  while (!Serial) {
    vTaskDelay(1);
  }
  TickType_t lasttick = xTaskGetTickCount();
  for (;;) {
    if (xQueueReceive(arrayQueue, &sensorRead, portMAX_DELAY) == pdPASS ) {
      Serial.print('X');
      Serial.print(sensorRead[1], 1);
      Serial.print(",");
      Serial.print(sensorRead[2], 1);
      Serial.print(",");
      Serial.print(sensorRead[3], 1);
      Serial.print(",");
      Serial.print(sensorRead[4], 0);
      Serial.print(",");
      Serial.print(sensorRead[5], 0);
      Serial.print(",");
      Serial.print(sensorRead[6], 0);
      Serial.print(",");
      Serial.print(sensorRead[7], 0);
      Serial.print(",");
      Serial.print(sensorRead[8], 0);
      Serial.print(",");
      Serial.print(sensorRead[9], 0);
      Serial.print(",");
      Serial.print(sensorRead[10], 0);
      Serial.print(",");
      Serial.print(sensorRead[11], 0);
      Serial.print(",");
      Serial.print(sensorRead[12], 0);
      Serial.print(",");
      Serial.print(sensorRead[13], 0);
      Serial.print(",");
      Serial.println(state_f);
      vTaskDelayUntil(&lasttick, 245 / portTICK_PERIOD_MS);
    }
  }
}

void init_Interfaces() {
  pinMode(9, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);
  Serial.begin(9600);
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    accelgyro.initialize();
  qmc.init(); //Initialise the QMC5883 Sensor
  qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);
  bmp280.begin(BMP280_I2C_ALT_ADDR);
  bmp280.setTimeStandby(TIME_STANDBY_05MS);
  bmp280.setPresOversampling(OVERSAMPLING_X4);    // Set the pressure oversampling to X4
  bmp280.setTempOversampling(OVERSAMPLING_X1);    // Set the temperature oversampling to X1
  bmp280.startNormalConversion();
  servoMotor.attach(5);
  servoMotor.write(45);
  qmc.read(&mx, &my, &mz, &a);
}
void validation_comand() {
  while (!EEPROM.read(0) == 1) {
    if (Serial.available() > 1 ) {
      char Dato = Serial.read();
      if (Dato == ComandoON) {
        EEPROM.update(0, 1);
        if (EEPROM.read(1) == 0) {
           xTaskCreate(TaskSensors, // Task function
                "TaskSensors",// Task name
                128,// Stack size
                NULL,
                1,// Priority
                &TaskHandle_Sensor);
           xTaskCreate(TaskSerial,// Task function
                "PrintSerial",// Task name
                128,// Stack size
                NULL,
                3,// Priority
                &TaskHandle_Serial);
        state_f="RELEASED";
          digitalWrite(6, HIGH);
          delay(3000);
          digitalWrite(6, LOW);
          delay(2000);
          digitalWrite(6, HIGH);
          delay(1000);
          digitalWrite(6, LOW);
          delay(1000);
          digitalWrite(6, HIGH);
          delay(500);
          digitalWrite(6, LOW);
          EEPROM.update(1, 1);
          resetFunc();
        }
      }
    }
  }
      xTaskCreate(TaskSerial,// Task function
                "PrintSerial",// Task name
                128,// Stack size
                NULL,
                3,// Priority
                &TaskHandle_Serial);
                    xTaskCreate(TaskSensors, // Task function
                "TaskSensors",// Task name
                128,// Stack size
                NULL,
                1,// Priority
                &TaskHandle_Sensor);
}

int dirSouth_heading(float h) {
 if(h < 0)
    h += 2*M_PI;
 if(h > 2*M_PI)
    h -= 2*M_PI;
    return h* 180/M_PI; 
}
