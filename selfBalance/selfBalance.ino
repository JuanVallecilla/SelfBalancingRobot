#include "I2Cdev.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include <HCSR04.h>
#include <NewPing.h>

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



/*********Tune these 4 values for your BOT*********/
double setpoint = 181; //set the value when the bot is perpendicular to ground using serial monitor.
//Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 18; //Set this first
double Kd = 1.2; //Set this secound
double Ki = 140; //Finally set this
/******End of values setting*********/


int IN4 = 5;    //  arduino  pin 5 to l298  pin IN4
int IN3 = 6;    //  arduino  pin 6 to l298  pin IN3
int IN1 = 7;    //  arduino  pin 7 to l298  pin IN1
int IN2 = 8;    //  arduino  pin 8 to l298  pin IN2
int ENB  = 9;    //  arduino  pin 9 to l298  pin ENB
int ENA  = 10;   //  arduino  pin 10 to l298  pin ENA

/*SRF05 Ultrasonic Sensor*/
#define TRIG 12 // the SRF04 Trig pin
#define ECHO 13 // the SRF04 Echo pin

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
NewPing sonar(TRIG, ECHO , 80);



volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif




  Serial.begin(115200);
  while (!Serial); //
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();


  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(126);
  mpu.setYGyroOffset(1);
  mpu.setZGyroOffset(12);
  mpu.setXAccelOffset(-1488);
  mpu.setYAccelOffset(-723);
  mpu.setZAccelOffset(1100);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    mpu.resetFIFO(); // Clear fifo buffer
    mpu.getIntStatus();
    mpuInterrupt = false; // wait for next interrupt

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(IN4, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);



  pinMode(IN4, LOW);
  pinMode(IN3, LOW);
  pinMode(IN2, LOW);
  pinMode(IN1, LOW);

  //SRF05 Ultrasonic Sensor
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);


}


void Forward() //Code to rotate the wheel forward
{
  int pwm = abs(output);
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}

void Reverse() //Code to rotate the wheel Backward
{
  int pwm = abs(output);
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);

}

void Stop() //Code to stop both the wheels
{
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}
void turnright() {
  int pwm = abs(output);
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


void gryroscope() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    //no mpu data - performing PID calculations and output to motors
    pid.Compute();

    //Print the value of Input and Output on serial monitor to check how it is working.
    //Serial.print(input); Serial.print(" =>"); Serial.println(output);

    if (input > 150 && input < 195) { //If the Bot is falling

      if (output > 0) //Falling towards front
        Reverse(); //Rotate the wheels forward
      else if (output < 0) //Falling towards back
        Forward(); //Rotate the wheels backward
    }
    else //If Bot not falling
      Stop(); //Hold the wheels still


  break;

  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
    mpu.dmpGetGravity(&gravity, &q); //get value for gravity
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
    input = ypr[1] * 180 / M_PI + 180;
    mpu.resetFIFO();



  }
}



void loop() {
  gryroscope();
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm());
  Serial.println("cm");
  
}
