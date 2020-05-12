#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

//================================================================================================================
//ROS handler
ros::NodeHandle_<ArduinoHardware, 1, 2, 32, 384> nh; //Limit NodeHandle 
                                                      //ROS buffer memory for Arduino-based device with low RAM
                                                      //http://wiki.ros.org/rosserial/Overview/Limitations
                                                      //https://answers.ros.org/question/28890/using-rosserial-for-a-atmega168arduino-based-motorcontroller/
                                                      //http://wiki.ros.org/rosserial_arduino/Tutorials/NodeHandle%20and%20ArduinoHardware
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
ros::Publisher imu_pub("imu",&imu_msg);
ros::Publisher mag_pub("MagneticField",&mag_msg);
uint32_t seq;

//=================================================================================================================


//================================================================================================================
/*  
0       | +/- 2g           | 8192 LSB/mg
1       | +/- 4g           | 4096 LSB/mg
2       | +/- 8g           | 2048 LSB/mg
3       | +/- 16g          | 1024 LSB/mg

and those the one you can use in the function setFullScaleGyroRange()
0 = +/- 250 degrees/sec
1 = +/- 500 degrees/sec
2 = +/- 1000 degrees/sec
3 = +/- 2000 degrees/sec
*/

float accel_scale = 1.196289e-3;
//scale = scale(g) / resolution, scale(g) = 8(g), resolution 16 bit
float gyro_scale = 1.33158055e-4;
//radians/s
//================================================================================================================


//================================================================================================================
MPU6050 mpu;  //MPU-9250 is basically a MPU-6050 with an external compass bypass via I2C

#define    INTERRUPT_PIN              8  
#define    MAG_ADDRESS                0x0C

bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int16_t data[10];
int16_t mx, my, mz;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 Gyro; 
VectorFloat gravity;    // [x, y, z]            gravity vector

//Value for calibration
//int16_t ax, ay, az;
//int16_t gx, gy, gz;
//int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset=0;
//int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
//int gyro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
//int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)


uint8_t mode = 0x11;
int16_t COEFF_X, COEFF_Y, COEFF_Z;
       
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
//================================================================================================================


//================================================   FUNCTIONS   =================================================


//================================================================================================================
/*
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}
//================================================================================================================


//================================================================================================================
void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();
    ////Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=gyro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(gyro_deadzone+1);

    if (abs(mean_gy)<=gyro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(gyro_deadzone+1);

    if (abs(mean_gz)<=gyro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(gyro_deadzone+1);

    if (ready==6) break;
  }
}
*/
//================================================================================================================



//================================================================================================================
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}
//================================================================================================================


//================================================================================================================
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();
    
    // Read Nbytes
    Wire.requestFrom(Address, Nbytes); 
    uint8_t index=0;
    while (Wire.available())
        Data[index++]=Wire.read();
}

//================================================================================================================


//================================================================================================================
void magInitialize() {
    // disable bypass mode
    mpu.setI2CMasterModeEnabled(false);
    mpu.setI2CBypassEnabled(true);
    mpu.setSleepEnabled(false);
    // set AKG8963 power down
    I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
}
//================================================================================================================


//==================================================    MAIN   ===================================================


//================================================================================================================
void setup() {

    //ROS Init
    //ROS BAUDRATE 115200!!!
    nh.initNode();
    nh.advertise(imu_pub);
    nh.advertise(mag_pub);
  
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    
    //Serial.begin(115200);
    //while (!//Serial); // wait for Leonardo enumeration, others continue immediately

    ////Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    mpu.setFullScaleAccelRange(2); //Set scale
    mpu.setFullScaleGyroRange(1);  //Set scale

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    //Calibration
    // supply your own gyro offsets here, scaled for min sensitivity
    //    mpu.setXGyroOffset(220);  
    //    mpu.setYGyroOffset(76);
    //    mpu.setZGyroOffset(-85);
    //    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // reset offsets
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);  
    mpu.setZGyroOffset(0);
    //meansensors();
    //calibration();
    

    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready     
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }
    
    magInitialize();

    // configure LED for output
    seq = 0;
}
//================================================================================================================


//================================================================================================================
void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        //Serial.println(F("FIFO overflow!"));
        //FIFO overflow usually happens if enable external compass reading!

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        imu_msg.header.stamp = nh.now();
        imu_msg.header.frame_id = 1;
        imu_msg.header.seq = seq;
        mag_msg.header.stamp = nh.now();
        mag_msg.header.frame_id = 1;
        mag_msg.header.seq = seq;
        seq++;
      
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize; 

        
        // _____________________
        // :::  Magnetometer :::        
        // Read register Status 1 and wait for the DRDY: Data Ready
      
        uint8_t ST1;
        do {  I2Cread(MAG_ADDRESS,0x02,1,&ST1); }
        while (!(ST1&0x01));
      
        // Read magnetometer data  
        uint8_t Mag[7];  
        I2Cread(MAG_ADDRESS,0x03,7,Mag);
        
        
        // Create 16 bits values from 8 bits data
          
        // Magnetometer
        int16_t mx=-(Mag[3]<<8 | Mag[2]);
        int16_t my=-(Mag[1]<<8 | Mag[0]);
        int16_t mz=-(Mag[5]<<8 | Mag[4]);
    
        
        // Magnetometer
        mag_msg.magnetic_field.x = mx; 
        mag_msg.magnetic_field.y = my; 
        mag_msg.magnetic_field.z = mz; 
        mag_pub.publish(&mag_msg);

        
        //========================================================================================================

        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        imu_msg.orientation.x = q.x * 2;
        imu_msg.orientation.y = q.y * 2;
        imu_msg.orientation.z = q.z * 2;
        //Why multiplication by 2? I test it, and I don't know why
        
        mpu.dmpGetGyro(&Gyro, fifoBuffer);
        imu_msg.angular_velocity.x = Gyro.x * gyro_scale;
        imu_msg.angular_velocity.y = Gyro.y * gyro_scale;
        imu_msg.angular_velocity.z = Gyro.z * gyro_scale; 

        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        imu_msg.linear_acceleration.x = aaReal.x * accel_scale;
        imu_msg.linear_acceleration.y = aaReal.y * accel_scale;
        imu_msg.linear_acceleration.z = aaReal.z * accel_scale;
        
        /*
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);


        // display angular acceleration
        mpu.dmpGetGyro(&Gyro, fifoBuffer);
        Serial.print("gyro\t");
        Serial.print(Gyro.x * gyro_scale;);
        Serial.print("\t");
        Serial.print(Gyro.y * gyro_scale;);
        Serial.print("\t");
        Serial.println(Gyro.z * gyro_scale;);
            

        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        Serial.print("areal\t");
        Serial.print(aaReal.x * accel_scale);
        Serial.print("\t");
        Serial.print(aaReal.y * accel_scale);
        Serial.print("\t");
        Serial.println(aaReal.z * accel_scale);
        */

        imu_pub.publish(&imu_msg);
        mag_pub.publish(&mag_msg);
        nh.spinOnce();
        delay(2);
    }
}
