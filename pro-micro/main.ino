/**
    \file main.ino
    \author George Brown
    \date 2017-03-11
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// Debug serial via the soft USB UART. (BLOCKING! Disable for flight.)
//#define DEBUG_SERIAL

// MPU Interrupt on INT6 (Pro Micro Pin 7)
#define INTERRUPT_PIN 7

// The MPU object we'll work with.
MPU6050 mpu;

// MPU Status Variables
// Status after DMP Config (0 = OK)
uint8_t dmp_status;
// Number of bytes in a FIFO packet.
uint16_t dmp_packet_length;
// Interrupt flag value.
uint8_t mpu_int_status;
// Number of bytes in the MPU's FIFO.
uint16_t fifo_used;
// Local buffer for reading FIFO data into.
uint8_t fifo_buffer[64];

// MPU data processing variables.
Quaternion quat;
VectorFloat gravity;
// [yaw, pitch, roll]
float ypr[3];
float yaw_rad, pitch_rad, roll_rad;
float yaw_deg, pitch_deg, roll_deg;
int16_t yaw_tx, pitch_tx, roll_tx;

// Level ground offsets.
float yaw_offset, pitch_offset, roll_offset;

// System time after all the configs are complete.
unsigned long startup_millis;

// Flag to keep track of if the IMU has stabilized.
bool imu_stable = false;

// Flag for when new data has been collected.
bool new_data = false;

// Level ground offsets taken during the startup sequence.
float offsetY, offsetP, offsetR;

// Flag set when the MPU interrupt is called.
volatile bool mpu_int_flag = false;

/* Empty MPU packet for sending to the Leonardo.
 * Byte 0: Constant 'P' for synchronisation.
 * Bytes 1, 2: Yaw
 * Bytes 3: Constant ','.
 * Bytes 4, 5: Pitch
 * Bytes 6: Constant ','.
 * Bytes 7, 8: Roll
 * Byte 9: XOR Sum of byes 0-8.
 * Byte 10: Constant '\n'.
 *
 * Yaw/Pitch/Roll parameters are scaled up from angles and sent MSB first.
 */

unsigned char packet[11] = {'P', 0,0, ',', 0,0, ',', 0,0, 0,'\n'};

/**
    \brief When the MPU raises an interrupt, set the flag and them leave.
    \author George Brown
*/
void mpu_interrupt(){
    mpu_int_flag = true;
}

/**
    \brief Configure MPU and initialise I2C, serial ports and DMP.
    \author George Brown
*/
void setup(){
    #ifdef DEBUG_SERIAL
        Serial.begin(115200);
        // Wait for USB serial enumeration. We lose most of the data otherwise!
        while(!Serial);

        // F() to store strings in flash. (Saves on RAM transfers!)
        Serial.println(F("Configuring MPU"));
    #endif

    // Set up the I2C periperhal - max speed is 400 kHz clock.
    Wire.begin();
    Wire.setClock(400000);

    // Wake up the MPU, set sensitivity to max (what we want, yay!).
    mpu.initialize();

    // Ensure our interrupt pin is an input, so the MPU can alert us of new data.
    pinMode(INTERRUPT_PIN, INPUT);

    // Check initalization went OK.
    if(mpu.testConnection()){
        #ifdef DEBUG_SERIAL
            Serial.println(F("MPU Connect OK"));
        #endif
    }
    else{
        #ifdef DEBUG_SERIAL
            Serial.println(F("MPU Connect Failed!"));
        #endif
        // Spin and do nothing if it fails.
        while(1){};
    }

    // Program the MPU's "Digital Motion Processor"
    // Sends a giant configuration blob to the MPU.
    // This configures the DMP to perform "motion fusion" for us.
    dmp_status = mpu.dmpInitialize();

    // Check DMP programming went OK.
    if(dmp_status == 0){
        // Set the MPU off crunching numbers.
        mpu.setDMPEnabled(true);

        #ifdef DEBUG_SERIAL
            Serial.println(F("DMP Enabled."));
        #endif

        // On a rising edge from the MPU, call the interrupt to set the flag.
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), mpu_interrupt, RISING);
        mpu_int_status = mpu.getIntStatus();

        #ifdef DEBUG_SERIAL
            Serial.println(F("Setup Complete."));
        #endif
        // We need to know the FIFO packet size for collecting data from the FIFO later.
        dmp_packet_length = mpu.dmpGetFIFOPacketSize();
    }
    else{
        #ifdef DEBUG_SERIAL
            Serial.println(F("DMP Programming Failure."));
        #endif
        // On DMP setup failure, fall over.
        while(1){}
    }

    // Resting position gyro offsets. Get these from the calibration example script.
    // We don't need this bit in theory, but it's handy for serial stuff.
    mpu.setXGyroOffset(62);
    mpu.setYGyroOffset(-47);
    mpu.setZGyroOffset(30);
    mpu.setZAccelOffset(2109);

    // Turn on the hardware UART serial link to the Leonardo.
    Serial1.begin(115200);

    // Record the uptime now, to permit nonblocking wait for stabilisation.
    startup_millis = millis();
}

/**
    \brief
    \author George Brown
*/
void loop(){
    // Give time for the MPU to thermally stabilize.
    // During this time collect data and just scrap it in order to avoid FIFO
    // overflow.
    while(!imu_stable){
        // Collect data, but do nothing with it.
        mpu_get_ypr();
        // If the 30 second delay is up.
        if((millis() - startup_millis) > 30000){
            // After the stabilisation delay, take the current values as "level".
            yaw_offset = ypr[0];
            pitch_offset = ypr[1];
            roll_offset = ypr[2];
            // Mark the IMU stable so we can proceed with data transmission.
            imu_stable = true;
      }
    }

    // Spin while there is not new MPU data to collect.
    while(!mpu_int_flag && fifo_used < dmp_packet_length){
    }

    // Handle the interrupt from the MPU (usually means new data!)
    mpu_get_ypr();
    // If we got new data, send it.
    if(new_data){
        send_ypr_packet();
        new_data = false;
    }
}

void send_ypr_packet(void){
    // Values in radians. Apply the level calibration offsets.
    yaw_rad = ypr[0] - yaw_offset;
    pitch_rad = ypr[1] - pitch_offset;
    roll_rad = ypr[2] - roll_offset;

    // Conver the angles to degrees for ease of use.
    yaw_deg = yaw_rad * (180.0f/M_PI);
    pitch_deg = pitch_rad * (180.0f/M_PI);
    roll_deg = roll_rad * (180.0f/M_PI);

    // Scale values to 16-bit integers to be ready for transmission.
    // Scale Yaw (-180 -> +180) to - 2^15 -> 2^15
    yaw_tx = yaw_deg * 182.039f;
    // Scale Pitch, Roll (-90 -> +90) to - 2^15 -> 2^15
    pitch_tx = pitch_deg * 364.078f;
    roll_tx = roll_deg * 364.078f;

    // Convert the 16 bit integers to 8-bit bytes for transmission.
    packet[1] = yaw_tx >> 8;
    packet[2] = yaw_tx & 0xff;
    packet[4] = pitch_tx >> 8;
    packet[5] = pitch_tx & 0xff;
    packet[7] = roll_tx >> 8;
    packet[8] = roll_tx & 0xff;
    packet[9] = checksum(packet, 9);
    Serial1.write(packet, 11);

    #ifdef SERIAL_DEBUG
        // Print out the MPU data in a friendly way for the user.
        Serial.print(yaw_deg);
        Serial.print(',');
        Serial.print(pitch_deg);
        Serial.print(',');
        Serial.println(roll_deg);
        Serial.write(packet, 11);
    #endif
}

void mpu_get_ypr(void){
    // Reset the interrupt flag, and find out why the interrupt occured.
    mpu_int_flag = false;
    mpu_int_status = mpu.getIntStatus();
    // Check the amount of data in the FIFO.
    fifo_used = mpu.getFIFOCount();

    // Check the FIFO interrupt flag.
    // Also check the FIFO is not full.
    // This shouldn't occur unless the code is too slow.
    if((mpu_int_status & 0x10) || fifo_used == 1024) {
        // If the FIFO overflows, clear it out.
        mpu.resetFIFO();
        #ifdef DEBUG_SERIAL
            // Warn the user if this is happening - would likley cause problems.
            Serial.println(F("FIFO Overflowed."));
        #endif

    }
    // Otherwise, if this is an inerrupt for new data.
    else if(mpu_int_status & 0x02) {
        // Wait for the MPU to finish filling the FIFO with a full data packet.
        while(fifo_used < dmp_packet_length){
            fifo_used = mpu.getFIFOCount();
        }

        // Pull the data from the FIFO.
        mpu.getFIFOBytes(fifo_buffer, dmp_packet_length);

        // If we missed an interrupt call at some point, there may still be
        // data in the FIFO. Keep track of the FIFO bytes so we can run again.
        fifo_used -= dmp_packet_length;

        // Process the Quaternion output into YPR data.
        mpu.dmpGetQuaternion(&quat, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &quat);
        mpu.dmpGetYawPitchRoll(ypr, &quat, &gravity);

        new_data = true;
    }
}

/**
    \brief Compute the XOR sum for the first len bytes of an array.
    \param packet Data to compute the checksum on.
    \param len Number of bytes to compute the checksum for.
*/
unsigned char checksum(unsigned char *packet, int len)
{
  unsigned char csum = 0;
  unsigned int idx;
  for(idx = 0; idx < len; idx++)
  {
    csum ^= packet[idx];
  }
  return csum;
}
