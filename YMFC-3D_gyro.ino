#include <Wire.h>   //Include the Wire library so we can communicate to he gyro

// Declare variables
int cal_int;
unsigned long UL_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_pitch_cal, gyro_roll_cal, gyro_yaw_cal;
byte highByte, lowByte;

// Setup Routine
void setup() {
    Wire.begin();
    Serial.begin(9600);                                   //Start the I2C as master

    //The gyro is disabled by default, so we need to enable it
    Wire.beginTransmission(107);                    //Start communication with the gyro (address 0x6B)
    Wire.write(0x20);                               //we want to write to register 20
    Wire.write(0x0F);                               //Set the register bits to 00001111 (turn on the gyro and enable all axis)
    Wire.endTransmission();                         //End the transmission w/ gyro

    Wire.beginTransmission(107);                    //Start communication with the gyro (address 0x6B)
    Wire.write(0x23);                               //we want to write to register 23
    Wire.write(0x80);                               //Set the register bits to 10000000 (set the BDU bit)
    Wire.endTransmission();                         //End the transmission w/ gyro


    delay(250); //Give the gyro time to start

    //Let's take multiple samples so we can determine the average gyro offset
    Serial.print("Starting Calibration...");        //print message
    for(cal_int = 0; cal_int < 2000; cal_int++){    //take 2000 readings for calibration
        gyro_signalen();                            //read the gyro output
        gyro_roll_cal += gyro_roll;                 //add the roll reading to the running total
        gyro_pitch_cal += gyro_pitch;               //add the pitch reading to the running total
        gyro_yaw_cal += gyro_yaw;                   //add the yaw reading to the running total
        if(cal_int%100 == 0)Serial.print(".");      //print a dot every 100 readings
        delay(4);                                   //wait 4ms before taking the next loop
    }

    //Now we have 20000 measures, we need to divide the running total by 2000 to get the average gyro offset
    Serial.println("done!");                        //20000 measures done
    gyro_roll_cal /= 2000;                          //Divide the roll running total by 2000 to get the average gyro offset
    gyro_pitch_cal /= 2000;                         //Divide the pitch running total by 2000 to get the average gyro offset
    gyro_yaw_cal /= 2000;                           //Divide the yaw running total by 2000 to get the average gyro offset

}

//Main Loop
void loop() {
    delay(50);                                      // Reduced from 250ms to 50ms for more responsive readings
    gyro_signalen();                                
    print_output();                                 
}

void gyro_signalen() {
    Wire.beginTransmission(107);                    //Start communication with the gyro
    Wire.write(168);                                //Read the gyro data
    Wire.endTransmission();                         //End the transmission
    Wire.requestFrom(107, 6);                       //Request 6 bytes from the gyro
    
    while(Wire.available() < 6);                    //Wait for the 6 bytes to be received
    
    // PITCH: First reading is physically X-axis but we'll store as pitch (Y in drone frame)
    lowByte = Wire.read();
    highByte = Wire.read();
    gyro_pitch = ((highByte << 8) | lowByte);       
    if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;
    
    // ROLL: Second reading is physically Y-axis but we'll store as roll (X in drone frame)
    lowByte = Wire.read();
    highByte = Wire.read();
    gyro_roll = ((highByte << 8) | lowByte);        
    if(cal_int == 2000)gyro_roll -= gyro_roll_cal;
    
    // YAW: (Z-axis) stays the same
    lowByte = Wire.read();
    highByte = Wire.read();
    gyro_yaw = ((highByte << 8) | lowByte);         
    gyro_yaw *= -1;                                 
    if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;
}

void print_output() {
    
    Serial.print("Pitch: ");
    if(gyro_pitch >= 0)Serial.print("+");
    Serial.print(gyro_pitch/57.14286, 0);           // Convert to degrees per second
    if(gyro_pitch/57.14286 - 2 > 0)Serial.print("NoU");
    else if(gyro_pitch/57.14286 + 2 < 0)Serial.print("NoD");
    else Serial.print("---");


    Serial.print("  Roll: ");
    if(gyro_roll >= 0)Serial.print("+");
    Serial.print(gyro_roll/57.14286, 0);            // Convert to degrees per second
    if(gyro_roll/57.14286 - 2 > 0)Serial.print("RwD");
    else if(gyro_roll/57.14286 + 2 < 0)Serial.print("RwU");
    else Serial.print("---");

    Serial.print("  Yaw: ");
    if(gyro_yaw >= 0)Serial.print("+");
    Serial.print(gyro_yaw/57.14286, 0);            // Convert to degrees per second
    if(gyro_yaw/57.14286 - 2 > 0)Serial.print("NoR");
    else if(gyro_yaw/57.14286 + 2 < 0)Serial.print("NoL");
    else Serial.print("---");
    
    Serial.println("");  // Add newline at the end of each reading set
}
