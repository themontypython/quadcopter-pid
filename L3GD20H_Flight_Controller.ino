    ///////////////////////////////////////////////////////////////////////////////////////
    //Terms of use - Adapted from YMFC-AL project for L3GD20H gyro
    ///////////////////////////////////////////////////////////////////////////////////////
    #include <Wire.h>                          

    #define LED_PIN 12
    #define GYRO_ADDR 0x6B  // We now know the correct address
    byte active_address = 0;  // Made global

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //PID gain and limit settings
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    float pid_p_gain_roll = 0.5;               // Start with a small P value
    float pid_i_gain_roll = 0.0;               // No integral control yet
    float pid_d_gain_roll = 5.0;               // Add some D to dampen oscillations
    int pid_max_roll = 200;                    // Reduce from 400 to 200 for first test

    float pid_p_gain_pitch = pid_p_gain_roll;  // Same for pitch
    float pid_i_gain_pitch = pid_i_gain_roll;
    float pid_d_gain_pitch = pid_d_gain_roll;
    int pid_max_pitch = 200;                    // Reduce from 400 to 200 for first test

    float pid_p_gain_yaw = 1.0;                // Slightly higher P for yaw
    float pid_i_gain_yaw = 0.0;                // No integral
    float pid_d_gain_yaw = 0.0;                // No derivative
    int pid_max_yaw = 200;                    // Reduce from 400 to 200 for first test

    boolean auto_level = true;                 //Auto level on (true) or off (false)

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Declaring global variables
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
    volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
    int receiver_input[5];
    int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
    int esc_1, esc_2, esc_3, esc_4;
    int throttle;
    int cal_int, start;
    unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
    unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
    unsigned long loop_timer;
    double gyro_pitch, gyro_roll, gyro_yaw;
    double gyro_axis_cal[4];
    float pid_error_temp;
    float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
    float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
    float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
    float angle_roll, angle_pitch;
    boolean gyro_angles_set;
    int16_t gyro_x, gyro_y, gyro_z;  // Added these declarations

    // Add with other global variables at top of file
    unsigned long lastPrint = 0;

    // Add these globals at top
    const int GYRO_X_OFFSET = 22;  // Average of X readings
    const int GYRO_Y_OFFSET = 11;  // Average of Y readings
    const int GYRO_Z_OFFSET = -54; // Average of Z readings

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Setup routine
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void setup() {
        Serial.begin(9600);
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, HIGH);
        
        Serial.println("Starting gyro initialization...");
        
        Wire.begin();
        TWBR = 12;
        delay(100);
        
        // Initialize L3GD20H
        Wire.beginTransmission(GYRO_ADDR);
        Wire.write(0x20);  // CTRL_REG1
        Wire.write(0x0F);  // Normal mode, all axes enabled
        Wire.endTransmission();
        
        Wire.beginTransmission(GYRO_ADDR);
        Wire.write(0x23);  // CTRL_REG4
        Wire.write(0x30);  // 2000 dps full scale
        Wire.endTransmission();
        
        // Verify settings
        Wire.beginTransmission(GYRO_ADDR);
        Wire.write(0x20);  // Read CTRL_REG1
        Wire.endTransmission();
        Wire.requestFrom(GYRO_ADDR, 1);
        byte reg1 = Wire.read();
        
        Serial.print("CTRL_REG1 = 0x");
        Serial.println(reg1, HEX);
        
        // Now let's read some gyro data
        Serial.println("Reading gyro data...");
        
        // Configure ESC pins
        DDRD |= B11110000;  // Digital pins 4,5,6,7 as outputs
        DDRB |= B00110000;  // Digital pins 12 and 13 as outputs
        
        // Set up receiver interrupts
        PCICR |= (1 << PCIE0);    // Set PCIE0 to enable PCMSK0 scan
        PCMSK0 |= (1 << PCINT0);  // Set PCINT0 (digital input 8) to trigger an interrupt on state change
        PCMSK0 |= (1 << PCINT1);  // Set PCINT1 (digital input 9) to trigger an interrupt on state change
        PCMSK0 |= (1 << PCINT2);  // Set PCINT2 (digital input 10)to trigger an interrupt on state change
        PCMSK0 |= (1 << PCINT3);  // Set PCINT3 (digital input 11)to trigger an interrupt on state change
        
        // Calibrate gyro
        for (cal_int = 0; cal_int < 2000; cal_int++) {
            read_gyro();
            gyro_axis_cal[1] += gyro_x;
            gyro_axis_cal[2] += gyro_y;
            gyro_axis_cal[3] += gyro_z;
            PORTD |= B11110000;
            delayMicroseconds(1000);
            PORTD &= B00001111;
            delay(3);
        }
        
        gyro_axis_cal[1] /= 2000;
        gyro_axis_cal[2] /= 2000;
        gyro_axis_cal[3] /= 2000;
        
        Serial.println("Waiting for receiver...");
        while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400) {
            receiver_input_channel_3 = convert_receiver_channel(3);
            receiver_input_channel_4 = convert_receiver_channel(4);
            Serial.print("CH3:");
            Serial.print(receiver_input_channel_3);
            Serial.print(" CH4:");
            Serial.println(receiver_input_channel_4);
            delay(4);
        }
        
        digitalWrite(LED_PIN, LOW);
        Serial.println("Setup complete!");
        
        loop_timer = micros();                                     //Reset the loop timer
        
        // Initialize ESCs with minimum throttle signal
        Serial.println("Initializing ESCs...");
        for(int i = 0; i < 50; i++) {
            PORTD |= B11110000;                     // Set pins 4, 5, 6, 7 high
            delayMicroseconds(1000);                // 1ms pulse for minimum throttle
            PORTD &= B00001111;                     // Set pins 4, 5, 6, 7 low
            delay(20);                              // Wait 20ms - important for ESC initialization
        }
        
        Serial.println("ESC initialization complete!");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Main program loop
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void loop() {
        read_gyro();
        
        // Get receiver inputs
        receiver_input_channel_1 = convert_receiver_channel(1);
        receiver_input_channel_2 = convert_receiver_channel(2);
        receiver_input_channel_3 = convert_receiver_channel(3);
        receiver_input_channel_4 = convert_receiver_channel(4);
        
        // Calculate PID values
        calculate_pid();
        
        // Motor values
        esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
        esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
        esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
        esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

        // Simple debug print
        if(millis() % 500 == 0) {  // Print every 500ms
            Serial.print("Motors: ");
            Serial.println(throttle);
        }
        
        // Original timing code must remain exactly as it was
        if(micros() - loop_timer > 4050)digitalWrite(12, HIGH);   //Set port 12 high to indicate a loop timing error
        while(micros() - loop_timer < 4000);                      //We wait until 4000us are passed
        loop_timer = micros();                                    //Set the timer for the next loop
        PORTD |= B11110000;                                       //Set digital outputs 4,5,6 and 7 high
        timer_channel_1 = esc_1 + loop_timer;                     //Calculate the time of the faling edge of the esc-1 pulse.
        timer_channel_2 = esc_2 + loop_timer;                     //Calculate the time of the faling edge of the esc-2 pulse.
        timer_channel_3 = esc_3 + loop_timer;                     //Calculate the time of the faling edge of the esc-3 pulse.
        timer_channel_4 = esc_4 + loop_timer;                     //Calculate the time of the faling edge of the esc-4 pulse.
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Interrupt routine for reading receiver signals
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ISR(PCINT0_vect){
        current_time = micros();
        //Channel 1=========================================
        if(PINB & B00000001){                                                     //Is input 8 high?
            if(last_channel_1 == 0){                                              //Input 8 changed from 0 to 1.
                last_channel_1 = 1;                                               //Remember current input state.
                timer_1 = current_time;                                           //Set timer_1 to current_time.
            }
        }
        else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
            last_channel_1 = 0;                                                   //Remember current input state.
            receiver_input[1] = current_time - timer_1;                           //Channel 1 is current_time - timer_1.
        }
        //Channel 2=========================================
        if(PINB & B00000010 ){                                                    //Is input 9 high?
            if(last_channel_2 == 0){                                              //Input 9 changed from 0 to 1.
                last_channel_2 = 1;                                               //Remember current input state.
                timer_2 = current_time;                                           //Set timer_2 to current_time.
            }
        }
        else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
            last_channel_2 = 0;                                                   //Remember current input state.
            receiver_input[2] = current_time - timer_2;                           //Channel 2 is current_time - timer_2.
        }
        //Channel 3=========================================
        if(PINB & B00000100 ){                                                    //Is input 10 high?
            if(last_channel_3 == 0){                                              //Input 10 changed from 0 to 1.
                last_channel_3 = 1;                                               //Remember current input state.
                timer_3 = current_time;                                           //Set timer_3 to current_time.
            }
        }
        else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
            last_channel_3 = 0;                                                   //Remember current input state.
            receiver_input[3] = current_time - timer_3;                           //Channel 3 is current_time - timer_3.
        }
        //Channel 4=========================================
        if(PINB & B00001000 ){                                                    //Is input 11 high?
            if(last_channel_4 == 0){                                              //Input 11 changed from 0 to 1.
                last_channel_4 = 1;                                               //Remember current input state.
                timer_4 = current_time;                                           //Set timer_4 to current_time.
            }
        }
        else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
            last_channel_4 = 0;                                                   //Remember current input state.
            receiver_input[4] = current_time - timer_4;                           //Channel 4 is current_time - timer_4.
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Subroutine for reading the gyro
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void read_gyro() {
        Wire.beginTransmission(GYRO_ADDR);
        Wire.write(0x28 | 0x80);  // Auto-increment bit set
        Wire.endTransmission();
        Wire.requestFrom(GYRO_ADDR, 6);
        
        while(Wire.available() < 6);  // Wait for all data
        
        gyro_x = Wire.read() | (Wire.read() << 8);
        gyro_y = Wire.read() | (Wire.read() << 8);
        gyro_z = Wire.read() | (Wire.read() << 8);
        
        // Apply calibration offsets
        gyro_x -= GYRO_X_OFFSET;
        gyro_y -= GYRO_Y_OFFSET;
        gyro_z -= GYRO_Z_OFFSET;
        
        // Convert to degrees per second (with L3GD20H sensitivity)
        gyro_roll = gyro_x * 0.07;    // For 2000dps range
        gyro_pitch = gyro_y * 0.07;
        gyro_yaw = gyro_z * 0.07;
        
        // Update the PID inputs
        gyro_roll_input = gyro_roll;
        gyro_pitch_input = gyro_pitch;
        gyro_yaw_input = gyro_yaw;
        
        // Debug output (every 500ms)
        if (millis() - lastPrint > 500) {
            Serial.print("Gyro (deg/s) - Roll:");
            Serial.print(gyro_roll);
            Serial.print(" Pitch:");
            Serial.print(gyro_pitch);
            Serial.print(" Yaw:");
            Serial.println(gyro_yaw);
            lastPrint = millis();
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Subroutine for calculating pid outputs
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void calculate_pid(){
        //Roll calculations
        pid_error_temp = gyro_roll_input - pid_roll_setpoint;
        pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
        if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
        else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
        
        pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
        if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
        else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
        
        pid_last_roll_d_error = pid_error_temp;
        
        //Pitch calculations
        pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
        pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
        if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
        else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
        
        pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
        if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
        else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
        
        pid_last_pitch_d_error = pid_error_temp;
        
        //Yaw calculations
        pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
        pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
        if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
        else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
        
        pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
        if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
        else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
        
        pid_last_yaw_d_error = pid_error_temp;
    }

    //Convert the actual receiver signals for calibration
    int convert_receiver_channel(byte function){
        int low, center, high, actual;
        int difference;
        
        actual = receiver_input[function];                                        //Read the actual receiver value for the corresponding function
        low = 1000;                                                              //Set the low value to 1000us
        center = 1500;                                                           //Set the center value to 1500us
        high = 2000;                                                             //Set the high value to 2000us
        
        if(actual < center){                                                     //The actual receiver value is lower than the center value
            if(actual < low)actual = low;                                        //Limit the lowest value to 1000us
            difference = ((long)(center - actual) * (long)500) / (center - low); //Calculate and scale the actual value to a 1000 - 2000us value
            return 1500 - difference;                                            //Return the calculated value
        }
        else if(actual > center){                                                //The actual receiver value is higher than the center value
            if(actual > high)actual = high;                                      //Limit the highest value to 2000us
            difference = ((long)(actual - center) * (long)500) / (high - center);//Calculate and scale the actual value to a 1000 - 2000us value
            return 1500 + difference;                                            //Return the calculated value
        }
        else return 1500;
    }

    void set_gyro_registers(){
        //Setup the L3GD20H
        Wire.beginTransmission(active_address);                                             //Start communication with L3GD20H
        Wire.write(0x20);                                                        //CTRL_REG1
        Wire.write(0x0F);                                                        //Normal mode, all axes enabled
        Wire.endTransmission();
        
        Wire.beginTransmission(active_address);
        Wire.write(0x23);                                                        //CTRL_REG4
        Wire.write(0x90);                                                        //Block data update, 500dps
        Wire.endTransmission();
        
        //Verify the gyro configuration
        Wire.beginTransmission(active_address);
        Wire.write(0x20);
        Wire.endTransmission();
        Wire.requestFrom(active_address, 1);
        while(Wire.available() < 1);
        if(Wire.read() != 0x0F){                                                //Verify CTRL_REG1 setting
            digitalWrite(LED_PIN, HIGH);                                              //Turn on warning LED
            while(1)delay(10);                                                   //Stay in this loop if there's an error
        }
    } 
    