//Declaring Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4;

//Setup Routine
void setup() {
    //Arduino (Atmega) pins default to inputs, so we don't need to set them as inputs
    PCICR |= (1 << PCIE0); //set PCIE0 to enable PCINT0 scan
    PCMSK0 |= (1 << PCINT0); //set PCINT0 (digital input 8) to trigger interrupt on state change
    PCMSK0 |= (1 << PCINT1); //set PCINT1 (digital input 9) to trigger interrupt on state change
    PCMSK0 |= (1 << PCINT2); //set PCINT2 (digital input 10) to trigger interrupt on state change
    PCMSK0 |= (1 << PCINT3); //set PCINT3 (digital input 11) to trigger interrupt on state change

    //Set up the serial communication
    Serial.begin(9600);
}

//Main Loop
void loop() {
    delay(250);
    print_signals();
}

//This routine is called everytime input 8,9,10 or 11 changes state
ISR(PCINT0_vect) {
    //Channel 1========================================
    if(last_channel_1 == 0 && PINB & B00000001 ){   //Input 8 changed from 0 to 1
        last_channel_1 = 1;                         //set last_channel_1 to 1
        timer_1 = micros();                         //set timer_1 to current time
    }
    else if(last_channel_1 == 1 && !(PINB & B00000001)){
        last_channel_1 = 0;
        receiver_input_channel_1 = micros() - timer_1;  
    }
    //Channel 2========================================
    if(last_channel_2 == 0 && PINB & B00000010 ){   //Input 8 changed from 0 to 1
        last_channel_2 = 1;                         //set last_channel_2 to 1
        timer_2 = micros();                         //set timer_2 to current time
    }
    else if(last_channel_2 == 1 && !(PINB & B00000010)){
        last_channel_2 = 0;
        receiver_input_channel_2 = micros() - timer_2;  
    }
    //Channel 3========================================
    if(last_channel_3 == 0 && PINB & B00000100 ){   //Input 8 changed from 0 to 1
        last_channel_3 = 1;                         //set last_channel_3 to 1
        timer_3 = micros();                         //set timer_3 to current time
    }
    else if(last_channel_3 == 1 && !(PINB & B00000100)){
        last_channel_3 = 0;
        receiver_input_channel_3 = micros() - timer_3;  
    }
    //Channel 4========================================
    if(last_channel_4 == 0 && PINB & B00001000 ){   //Input 8 changed from 0 to 1
        last_channel_4 = 1;                         //set last_channel_4 to 1
        timer_4 = micros();                         //set timer_4 to current time
    }
    else if(last_channel_4 == 1 && !(PINB & B00001000)){
        last_channel_4 = 0;
        receiver_input_channel_4 = micros() - timer_4;  
    }
}

//Subroutine for displaying the receiver signals
void print_signals() {
    Serial.print("Roll:");
    if(receiver_input_channel_1 - 1480 < 0)Serial.print("<<<");
    else if(receiver_input_channel_1 - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(receiver_input_channel_1);
    
    Serial.print("  Nick:");
    if(receiver_input_channel_2 - 1480 < 0)Serial.print("^^^");
    else if(receiver_input_channel_2 - 1520 > 0)Serial.print("vvv");
    else Serial.print("-+-");
    Serial.print(receiver_input_channel_2);
    
    Serial.print("  Gas:");
    if(receiver_input_channel_3 - 1480 < 0)Serial.print("vvv");
    else if(receiver_input_channel_3 - 1520 > 0)Serial.print("^^^");
    else Serial.print("-+-");
    Serial.print(receiver_input_channel_3);
    
    Serial.print("  Yaw:");
    if(receiver_input_channel_4 - 1480 < 0)Serial.print("<<<");
    else if(receiver_input_channel_4 - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.println(receiver_input_channel_4);
}



