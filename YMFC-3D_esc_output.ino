//Declare variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter, start;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
//volitile int int_delay;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_timer;
//volatile unsigned long master_timer_overflow;

//Setup routine
void setup() {
    DDRD |= B11110000;     //Configure digital ports 4, 5, 6, 7 as output
    DDRB |= B00010000;     //Configure digital port 12 as OUTPUT
    pinMode(12, OUTPUT);    //LED pin

    PCICR |= (1 << PCIE0);   //set PCIE0 to enable PCMSKO scan
    PCMSK0 |= (1 << PCINT0); //Set PCINT0 (digital input 8)
    PCMSK0 |= (1 << PCINT1); //Set PCINT1 (digital input 9)
    PCMSK0 |= (1 << PCINT2); //Set PCINT2 (digital input 10)
    PCMSK0 |= (1 << PCINT3); //Set PCINT3 (digital input 11)
    
    while (1) {
        start++;
        PORTD |= B11110000;  
        delayMicroseconds(1000);
        PORTD &= B00001111;  
        delay(3);
        
        if(start == 125) {
            digitalWrite(12, !digitalRead(12));
            start = 0;
        }
        
        if (receiver_input_channel_3 > 1000 && receiver_input_channel_3 < 1050) break;
    }
    
    start = 0;
    zero_timer = micros();
}

//Main loop
void loop() {
    while(zero_timer + 4000 > micros()); 
    zero_timer = micros();               
    
    int throttle = receiver_input_channel_3;
    if(throttle < 800) throttle = 800;
    if(throttle > 2000) throttle = 2000;
    
    PORTD |= B11110000;
    
    timer_channel_1 = throttle + zero_timer;
    timer_channel_2 = throttle + zero_timer;
    timer_channel_3 = throttle + zero_timer;
    timer_channel_4 = throttle + zero_timer;

    while(PORTD >= 16) {
        esc_loop_timer = micros();
        if(timer_channel_1 <= esc_loop_timer) PORTD &= B11101111;
        if(timer_channel_2 <= esc_loop_timer) PORTD &= B11011111;
        if(timer_channel_3 <= esc_loop_timer) PORTD &= B10111111;
        if(timer_channel_4 <= esc_loop_timer) PORTD &= B01111111;
    }
}

//This routine is called every time input 8, 9, 10, or 11 changes state
ISR(PCINT0_vect) {
    current_timer = micros();
    //Channel 1==============================================
    if(PINB & B00000001) {          //Is input 8 high?
        if(last_channel_1 == 0) {    //Input 8 changed from low to high
            last_channel_1 = 1;      //Remember current input state (fixed == to =)
            timer_1 = current_timer; //Store timer value
        }
    }
    else if(last_channel_1 == 1) {   //Input 8 changed from high to low
        last_channel_1 = 0;          //Remember current input state
        receiver_input_channel_1 = current_timer - timer_1; //Calculate pulse time
    }
    
    //Channel 2==============================================
    if(PINB & B00000010) {          //Is input 9 high?
        if(last_channel_2 == 0) {    
            last_channel_2 = 1;
            timer_2 = current_timer;
        }
    }
    else if(last_channel_2 == 1) {
        last_channel_2 = 0;
        receiver_input_channel_2 = current_timer - timer_2;
    }
    
    //Channel 3==============================================
    if(PINB & B00000100) {          //Is input 10 high?
        if(last_channel_3 == 0) {
            last_channel_3 = 1;
            timer_3 = current_timer;
        }
    }
    else if(last_channel_3 == 1) {
        last_channel_3 = 0;
        receiver_input_channel_3 = current_timer - timer_3;
    }
    
    //Channel 4==============================================
    if(PINB & B00001000) {          //Is input 11 high?
        if(last_channel_4 == 0) {
            last_channel_4 = 1;
            timer_4 = current_timer;
        }
    }
    else if(last_channel_4 == 1) {
        last_channel_4 = 0;
        receiver_input_channel_4 = current_timer - timer_4;
    }
}



