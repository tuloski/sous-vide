

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2               // temp sensor data on pin 2
#define TEMPERATURE_PRECISION 12     //11 bit precision
#define LOOP_TIME 50                //buttons handler loop time
#define DT LOOP_TIME/1000
#define DT_CONTROL 500/1000     //control loop at 2 Hz-->500ms

#define STATE_INIT 0         //state machine states
#define STATE_PRE 10
#define STATE_PRE2 11
#define STATE_ON 20

#define KP 14               //controller
#define KI 0.038
#define MAX_I 80
#define MIN_I 0
#define MAX_PI 100    //maximum 100% of duty cycle --> power 

int Touch_1 = 7;       //buttons and relay pin
int Touch_2 = 8;
int Relay = 10;

// set the LCD address to 0x20 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address


int flag_timer = 0;               //run timer or not
int STATE = STATE_INIT;         //initialize state
uint64_t start_time = 0;
float comm_temp_preround = 40;     //initialize commanded temperature
float comm_temp = 40;            //initialize commanded temperature
float actual_temp = 0;   
long counter_button = 0;

float proportional = 0;     //controller variables
float integral = 0;
float error;
float pi = 0;
int pwm = 0;

unsigned long timer_init = 0;
unsigned long timer = 0;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

void setup()   
{
  pinMode(Touch_1, INPUT);
  pinMode(Touch_2, INPUT);
  pinMode(Relay, OUTPUT);
  lcd.begin(20,4);         // initialize the lcd for 20 chars 4 lines and turn on backlight
  lcd.backlight();
  Serial.begin(9600);
  
  
  
  sensors.begin();
  sensors.setResolution(TEMPERATURE_PRECISION);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  
  cli();
  //set timer1 interrupt at 1.333Hz (waiting the probe 750ms)
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  // 16x10e6/(freq*prescaler)-1
  OCR1A = 46875;// = (16*10^6) / (1.333*256) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 bits for 256 prescaler
  TCCR1B |= (1 << CS12);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();
  
// NOTE: Cursor Position: CHAR, LINE) start at 0  
  lcd.setCursor(1,0); //Start at character 4 on line 0
  lcd.print("Sous vide machine!");
  lcd.setCursor(1,2);
  lcd.print("Touch both buttons");
  lcd.setCursor(3,3);
  lcd.print("to start....");
  //delay(1000);

}/*--(end setup )---*/


void loop() 
{
  switch (STATE){
    case STATE_INIT:
      if (digitalRead(Touch_1) && digitalRead(Touch_2)){     //TODO put Touch_2 too
        STATE = STATE_PRE;
      }
      break;
    case STATE_PRE:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Actual temp: ");
      lcd.setCursor(0,1);
      lcd.print("Commanded temp: ");
      lcd.setCursor(0,3);
      lcd.print(" Press both to heat!");
      lcd.setCursor(15,0);
      lcd.print(actual_temp);
      lcd.setCursor(15,1);
      lcd.print(comm_temp);
      delay(100);
      STATE = STATE_PRE2;
      start_time = millis();
      break;
    case STATE_PRE2:
      if (millis()-start_time>LOOP_TIME){
        start_time = millis();
        if (digitalRead(Touch_1) && !digitalRead(Touch_2)){
            counter_button++;
        } else if (!digitalRead(Touch_1) && digitalRead(Touch_2)){
            counter_button--;
        } else {
            counter_button = 0;
            if (digitalRead(Touch_1) && digitalRead(Touch_2)){
              STATE = STATE_ON;
              lcd.setCursor(0,3);
              lcd.print("                    ");
              lcd.setCursor(0,3);
              lcd.print("Power used: ");
              lcd.print(pi);
              lcd.print("%"); 
              delay(1000);  
            }
        } 
        comm_temp_preround += par_sat(counter_button,3)*DT;
        comm_temp = round(comm_temp_preround*4)/4.f;     //round to 0.25 steps
        
        lcd.setCursor(15,0);
        lcd.print(actual_temp,2);
        //Serial.println((long)counter_button);
        lcd.setCursor(15,1);
        lcd.print(comm_temp,2);
      }
      break; 
    case STATE_ON:
      if (millis()-start_time>LOOP_TIME){
        start_time = millis();
        
        if (digitalRead(Touch_1) && !digitalRead(Touch_2)){
            counter_button++;
        } else if (!digitalRead(Touch_1) && digitalRead(Touch_2)){
            counter_button--;
        } else {
            counter_button = 0;
            if (digitalRead(Touch_1) && digitalRead(Touch_2)){
              if  (!flag_timer){
                flag_timer = 1;
                timer_init = millis();
              } else {
                flag_timer = 0;
              }  
            }
        }
        
        if (flag_timer) {
          timer = millis()-timer_init;
          print_time(timer);
        } else {
          timer = 0;
        }  
        
        comm_temp_preround += par_sat(counter_button,3)*DT;
        comm_temp = round(comm_temp_preround*4)/4.f;     //round to 0.25 steps
        
        // print LCD
        
        lcd.setCursor(15,0);
        lcd.print(actual_temp,2);
        //Serial.println((long)counter_button);
        lcd.setCursor(15,1);
        lcd.print(comm_temp,2);
        lcd.setCursor(12,3);
        lcd.print(pi);
        lcd.print("% ");
      }
      break;
    }  
  delay(2); 
}

float par_sat(float x, float lambda){
  float sat_comp = sign(x)*0.000035*x*x+sign(x);
  if (sat_comp>lambda){
    sat_comp = lambda;
  }  
  else if (sat_comp<-lambda){
    sat_comp = -lambda;
  }
  return sat_comp;
}

int sign(float x)
{
  if (x>0){
    return 1;
  } else if (x<0){
    return -1;
  } else {
    return 0;
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void print_time (unsigned long milliseconds){
  if (milliseconds == 0){
    lcd.setCursor(3,2);
    lcd.print("               ");
    lcd.setCursor(3,2);
    lcd.print("0:0:0:0");
  } else {  
    int days = floor (milliseconds/86400000);
    int hours = (milliseconds-days*86400000)/3600000;
    int minutes = (milliseconds-days*86400000-hours*3600000)/60000;
    int seconds = (milliseconds-days*86400000-hours*3600000-minutes*60000)/1000;
    lcd.setCursor(3,2);
    lcd.print(days);
    lcd.print(":");
    lcd.print(hours);
    lcd.print(":");
    lcd.print(minutes);
    lcd.print(":");
    lcd.print(seconds);
  }  
} 

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1.333Hz

  /*------ get temperature ------*/
  actual_temp = sensors.getTempCByIndex(0);
  sensors.requestTemperatures();
  //Serial.println("TIMER INTERRUPT!");
  
  if (STATE==STATE_ON){
    /*--------- controller --------*/
    error = comm_temp - actual_temp;
    proportional = KP*error;
    if (integral>=MAX_I && sign(error)>0){
      integral = MAX_I;
    } else if (integral<=MIN_I && sign(error)<0){
      integral = MIN_I;
    } else if (pi < MAX_PI && pi > 0){
      integral += KI*DT_CONTROL*error;
    } else {
      integral = integral;
    }
    pi = integral+proportional;
    if (pi > MAX_PI) {
      pi = MAX_PI;
    } else if (pi<0){
      pi = 0;
    }
    //Serial.print("Controller: ");
    //Serial.println(pi);
    //Serial.print("Error: ");
    //Serial.println(error);
    
    /*------- pwm ------*/
    
    pwm = round (mapfloat(pi,0,100,0,255));
    analogWrite(Relay,pwm);
    //Serial.print("PWM: ");
    Serial.println(actual_temp);
    //Serial.print("proportional: ");
    //Serial.println(proportional);
    //Serial.print("integral: ");
    //Serial.println(integral);
  }
}