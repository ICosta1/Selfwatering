/**
 * @file IoTSelfWateringSystem.ino
 *
 * @brief IoT Self Watering Ssytem 
 * @brief Hardware requirements 
 *									: Argon Particle
 *
 * @author I. Costa
 * @date 01-04-2022
 */
 
 /* @owner ICosta
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/* -- Includes */
/* Associated header */
//#include <Wire.h>
#include "SparkFunMPL3115A2.h"


/* Standard headers */
#include <math.h>
/* Functional headers */

/* -- Constants */
#define BATTERY_SAMPLES     15
#define MOISTURE_SAMPLES    15
#define SEA_LEVEL_PRESSURE  101325 //Pa
#define ONE_DAY_MILLIS      (24 * 60 * 60 * 1000)
#define WATER_PUMP_DEFAULT  30000 //30 Seconds //120000 // 2 min
#define RESET_STATE_TIME    3600000 // 1Hour
#define WATER_TIME          21 // Hour to start Water

#define ZONE1_PUMP          D5           
#define ZONE2_PUMP          D4
#define ZONE3_PUMP          D3       
#define ZONE4_PUMP          D2
#define ZONE5_PUMP          D13

#define LED_LOW_WATER       D12


/* -- Types */ 

// Changing default values of enum constants
enum state {
    not_done,
    ong,
    done,
};

typedef struct Gardenzone {
    int Moisture_GPIO;      // Capacitivr sensor GPIO
    int AirValue;           // Capacitivr sensor config
    int WaterValue;         // Capacitivr sensor config
    int Moisture_Level;     // Current Moisture Level in %
    enum state water;
    int Pump_GPIO;          // Pump GPIO       
    int WATER_PUMP_TIME;    // ms
    int MaxMoistureLevel;   // in %
    int MinMoistureLevel;   // in %
} Gardenzone_t;


typedef struct Weather {
    float Temperature;      // Temperature in degree Celcius  
    float Presure;          // Presure in Pa  
    float Altitude;         // Altitude in mts
} Weathere_t;

/* -- Variables */
char version[] = "v2";  /** Version Control*/
unsigned long lastSync = millis();

bool moisture_done =false;

// init Moisture sensor struct
Gardenzone_t moi1 = { A0, 3547, 1429, 0, not_done, ZONE1_PUMP, 15000, 85 , 80 };    // Zone 1 Tomatoes
Gardenzone_t moi2 = { A1, 3040, 1121, 0, not_done, ZONE2_PUMP, 15000, 85 , 80 };    // Zone 2 Courgette
Gardenzone_t moi3 = { A2, 3670, 1573, 0, not_done, ZONE3_PUMP, 10000, 85 , 80 };    // Zone 3 Carrots
Gardenzone_t moi4 = { A3, 3058, 1143, 0, not_done, ZONE4_PUMP, 10000, 85 , 80 };    // Zone 4 Strawberries

struct Gardenzone *garden[] = { &moi1, &moi2, &moi3 , &moi4 };
enum { NB_ZONES = sizeof(garden) / sizeof(garden[0]) };

Weathere_t myStation;
MPL3115A2 myPressure;

#ifdef DEBUG_MODE
    SerialLogHandler logHandler;
#endif

/* Private function prototypes -----------------------------------------------*/
int get_moisture_level(Gardenzone *moi) ;	
float get_batt_level(void);
int get_water_level(void);
void get_weather(Weather *station);
int auto_water(void);
void check_auto_water(void);

Timer timer1(WATER_PUMP_DEFAULT, check_auto_water, true);
//Timer timer2(RESET_STATE_TIME, reset_state, true);

//particle serial monitor --follow

// Last time, we only needed to declare pins in the setup function.
// This time, we are also going to register our Particle function
void setup()
{
    // Configure barometric pressure sensor
    // Join i2c bus
    Wire.begin();
    // Start serial for output
    Serial.begin(9600);
    // Get sensor onlin
    myPressure.begin(); 

    // Configure sensor Mode
    // Measure pressure in Pascals from 20 to 110 kPa
    myPressure.setModeBarometer();
    // Set Oversample to the recommended 128
    myPressure.setOversampleRate(7); 
    // Enable all three pressure and temp event flags 
    myPressure.enableEventFlags();

    // Pumps GPIO Config
    pinMode(ZONE1_PUMP, OUTPUT);    // sets the digital pin as output
    pinMode(ZONE2_PUMP, OUTPUT);    // sets the digital pin as output
    pinMode(ZONE3_PUMP, OUTPUT);    // sets the digital pin as output
    pinMode(ZONE4_PUMP, OUTPUT);    // sets the digital pin as output
    pinMode(ZONE5_PUMP, OUTPUT);    // sets the digital pin as output

    // Water Level GPIO Config
    pinMode(D6, INPUT);    // sets the digital pin 6 as input
    pinMode(D7, INPUT);    // sets the digital pin 7 as input
    pinMode(D8, INPUT);    // sets the digital pin 8 as input

    // Led GPIO Config
    pinMode(LED_LOW_WATER, OUTPUT);    // sets the digital pin as output

    // Set Pumps to IDle
    digitalWrite(ZONE1_PUMP, HIGH);
    digitalWrite(ZONE2_PUMP, HIGH);
    digitalWrite(ZONE3_PUMP, HIGH);  
    digitalWrite(ZONE4_PUMP, HIGH); 
     

    // Particle functions for cloud 
    Particle.function("Data",getdata);

    // Particle.variable("analogvalueA0", &analogvalueA0, INT);
    // Particle.variable("analogvalueA1", &analogvalueA1, INT);
    // Particle.variable("analogvalueA2", &analogvalueA2, INT);
    // Particle.variable("analogvalueA3", &analogvalueA3, INT);
    // Particle.variable("analogvalueA4", &analogvalueA4, INT);

    // Start Daylight Saving Time
    Time.beginDST();
    // Set time zone to Central European Time
    Time.zone(+1);

    // Init Garden Moisture
    for (int i = 0; i < NB_ZONES; i++){  

        get_moisture_level(garden[i]);
 
    } 

}


void loop()
{

    if (millis() - lastSync > ONE_DAY_MILLIS) {
        // Request time synchronization from the Particle Device Cloud
        Particle.syncTime();
        waitUntil (Particle.syncTimeDone);
        lastSync = millis();
        
        // Reset Statue
        reset_state();
    }


    if ((Time.hour() == WATER_TIME) && (moisture_done == false)) {

         if (timer1.isActive()==false) {
             // Start Watering by Zones
             auto_water();
        }
    }

    // Check if low Water and advertise 
    if ((Time.hour() >= 21) && (get_water_level() == 0)) {
        digitalWrite(LED_LOW_WATER, HIGH); 
    }else if ((Time.hour() == 00) && (get_water_level() == 0)) {
        digitalWrite(LED_LOW_WATER, LOW);
    }else if (get_water_level() != 0){
        digitalWrite(LED_LOW_WATER, LOW);
    }

}

int getdata(String command) {
    /* Particle.functions always take a string as an argument and return an integer.
    Since we can pass a string, it means that we can give the program commands on how the function should be used.
    */

    if (command=="battery") {
        Particle.publish("Battery Voltage", String::format("%.2f V", get_batt_level()));
        return 0;
    }
    else if (command=="moisture") {
        Particle.publish("Moisture", String::format("M1 %d%%; M2 %d%%; M3 %d%%; M4 %d%%", get_moisture_level(garden[0]),get_moisture_level(garden[1]),get_moisture_level(garden[2]),get_moisture_level(garden[3])));
        return 0;
    }
    else if (command=="water") {
        Particle.publish("Water Level", String::format("%d  D8=%d D7=%d D6=%d", get_water_level(),digitalRead(D8),digitalRead(D7),digitalRead(D6)));
        return 0;
    }
    else if (command=="weather") {
        // Geat Weather station information
        get_weather(&myStation);
        Particle.publish("Weather", String::format("Temp %.2f C, Alt %.2f m, Pre %.0f Pa, Time %d:%d",myStation.Temperature,myStation.Altitude,myStation.Presure,Time.hour(),Time.minute() ));
        return 0;
    }else if (command=="set4") {
        // Geat Weather station information
        //moi1.analog = 320;
        digitalWrite(ZONE4_PUMP, LOW);  //1
        return 0;
    }else if (command=="reset4") {
        // Geat Weather station information
        //moi1.analog = 850;
        digitalWrite(ZONE4_PUMP, HIGH);  //1
        return 0;
    }else if (command=="set3") {
        // Geat Weather station information
        //moi2.analog = 330;
        digitalWrite(ZONE3_PUMP, LOW);  // 2
        return 0;
    }else if (command=="reset3") {
        // Geat Weather station information
        //moi2.analog = 860;
        digitalWrite(ZONE3_PUMP, HIGH);  // 2
        return 0;
    }else if (command=="set2") {
        // Geat Weather station information
        //moi3.analog = 340;
        digitalWrite(ZONE2_PUMP, LOW);  // 3
        return 0;
    }else if (command=="reset2") {
        // Geat Weather station information
        //moi4.analog = 870;
        digitalWrite(ZONE2_PUMP, HIGH);  // 3
        return 0;
    }else if (command=="set1") {
        // Geat Weather station information
        //moi4.analog = 350;
        digitalWrite(ZONE1_PUMP, LOW);  //4
        return 0;
    }else if (command=="reset1") {
        // Geat Weather station information
        //moi3.analog = 870;
        digitalWrite(ZONE1_PUMP, HIGH);  //4
        return 0;
    }else if (command=="status") {
        Particle.publish("Status", String::format("Watering 1-%d 2-%d 3-%d 4-%d ",garden[0]->water,garden[1]->water,garden[2]->water,garden[3]->water ));
        return 0;
    }else {
        Particle.publish("List CMD:", "battery;moisture;water;weather,status");
        return -1;
    }

}

void get_weather(Weather *station) {

    // Discard First Conversion
    myPressure.readTemp();
    myPressure.readPressure();

    // Read Preasure in PA and Temperature in C
    station->Presure = myPressure.readPressure();
    station->Temperature = myPressure.readTemp();

    // Calculate he altitude in meters with the international barometric formula:
    // H = 44330 * [1 - (P/p0)^(1/5.255) ]
    // H = altitude (m)
    // P = measured pressure (Pa) from the sensor
    // p0 = reference pressure at sea level (Pa)
    station->Altitude = 44330 * (1 - (pow((station->Presure/SEA_LEVEL_PRESSURE), (1/5.255))) );

}


float get_batt_level(void) {

    float battery_voltage=0;

    for (int i = 0; i < BATTERY_SAMPLES; ++i){
        
        battery_voltage = ((((analogRead(A4) *3.3 ) / 4030)  * 4.2) /3.3) + battery_voltage;
    }

    return  (battery_voltage/BATTERY_SAMPLES);
}


int get_moisture_level(Gardenzone *moi) {

    int moisture=0;

    for (int i = 0; i < MOISTURE_SAMPLES; ++i){

        moisture = analogRead(moi->Moisture_GPIO) + moisture ;
    }

    // (moisture/MOISTURE_SAMPLES)
    //moi->Moisture_Level= map(moi->analog, moi->AirValue, moi->WaterValue, 0, 100);
    moi->Moisture_Level= map((moisture/MOISTURE_SAMPLES), moi->AirValue, moi->WaterValue, 0, 100);

    return moi->Moisture_Level;
}


int get_water_level(void) {

    switch ( ((digitalRead(D8) << 2)|(digitalRead(D7) << 1)| (digitalRead(D6))) ){

        case 7:
            return 100;
        break;

        case 3:
            return 65;
        break;

        case 1:
            return 25;
        break;

        case 0:
            return 0;
        break; 

        default:
            return 0;
    }
}


int auto_water(void) {

    if(get_water_level() != 0){

        for (int i = 0; i < NB_ZONES; i++)
        {    
            #ifdef DEBUG_MODE
                Log.warn(String::format("On Zone = %d Moisture Level = %d and state = %d",i ,(garden[i]->Moisture_Level),(garden[i]->water)));
            #endif

            if((garden[i]->water) == not_done){

                if((garden[i]->Moisture_Level) <= garden[i]->MinMoistureLevel){
                    garden[i]->water = ong;

                    #ifdef DEBUG_MODE
                        Log.warn(String::format("Moisture less than 00 %%  on zone %d Start Watering",i));
                    #endif

                    digitalWrite(garden[i]->Pump_GPIO, LOW);
                    timer1.changePeriod(garden[i]->WATER_PUMP_TIME); // Reset period of timer to 
                    // start timer to cut off pum and check moisture
                    timer1.start();
                    break;            
        
                }else
                // Not done but moisture level is OK
                garden[i]->water = done;

                #ifdef DEBUG_MODE
                    Log.warn(String::format("Moisture Bigger than 80 %%  on zone %d  state = %d ",i,garden[i]->water));
                #endif

            }else if(i == (NB_ZONES-1)){

                #ifdef DEBUG_MODE
                    Log.warn("All Zones are Done Start Timer to reset state in 1hour ");
                #endif
                moisture_done = true;
                //timer2.start();
            }

        }
    
        return 0;
    }
    
    #ifdef DEBUG_MODE
        Log.warn("***** No Water *****");
    #endif
    moisture_done = true;
    //timer2.start();
    return -1;
}


void check_auto_water(void) {

    #ifdef DEBUG_MODE
        Log.warn("**** ON TIMER****");
    #endif

    for (int i = 0; i < NB_ZONES; i++){  

        #ifdef DEBUG_MODE
            Log.warn(String::format("Check water on zone=%d",i));
        #endif 

        // Loop on all Zones to see the find the Active One   
        if((garden[i]->water) == ong){
            // Stop Water Pump
            digitalWrite(garden[i]->Pump_GPIO, HIGH);
            // Update Moisture Level
            get_moisture_level(garden[i]);

            // Check Soil Moisture
            if((garden[i]->Moisture_Level) >= garden[i]->MaxMoistureLevel){
                // Water is done
                garden[i]->water = done;
                #ifdef DEBUG_MODE
                    Log.warn(String::format("Moisture Level %d ON interrup is bigger than 80 %% state = %d",garden[i]->Moisture_Level,garden[i]->water));
                #endif         
            }else{
                // Water not enough
                // To Be changed
                garden[i]->water = done;
                #ifdef DEBUG_MODE
                    Log.warn(String::format("Moisture Level %d ON interrup is smaller than 80 %% state = %d",garden[i]->Moisture_Level,garden[i]->water)); 
                #endif                     
            }
            break;
        }   
    }     
}

void reset_state(void) {

    moisture_done = false;

    for (int i = 0; i < NB_ZONES; i++)
    {  
        garden[i]->water = not_done;
    }
        
}

