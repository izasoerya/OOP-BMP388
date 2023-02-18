#ifndef telemetry_h
#define telemetry_h

#include <Arduino.h>
#include "component.h"
extern float accelX,accelY,temp,press,altit,lat,lng;
float sim_press;
enum State {
    STANDBY     =  0,
    ASCENT      =  1,
    SEPARATION  =  2,
    DESCENT     =  3,
    HS_RELEASE  =  4,
    PP_RELEASE  =  5,
    LANDED      =  6,
    INVALID     = -1
};

    char mode = 'F';
    char HS_DEPLOYED = 'N';
    char PC_DEPLOYED = 'N';
    char MAST_RAISED = 'N';
    byte State;
    String cmdEcho = "kntl";
    
    /* Pressure sensor */
    float altitude=altit; char str_altitude[10]; //dtostrf(altitude, 1, 1, str_altitude);
    float temperature=temp; char str_temperature[10]; //dtostrf(temperature, 1, 1, str_temperature);
    float pressure=press; char str_pressure[10]; //dtostrf(pressure, 1, 1, str_pressure);

    /* IMU */
    float tiltX=accelX, tiltY=accelY;char str_x[20];char str_y[20];
    //dtostrf(tiltX, 1, 2, str_x); dtostrf(tiltY, 1, 2, str_y);

    /* Voltage divider */
    float voltage; char str_voltage[10]; //dtostrf(voltage, 1, 2, str_voltage);

    /* GPS */
    int hhGPS ;
    int mmGPS ;
    int ssGPS ;
    float latitudeGPS;char str_lat[20];// dtostrf(latitudeGPS, 1, 8, str_lat);
    float longitudeGPS;char str_lng[20];// dtostrf(longitudeGPS, 1, 8, str_lng);
    float altitudeGPS;char str_altigps[10];// dtostrf(altitudeGPS, 1, 1, str_altigps);
    int satsGPS;
    
    unsigned long int packetCount = 0;
    const char teamID[5] = "1084";
    bool tele_command=false,tele_calibration=false,tele_enable=false,tele_sim=false,tele_activate=false;
    const char* getState() {
        //int rand = random(0,4);
        switch(State){
            case STANDBY:
                return "STANDBY";
            case ASCENT:
                return "ASCENT";
            case SEPARATION:
                component().begin_pinservo(3);
                component().servo_run(3);
                return "SEPARATION";
            case DESCENT:
                return "DESCENT";
            case HS_RELEASE:
                component().begin_pinservo(4);
                component().servo_run(4);
                return "HS_RELEASE";
            case PP_RELEASE:
                component().begin_pinservo(5);
                component().servo_run(5);
                return "PP_RELEASE";
            case LANDED:
                component().begin_pinbuzzer(6);
                component().begin_pinflag(7);
                component().buzzer_run(6);
                component().flag_run(7);
                return "LANDED";
            default:
                return "INVALID";
        }
    }

class telemetry {
    private:
    public:
    telemetry(){};
    void state(int condition) {
        State = condition;
    }
    void tele_readcomm(String head, String label, String mode, String content) {
    Serial.begin(9600);
    if (String(head+label+mode+content)=="CMD1084CXON"){ //kalo di string ada kode maka do something
    tele_command = true;Serial.println("ON DONE!");cmdEcho = "CXON";}
    if (String(head+label+mode+content)=="CMD1084CXCAL"){ //kalo di string ada kode maka do something
    tele_calibration = true;Serial.println("CALI ON!");cmdEcho = "CAL";
    }
    if (String(head+label+mode+content)=="CMD1084SIMACTUVATE"){ //kalo di string ada kode maka do something
    tele_activate = true;Serial.println("ACTIVATE DONE!");cmdEcho = "SIMACTIVATE";
    }
    if (String(head+label+mode+content)=="CMD1084SIMENABLE"){ //kalo di string ada kode maka do something
    tele_enable = true;Serial.println("ENABLE DONE!");cmdEcho = "SIMENABLE";
    }
    if (String(head+label+mode+content)=="CMD1084SIMDISABLE"){ //kalo di string ada kode maka do something
    tele_enable = false,tele_sim=false,tele_activate=false;Serial.println("SIM DONE!");cmdEcho = "SIMDISABLE";
    }
    if (String(head+label+mode)=="CMD1084SIMP"&&tele_enable==true&&tele_activate==true){ //kalo di string ada kode maka do something
    tele_sim = true;sim_press = content.toFloat();Serial.println("PRESS DONE!");
    }
    }
    void distort (float a,float t,float p,float x,float y,float v, int j, int m, int d, float lat, float lng, float gps_alti, int satelite) {
    memset(str_altitude, 0, sizeof(str_altitude));
    memset(str_temperature, 0, sizeof(str_temperature));
    memset(str_pressure, 0, sizeof(str_pressure));
    memset(str_x, 0, sizeof(str_x));
    memset(str_y, 0, sizeof(str_y));
    memset(str_voltage, 0, sizeof(str_voltage));
    dtostrf(a, -4, 1, str_altitude);
    dtostrf(t, 3, 1, str_temperature);
    dtostrf(p, 6, 1, str_pressure);
    dtostrf(x, 4, 2, str_x);
    dtostrf(y, 4, 2, str_y);
    dtostrf(v, 3, 2, str_voltage);
    dtostrf(lat, 3, 6, str_lat);
    dtostrf(lng, 3, 6, str_lng);
    dtostrf(gps_alti, -4, 1, str_altigps);
    hhGPS = j;
    mmGPS = m;
    ssGPS = d;
    satsGPS = satelite;
    }
    String constructMessage() {
    memset(buffer, 0, sizeof(buffer));
    snprintf(buffer, sizeof(buffer),
              "%s,"
              "%02d:%02d:%02d,"
              "%ld,"
              "%c,%s,%s,"
              "%c,%c,%c,"
              "%s,%s,%s,"
              "%02d:%02d:%02d,"
              "%s,%s,%s,"
              "%02d,%s,%s,%s\r",
              teamID,
              hhGPS, mmGPS, ssGPS,
              packetCount,
              mode, getState(), str_altitude,
              HS_DEPLOYED, PC_DEPLOYED, MAST_RAISED,
              str_temperature, str_pressure, str_voltage,
              hhGPS, mmGPS, ssGPS,
              str_altigps, str_lat, str_lng,
              satsGPS, str_x, str_y, cmdEcho.c_str()
            );

        return String(buffer);
  };
  char buffer[256];
};

#endif