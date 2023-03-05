#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include "mpumine.h"
#include "bmp.h"
#include "telemetry.h"
#include "component.h"
#define CS 10
#define xbee Serial2

Mpumine mpu(Wire1);
bmp_read bmed;
TinyGPSPlus gps;
File myFile;

TaskHandle_t TaskSENSOR_Handler;
TaskHandle_t TaskPRINTER_Handler;
TaskHandle_t TaskEPROM_Handler;
TaskHandle_t TaskSIM_Handler;
void SENSOR_S(void *pvParameters );
void PRINTER_S(void *pvParameters );
void EPROM_SD(void *pvParameters );
void SIMULATOR(void *pvParameters );

extern unsigned long packetCount; extern bool tele_command, tele_calibration, tele_enable, tele_sim;extern float sim_press;
float accelX,accelY,gForce,l_gforce,accelZ,value_roll,value_pitch,c,temp=0,press,altit,last_altit=0,ref,lat,lng,eprom,voltase=5.0,gps_altitude;    //MPU, BME, GPS, EEPROM
int packet[3] = {0,0,0},time[7],gps_satelite,paket_xbee=0,error; bool var_sim;    //GPS
int no=0,i,sensor_counter=0; int n ; String ayaya[100]; int k=0,state; String hasil, tele; char tampung; bool lock=false;    //PARSING

void setup() {
  Serial.begin(9600);
  while(!Serial){;}     //make sure program start after serial is open
  Serial3.begin(9600);
  Serial2.begin(115200);
  mpu.begin();   
  bmed.begin();
  temp = bmed.read_temp();
  /*Dummy Data*/
  while (temp<25) {
    temp = bmed.read_temp();
    press = bmed.read_press();
    altit = bmed.read_altitude(1023.5);
  }
  /*Dummy Data END*/
  ref = bmp.pressure/100.0;
  pinMode(5, OUTPUT);     //hanya tes program run atau tidak
  digitalWrite(5, HIGH);
  xTaskCreate(SENSOR_S, "Task2", 512, NULL, 3, &TaskSENSOR_Handler);    //func bme,mpu
  xTaskCreate(PRINTER_S, "Task3", 1024, NULL, 4, &TaskPRINTER_Handler);  //func serial print        
  xTaskCreate(EPROM_SD, "Task5", 512, NULL, 2, &TaskEPROM_Handler);      //func EEPROM
  xTaskCreate(SIMULATOR, "Task6", 512, NULL, 1, &TaskSIM_Handler);      //func EEPROM
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}

void displayInfo() {
  if (gps.location.isValid()) {   // cek valid location apa tidak (kalo valid brati dah nge lock)
  lat = gps.location.lat();
  lng = gps.location.lng();
  }
  if (gps.date.isValid()) {   // cek valid tanggal apa tidak (kalo valid brati dah nge lock)
  time[0] = gps.date.month();//EEPROM.put(1, time[0]);
  time[1] = gps.date.day();//EEPROM.put(2, time[1]);
  time[2] = gps.date.year();//EEPROM.put(3, time[2]);
  }
  if (gps.time.isValid()) {   // cek valid waktu apa tidak (kalo valid brati dah nge lock)
  time[3] = gps.time.hour();//EEPROM.put(4, time[3]);
  time[4] = gps.time.minute();//EEPROM.put(5, time[4]);
  time[5] = gps.time.second();//EEPROM.put(6, time[5]);
  time[6] = gps.time.centisecond();//EEPROM.put(7, time[6]);
  }
  if (gps.altitude.isValid()) {
  gps_altitude = gps.altitude.meters();
  }
  if (gps.satellites.isValid()) {
  gps_satelite = gps.satellites.value();
  }
}

void SENSOR_S (void *pvParameters) {
  (void) pvParameters;  //ga penting, dihapus juga boleh
  mpu.update_sens();
  l_gforce = mpu.readGforce();
  while (1) {

  /*GPS READ*/
  while(Serial3.available()>0) {    //kalo ada gps , baca 
    if (gps.encode(Serial3.read()))  //trs di encode pake tiny gps+
      displayInfo();      //proses hasil encodenya
      mpu.update_sens();
  }
  while(!Serial3.available()) {
      vTaskDelay( 1 / portTICK_PERIOD_MS );  //kalo gada gps, do else tiap 1 mili second
      mpu.update_sens();
  } 
  /*GPS READ END*/

  /*BMP READ*/
  bmed.bmp_error(ref);
  temp = bmed.read_temp();
  press = bmed.read_press();
  if (tele_calibration==true) {
    ref = bmed.read_press();
    bmed.tele_calibration(ref);
    tele_calibration=false;
  }
  if (tele_sim==true) {
    altit = bmed.output_bmp(bmed.read_altitude_sim(sim_press));
  }
  else {
    altit = bmed.output_bmp(bmed.read_altitude(ref));
  }
  /*BMP READ END*/

//  error = mpu.error_cek();

/*MPU READ*/
  if (error!=0||(mpu.readacc_x()&&mpu.readacc_y()&&mpu.readacc_z())==0) { //kalau tidak nyambung nilai error !=0 sama nilai xyz == 0
    mpu.begin();vTaskDelay( 10 / portTICK_PERIOD_MS ) ;//coba .begin biar jalan lagi
  }else { //kalau jalan baca data
  accelX = mpu.readacc_x();
  accelY = mpu.readacc_y();
  accelZ = mpu.readacc_z();
  value_roll = mpu.read_tiltx();
  value_pitch = mpu.read_tilty();
  gForce = (mpu.readGforce()+l_gforce)/2;
  }
  /*MPU READ END*/

  /*DETECTION*/
  telemetry().detect_mode(tele_sim);
  telemetry().detect_state(packetCount,gForce,press,altit,last_altit);
  last_altit = altit;
  l_gforce = gForce;sensor_counter++;
  vTaskDelay( 10 / portTICK_PERIOD_MS );  //baca sensor tiap 1 ms biar ga menuhin buffer, tapi nilai di detik 
  }                                      //1 sj yang dipakai biar up to date (*tanya aja nek bingung maksudnya)
  /*DETECTION END*/
}

void parsing() {
  n = hasil.length();     //read panjang string
  char inChar[n+1];   //buat variabel penampung string
  strcpy(inChar, hasil.c_str());   //convert string to array of char
  for ( i=0;i<n;i++) {    //baca masing-masing karakter
    if (inChar[i]==','){  //kalo ada tanda (,) hilangkan
      k++; //menghilangkan (,) sekaligus menuju array berikutnya
    }
    else {
      if(inChar[i] >= 30 && inChar[i] <= 122) {
        ayaya[k]+=inChar[i];  //kalo gada(,), tulis isinya
      }
    }
  };
  //Serial.print(ayaya[0]);
  telemetry().tele_readcomm(ayaya[0], ayaya[1], ayaya[2], ayaya[3]);
  for (i=0;i<n;i++) { //kosongkan kembali array
    ayaya[i] = "";  //hapus isi array
  }
  k=0;
  lock=false;
}


void SIMULATOR(void *pvParameters) {
  (void) pvParameters;
  while (1) {
  /* Telemetry Format */
  telemetry().distort(altit,temp,press,value_pitch,value_roll,voltase,time[3],time[4],time[5],lat,lng,gps_altitude,gps_satelite);
  tele = telemetry().constructMessage();
  tele.replace(" ", "");
  /* Parsing Command */
  hasil = "";
  while(Serial2.available()) {
    tampung=(char)Serial2.read();
    hasil+=tampung;
    lock=true;
  }
  if(lock==true) {parsing();}  
  vTaskDelay(300 / portTICK_PERIOD_MS );
  }
}

void EPROM_SD (void *pvParameters) {   //*buat EEPROM butuh cara untuk avoid overwrite eeprom
  (void) pvParameters;
  SD.begin(CS); //init SD card sesuai pin CS
  myFile = SD.open("1088.csv", FILE_WRITE);
  myFile.println("TeamID,Date,Count,Mode,State,Altitude,HS,PC,MAST,Temperature,Pressure,Voltage,DateGPS,AltiGPS,LAT,LNG,Satelite,TILTX,TILTY,ECHO");
  myFile.close();
  while(1) {
  myFile = SD.open("1088.csv", FILE_WRITE);  //open notepad buat isi data
  if (myFile) {
    myFile.print(tele);
    myFile.close();   //close notepad
  }                   //bisa buka file di SD card atau print di serial hasilnya
  vTaskDelay( 1000/ portTICK_PERIOD_MS );
  }
}

void PRINTER_S (void *pvParameters) {  //serial print buat semua sensor dkk (telemetrinya)
  (void) pvParameters;
  while (1) {
  if (tele=="") {;}
  else {
  Serial.println(tele);
  if (tele_command==true) {
    Serial2.print(tele);
  }
  packetCount++;
  }
  vTaskDelay( 1000 / portTICK_PERIOD_MS );
  }
}


/*TESTING CLONE LENOVO*/ 