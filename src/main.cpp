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
TaskHandle_t TaskGPS_Handler;
TaskHandle_t TaskEPROM_Handler;
TaskHandle_t TaskSIM_Handler;
void SENSOR_S(void *pvParameters );
void PRINTER_S(void *pvParameters );
void GPS_S(void *pvParameters );
void EPROM_SD(void *pvParameters );
void SIMULATOR(void *pvParameters );

extern unsigned long packetCount; extern bool tele_command, tele_calibration, tele_enable, tele_sim;extern float sim_press;
float accelX,accelY,gForce,l_gforce,accelZ,value_roll,value_pitch,c,temp=0,press,altit,last_altit=0,ref,lat,lng,eprom,voltase=5.0,gps_altitude;    //MPU, BME, GPS, EEPROM
int packet[3] = {0,0,0},time[7],gps_satelite,paket_xbee=0,error; bool var_sim;    //GPS
int no=0,i; int n ; String ayaya[100]; int k=0,state; String hasil, tele; char tampung; bool lock=false;    //PARSING

void setup() {
  Serial.begin(9600);
  while(!Serial){;}     //make sure program start after serial is open
  Serial3.begin(9600);
  Serial2.begin(9600);
  mpu.begin();   
  bmed.begin();
  temp = bmp.temperature;
  while (temp<25) {
    temp = bmed.read_temp();
    press = bmed.read_press();
    altit = bmed.read_altitude(1023.5);
  }
  ref = bmp.pressure/100.0;
  pinMode(5, OUTPUT);     //hanya tes program run atau tidak
  xTaskCreate(SENSOR_S, "Task2", 512, NULL, 4, &TaskSENSOR_Handler);    //func bme,mpu
  xTaskCreate(PRINTER_S, "Task3", 1024, NULL, 5, &TaskPRINTER_Handler);  //func serial print
  xTaskCreate(GPS_S, "Task4", 512, NULL, 3, &TaskGPS_Handler);          //func gps
  xTaskCreate(EPROM_SD, "Task5", 512, NULL, 2, &TaskEPROM_Handler);      //func EEPROM
  xTaskCreate(SIMULATOR, "Task6", 512, NULL, 1, &TaskSIM_Handler);      //func EEPROM
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}

void SENSOR_S (void *pvParameters) {
  (void) pvParameters;  //ga penting, dihapus juga boleh
  l_gforce = mpu.readGforce();
  while (1) {
  if (isnan(bmed.read_altitude(ref))) {      //detect bme nyambung atau ga
  bmed.begin();vTaskDelay( 1 / portTICK_PERIOD_MS );  //kalo ga nyambung coba .begin biar jalan lagi
  }else {  // kalo nyambung baca datanya
  temp = bmed.read_temp();
  press = bmed.read_press();
  if (tele_calibration==true) {
    ref = bmed.read_press();
    bmed.tele_calibration(ref);tele_calibration=false;
  }
  if (tele_sim==true) {
 //   if (altit>0) {
    altit = bmed.read_altitude_sim(sim_press);
//    }
 //   else {altit=-0;}
  }
  else {
//    if(bmed.read_altitude(ref)>=0) {  if(bmed.read_altitude(ref)>=600) {digitalWrite(5,HIGH);}
    altit = bmed.read_altitude(ref);
 //   } else {altit = 0;}
  }
  }
//  error = mpu.error_cek();
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
  telemetry().detect_mode(tele_sim);
  telemetry().detect_state(packetCount,gForce,press,altit,last_altit);
  last_altit = altit;
  l_gforce = gForce;
  vTaskDelay( 10 / portTICK_PERIOD_MS );  //baca sensor tiap 1 ms biar ga menuhin buffer, tapi nilai di detik 
  }                                      //1 sj yang dipakai biar up to date (*tanya aja nek bingung maksudnya)
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

void GPS_S (void *pvParameters) {
  (void) pvParameters;
  while(1) {
  while(Serial3.available()>0) {    //kalo ada gps , baca 
    if (gps.encode(Serial3.read()))  //trs di encode pake tiny gps+
      displayInfo();      //proses hasil encodenya
    }
    while(!Serial3.available()) {
      vTaskDelay( 1 / portTICK_PERIOD_MS );  //kalo gada gps, do else tiap 1 mili second
    } 
  }
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
  telemetry().distort(altit,temp,press,tiltX,tiltY,voltase,time[3],time[4],time[5],lat,lng,gps_altitude,gps_satelite);
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
  packetCount++;
//  Serial.println(gForce);
  }
  vTaskDelay( 1000 / portTICK_PERIOD_MS );
  }
}



/* note : 
1. GPS sekarang masih nge print data kalau blm ngelock biar jml packet nya setara sama lainnya
   kalau mau ga di print juga bisa ngikut aku
2. EEPROM masih belum nemu cara biar tiap restart MCU ga overwrite data, tapi harusnya bisa 
   cek lokasi address ada isinya apa ga, kalau ada jangan write
3. Kalau mau pake XBEE tinggal ganti serial.print di void print_s jadi Serial nya XBEE
4. MPU dan BME sudah redundant (reset kalo hilang koneksi, trs nyambung lg kalo tiba2 nyambung)
   kalau ad sensor disconect juga program tetap jalan 
5. Bacaan antar serial ada delay 10 ms, masalah ga ini? 
6. Semua delay (VtaskDelay) cara kerjanya yaitu nge buat fungsi sleep jadi ttp bsia jalanin program lain
7. To Do list : 
    - Nyoba ngirim data lewat XBEE dari komputer 1 ke komputer lain yg nyambung arduino
    - FSW nya parameternya apa aja? perlu diskusi lagi karena kondisi di udara != sekarang
    - Coba jalanin programnya sampe +- 2 jam buat cek masih jalan apa ga soalnya RTOS punya masalah buffer
    - Diskusi apakah perlu filter di tiap sensor, nge rata2 pembacaan, kalman dll. 
      kalo aku sih pake built in cukup
    - 
*/