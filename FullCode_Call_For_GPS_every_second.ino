#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_asukiaaa.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Pins
#define SD_CS 10       // Chip Select for SD card
#define APC_RX 9       // APC220 TX → Arduino RX
#define APC_TX 7       // APC220 RX → Arduino TX
#define BUZZER_PIN 8
#define CALLSIGN "AERONITE_UPNM"    //callsign

// Objects
Adafruit_BMP280 bmp;
MPU9250_asukiaaa mySensor;
TinyGPSPlus gps;
SoftwareSerial apcSerial(APC_RX, APC_TX);    // APC220 port

// Scheduler
unsigned long lastSend = 0;
unsigned long packetCounter = 0;

// SD card state
bool sdAvailable = true;
unsigned long lastSDCheck = 0;
const unsigned long sdRetryInterval = 5000; // retry every 5s

// -------------------- Buzzer ---------------------
void fail_tone(){
  tone(BUZZER_PIN,1000,300);
  delay(400);
  tone(BUZZER_PIN,800,200);
  delay(300);
  tone(BUZZER_PIN,600,200);
  delay(300);
  tone(BUZZER_PIN,500,200);
  delay(300);
  tone(BUZZER_PIN,400,200);
  delay(300);
  noTone(BUZZER_PIN);
}

void start_up_tone(){
  tone(BUZZER_PIN,660,100); delay(150); noTone(BUZZER_PIN);
  tone(BUZZER_PIN,660,100); delay(300); noTone(BUZZER_PIN);
  tone(BUZZER_PIN,660,100); delay(300); noTone(BUZZER_PIN);
  tone(BUZZER_PIN,510,100); delay(150); noTone(BUZZER_PIN);
  tone(BUZZER_PIN,660,100); delay(300); noTone(BUZZER_PIN);
  tone(BUZZER_PIN,770,100); delay(550); noTone(BUZZER_PIN);
  tone(BUZZER_PIN,380,100); delay(575); noTone(BUZZER_PIN);
}

void success_tone(){
  tone(BUZZER_PIN,500,200); delay(250); noTone(BUZZER_PIN);
  tone(BUZZER_PIN,500,200); delay(250); noTone(BUZZER_PIN);
  tone(BUZZER_PIN,500,200); delay(250); noTone(BUZZER_PIN);
  tone(BUZZER_PIN,800,150); delay(200); noTone(BUZZER_PIN);
}

void countdown_success(){
  Serial.println("Countdown to send packet in");
  apcSerial.println("Countdown to send packet in");

  for (int i = 10; i >= 1; i--) {
    Serial.println(i);
    apcSerial.println(i);
    tone(BUZZER_PIN, 1000, 120);
    delay(200);
    noTone(BUZZER_PIN);
    delay(200);
  }

  Serial.println();
  apcSerial.println();
  Serial.println("-------------------------------------------------------------------");
  apcSerial.println("-------------------------------------------------------------------");
  Serial.println("||                        GO AERONITE UPNM!                      ||");
  apcSerial.println("||                        GO AERONITE UPNM!                      ||");
  Serial.println("-------------------------------------------------------------------");
  apcSerial.println("-------------------------------------------------------------------");

  String header = "Callsign,Packet_No,Date_UTC,Time_UTC,Time_MYT,Latitude,Longitude,GPS_Altitude_m,Satellites,Temp_C,Pressure_atm,Baro_Altitude_m,Accel_X_mps2,Accel_Y_mps2,Accel_Z_mps2,Accel_res_mps2,Gyro_X_dps,Gyro_Y_dps,Gyro_Z_dps,Mag_X_uT,Mag_Y_uT,Mag_Z_uT";
  Serial.println(header);
  apcSerial.println(header);

  tone(BUZZER_PIN, 1500, 900); delay(950); noTone(BUZZER_PIN);
}

// -------------------- GPS Helper --------------------
void readGPS() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(BUZZER_PIN, OUTPUT);
  Wire.begin();

  start_up_tone();

  // Initialize BMP280
  Serial.println("Initializing BMP280...");
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not detected at 0x76. Check wiring!");
    fail_tone();
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  Serial.println("BMP280 OK");

  // Initialize MPU9250
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  delay(500);
  Serial.println("MPU9250 initialized.");

  // Initialize GPS
  Serial1.begin(9600);
  Serial.println("GPS module serial started (using Serial1)...");

  // Initialize APC220
  apcSerial.begin(9600);
  Serial.println("APC220 ready...");

  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card failed at startup!");
    fail_tone();
    sdAvailable = false;   // mark unavailable, will retry later
    lastSDCheck = millis();
  } else {
    Serial.println("SD card ready.");
    // Write CSV header
    File dataFile = SD.open("data.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Callsign,Packet_No,Date_UTC,Time_UTC,Time_MYT,Latitude,Longitude,GPS_Altitude_m,Satellites,Temp_C,Pressure_atm,Baro_Altitude_m,Accel_X_mps2,Accel_Y_mps2,Accel_Z_mps2,Accel_res_mps2,Gyro_X_dps,Gyro_Y_dps,Gyro_Z_dps,Mag_X_uT,Mag_Y_uT,Mag_Z_uT");
      dataFile.close();
    }
  }

  // Print header
  String header = "Callsign,Packet_No,Date_UTC,Time_UTC,Time_MYT,Latitude,Longitude,GPS_Altitude_m,Satellites,Temp_C,Pressure_atm,Baro_Altitude_m,Accel_X_mps2,Accel_Y_mps2,Accel_Z_mps2,Accel_res_mps2,Gyro_X_dps,Gyro_Y_dps,Gyro_Z_dps,Mag_X_uT,Mag_Y_uT,Mag_Z_uT";
  Serial.println(header);
  apcSerial.println(header);

  success_tone();
  delay(300);
  countdown_success();
}

// -------------------- Loop --------------------
void loop() {
  // Always feed GPS at the start
  readGPS();

  unsigned long now = millis();
  if (now - lastSend >= 1000) {
    lastSend = now;
    packetCounter++;

    // Feed GPS again before using values
    readGPS();

    // Prepare date/time strings
    char timeBuffer[20] = "Invalid";
    if (gps.time.isValid()) {
      sprintf(timeBuffer, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    }

    char mytBuffer[20] = "Invalid";
    if (gps.time.isValid()) {
      int hourMYT = (gps.time.hour() + 8) % 24;
      sprintf(mytBuffer, "%02d:%02d:%02d", hourMYT, gps.time.minute(), gps.time.second());
    }

    char dateBuffer[20] = "Invalid";
    if (gps.date.isValid()) {
      sprintf(dateBuffer, "%02d/%02d/%04d", gps.date.day(), gps.date.month(), gps.date.year());
    }

    // GPS values
    static double latitude = 0.0, longitude = 0.0, gpsAlt = -1.0;
    static int satCount = 0;

// Only update if new data came in
    if (gps.location.isUpdated()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
}

    if (gps.altitude.isUpdated()) gpsAlt = gps.altitude.meters();
    if (gps.satellites.isUpdated()) satCount = gps.satellites.value();


    // Sensors
    mySensor.accelUpdate();  readGPS();
    mySensor.gyroUpdate();   readGPS();
    mySensor.magUpdate();    readGPS();

    float temp = bmp.readTemperature();
    float pressure_pa = bmp.readPressure();
    float pressure_atm = pressure_pa / 101325.0;
    float altitude = bmp.readAltitude(1012.13);

    float ax = mySensor.accelX()*9.80665, ay = mySensor.accelY()*9.80665, az = mySensor.accelZ()*9.80665;
    float a_res = sqrt(ax*ax + ay*ay + az*az);
    float gx = mySensor.gyroX(), gy = mySensor.gyroY(), gz = mySensor.gyroZ();
    float mx = mySensor.magX(), my = mySensor.magY(), mz = mySensor.magZ();

    // Build CSV line
    String dataLine = String(CALLSIGN) + "," +
                      String(packetCounter) + "," + 
                      String(dateBuffer) + "," +
                      String(timeBuffer) + "," +
                      String(mytBuffer) + "," +
                      String(latitude, 6) + "," +
                      String(longitude, 6) + "," +
                      String(gpsAlt) + "," +
                      String(satCount) + "," +
                      String(temp) + "," +
                      String(pressure_atm, 6) + "," +
                      String(altitude) + "," +
                      String(ax, 6) + "," +
                      String(ay, 6) + "," +
                      String(az, 6) + "," +
                      String(a_res, 6) + "," +
                      String(gx, 6) + "," +
                      String(gy, 6) + "," +
                      String(gz, 6) + "," +
                      String(mx, 6) + "," +
                      String(my, 6) + "," +
                      String(mz, 6);

    // 1) Always send telemetry
    Serial.println(dataLine);
    apcSerial.println(dataLine);

    // 2) SD logging (non-blocking with retry cooldown)
    if (sdAvailable) {
      File dataFile = SD.open("data.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.println(dataLine);
        dataFile.close();
      } else {
        Serial.println("Error writing to SD, disabling until retry...");
        sdAvailable = false;
        lastSDCheck = now;
      }
      readGPS(); // keep feeding GPS while handling SD
    } else {
      if (now - lastSDCheck >= sdRetryInterval) {
        Serial.println("Retrying SD card...");
        if (SD.begin(SD_CS)) {
          Serial.println("SD card re-detected, resuming logging.");
          sdAvailable = true;
          // if file missing, write header again
          if (!SD.exists("data.csv")) {
            File dataFile = SD.open("data.csv", FILE_WRITE);
            if (dataFile) {
              dataFile.println("Callsign,Packet_No,Date_UTC,Time_UTC,Time_MYT,Latitude,Longitude,GPS_Altitude_m,Satellites,Temp_C,Pressure_atm,Baro_Altitude_m,Accel_X_mps2,Accel_Y_mps2,Accel_Z_mps2,Accel_res_mps2,Gyro_X_dps,Gyro_Y_dps,Gyro_Z_dps,Mag_X_uT,Mag_Y_uT,Mag_Z_uT");
              dataFile.close();
            }
          }
        } else {
          Serial.println("Retry: SD still not detected.");
          lastSDCheck = now;
        }
        readGPS();
      }
    }
  }
}
