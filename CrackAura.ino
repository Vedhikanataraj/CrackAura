#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <time.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>

// Real hardware serial, captured BEFORE it gets shadowed below.
// Must come before the #undef/#define block or this will be dummied out too.
static HardwareSerial &RealSerial = Serial;

// Serial Monitor: IP address only
// All existing Serial.print/println calls are disabled without changing system logic.
#define SerialMonitor Serial
#undef Serial
#define Serial NoSerialOutput

class NoSerialOutputClass {
public:
  void begin(unsigned long) {}
  void print(...) {}
  void println(...) {}
};
NoSerialOutputClass NoSerialOutput;

#define WIFI_SSID "your wifi"
#define WIFI_PASSWORD "your password"
#define WIFI_CONNECT_TIMEOUT_MS 12000

#define PIN_SDA 21
#define PIN_SCL 22
#define MPU6050_ADDR 0x68

#define PIN_SOIL_MOISTURE 34
#define PIN_VIBRATION 35
#define VIBRATION_ACTIVE_HIGH 1

#define PIN_RELAY_1 26
#define PIN_RELAY_2 27
#define PIN_STATUS_LED 2
#define RELAY_ACTIVE_LOW 1
#define SOLENOID_RELAY_CHANNEL 2

#define SAMPLE_RATE_HZ 200
#define SAMPLE_PERIOD_MS 5
#define TAPS_PER_TEST 5
#define DEFAULT_TAP_THRESHOLD_G 0.35f
#define TAP_REFRACTORY_MS 650
#define BASELINE_SAMPLES 150
#define CAPTURE_SAMPLES 128
#define CAPTURE_WINDOW_MS (CAPTURE_SAMPLES * SAMPLE_PERIOD_MS)
#define FILTER_WIDTH 5
#define MAX_HISTORY_TESTS 50

#define MOISTURE_DRY 3000
#define MOISTURE_WET 1200

#define VIBRATION_LOW_MAX_G 0.30f
#define VIBRATION_MEDIUM_MAX_G 0.70f

#define TESTS_DIR "/tests"
#define TESTS_INDEX_FILE "/tests/index.json"
#define SETTINGS_FILE "/settings.json"
#define COUNTER_FILE "/counter.txt"

#define GMT_OFFSET_SEC 19800
#define DAYLIGHT_OFFSET_SEC 0

#define MPU_RAW_ADDR      0x68
#define REG_PWR_MGMT_1    0x6B
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_SMPLRT_DIV    0x19
#define REG_ACCEL_XOUT_H  0x3B

#define ACCEL_SENS_LSB_PER_G   8192.0f   // +/-4g range
#define GYRO_SENS_LSB_PER_DPS  65.5f     // +/-500 dps range

bool mpu6500WriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_RAW_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpu6500Init() {
  bool ok = true;
  ok &= mpu6500WriteReg(REG_PWR_MGMT_1, 0x00);   // wake the chip
  delay(50);
  ok &= mpu6500WriteReg(REG_CONFIG, 0x03);        // DLPF ~44Hz
  ok &= mpu6500WriteReg(REG_GYRO_CONFIG, 0x08);   // +/-500 dps
  ok &= mpu6500WriteReg(REG_ACCEL_CONFIG, 0x08);  // +/-4g
  ok &= mpu6500WriteReg(REG_SMPLRT_DIV, 0x00);
  return ok;
}

// Reads accel (m/s^2), gyro (rad/s), and temperature (degC).
bool mpu6500Read(float &axMs2, float &ayMs2, float &azMs2,
                  float &gxRad, float &gyRad, float &gzRad,
                  float &tempC) {
  Wire.beginTransmission(MPU_RAW_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom(MPU_RAW_ADDR, 14, true);
  if (Wire.available() < 14) return false;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  int16_t rawTemp = (Wire.read() << 8) | Wire.read();
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  float axG = rawAx / ACCEL_SENS_LSB_PER_G;
  float ayG = rawAy / ACCEL_SENS_LSB_PER_G;
  float azG = rawAz / ACCEL_SENS_LSB_PER_G;

  axMs2 = axG * 9.80665f;
  ayMs2 = ayG * 9.80665f;
  azMs2 = azG * 9.80665f;

  float gxDps = rawGx / GYRO_SENS_LSB_PER_DPS;
  float gyDps = rawGy / GYRO_SENS_LSB_PER_DPS;
  float gzDps = rawGz / GYRO_SENS_LSB_PER_DPS;

  gxRad = gxDps * PI / 180.0f;
  gyRad = gyDps * PI / 180.0f;
  gzRad = gzDps * PI / 180.0f;

  tempC = (rawTemp / 333.87f) + 21.0f;

  return true;
}

WebServer server(80);

bool mpuReady=false,testRunning=false,testComplete=false,captureActive=false;
bool relay1State=false,relay2State=false;
String currentTestId="",currentBuilding="Building A",currentWall="Wall 1",lastStatusText="IDLE";
float tapThresholdG=DEFAULT_TAP_THRESHOLD_G,baselineResultantG=1.0f;
int moistureDry=MOISTURE_DRY,moistureWet=MOISTURE_WET;
uint8_t tapCount=0;
float liveX=0,liveY=0,liveZ=0,liveResultant=1,liveMoisturePercent=0;
bool liveVibration=false;
int liveMoistureRaw=0;
unsigned long lastSensorMs=0,lastTapMs=0;

// =====================================================
// AUTOMATIC RELAY CONTROL DURING 5-TAP TEST
// =====================================================
#define RELAY_INTERVAL_MS 1000
unsigned long lastRelayToggleMs=0;
bool autoRelayState=false;
uint16_t captureIndex=0; float cap[CAPTURE_SAMPLES]; float capX=0,capY=0,capZ=0,capResultant=0,capPeakDev=0; int capMoistRaw=0; float capMoist=0;

struct TapRecord {uint8_t tapNumber=0;float x=0,y=0,z=0,resultant=0,peak=0,moisturePercent=0,dominantFrequency=0,dominantAmplitude=0;int moistureRaw=0,tapScore=0;String vibrationLevel="LOW",tapCondition="POOR";};
TapRecord taps[TAPS_PER_TEST];

String jsEscape(const String&s){String o;for(size_t i=0;i<s.length();i++){char c=s[i];if(c=='\\\\'||c=='"'){o+='\\\\';o+=c;}else if(c=='\n')o+="\\\n";else o+=c;}return o;}
String nowDate(){struct tm t;if(!getLocalTime(&t,20))return "----/--/--";char b[16];strftime(b,sizeof(b),"%Y-%m-%d",&t);return String(b);}
String nowTime(){struct tm t;if(!getLocalTime(&t,20))return "--:--:--";char b[12];strftime(b,sizeof(b),"%H:%M:%S",&t);return String(b);}

void relayPin(uint8_t pin,bool on){digitalWrite(pin,(RELAY_ACTIVE_LOW?!on:on)?HIGH:LOW);}void allOff(){relay1State=false;relay2State=false;relayPin(PIN_RELAY_1,false);relayPin(PIN_RELAY_2,false);}

void automaticRelayControl(){
  if(!testRunning){
    autoRelayState=false;
    relay1State=false;
    relayPin(PIN_RELAY_1,false);
    return;
  }
  unsigned long now=millis();
  if(lastRelayToggleMs==0){
    autoRelayState=true;
    relay1State=true;
    relayPin(PIN_RELAY_1,true);
    lastRelayToggleMs=now;
    Serial.println("AUTO RELAY: ON");
    return;
  }
  if(now-lastRelayToggleMs>=RELAY_INTERVAL_MS){
    lastRelayToggleMs=now;
    autoRelayState=!autoRelayState;
    relay1State=autoRelayState;
    relayPin(PIN_RELAY_1,autoRelayState);
    Serial.println(autoRelayState?"AUTO RELAY: ON":"AUTO RELAY: OFF");
  }
}

float clamp100(float v){return v<0?0:(v>100?100:v);}float moisturePct(int raw){if(moistureDry==moistureWet)return 0;return clamp100((moistureDry-raw)*100.0f/(moistureDry-moistureWet));}

String vibLevel(float p){if(p<=VIBRATION_LOW_MAX_G)return "LOW";if(p<=VIBRATION_MEDIUM_MAX_G)return "MEDIUM";return "HIGH";}

int vibScore(float p){if(p<=0.30f)return 100;if(p>=1.20f)return 0;if(p<=0.70f)return (int)roundf(100-(p-0.30f)/0.40f*40);return (int)roundf(60-(p-0.70f)/0.50f*60);}

int moistScore(float m){if(m<=40)return 100;if(m<=60)return (int)roundf(100-(m-40)/20*30);if(m<=80)return (int)roundf(70-(m-60)/20*30);return (int)roundf(max(0.0f,40-(m-80)/20*40));}

int tapScore(float p,float m){return constrain((int)roundf(vibScore(p)*0.70f+moistScore(m)*0.30f),0,100);}String cond(int s){return s>=70?"GOOD":(s>=40?"FAIR":"POOR");}

int counter(){if(!LittleFS.exists(COUNTER_FILE))return 0;File f=LittleFS.open(COUNTER_FILE,"r");int n=f?f.readString().toInt():0;if(f)f.close();return n;}void setCounter(int n){File f=LittleFS.open(COUNTER_FILE,"w");if(f){f.print(n);f.close();}}

String nextId(){int n=counter()+1;setCounter(n);char b[20];snprintf(b,sizeof(b),"CA-%03d",n);return String(b);}

void storage(){if(!LittleFS.exists(TESTS_DIR))LittleFS.mkdir(TESTS_DIR);if(!LittleFS.exists(TESTS_INDEX_FILE)){File f=LittleFS.open(TESTS_INDEX_FILE,"w");if(f){f.print("[]");f.close();}}}

void loadSettings(){if(!LittleFS.exists(SETTINGS_FILE))return;File f=LittleFS.open(SETTINGS_FILE,"r");if(!f)return;JsonDocument d;deserializeJson(d,f);f.close();tapThresholdG=d["tapThresholdG"]|DEFAULT_TAP_THRESHOLD_G;moistureDry=d["moistureDry"]|MOISTURE_DRY;moistureWet=d["moistureWet"]|MOISTURE_WET;}

void saveSettings(){JsonDocument d;d["tapThresholdG"]=tapThresholdG;d["moistureDry"]=moistureDry;d["moistureWet"]=moistureWet;d["vibrationDetected"]=liveVibration;File f=LittleFS.open(SETTINGS_FILE,"w");if(f){serializeJson(d,f);f.close();}}

float lastDominantFreq=0,lastDominantAmp=0;

float lastWave[CAPTURE_SAMPLES]; float lastSpectrum[CAPTURE_SAMPLES/2+1]; uint16_t lastWaveN=0;

float median5(float a,float b,float c,float d,float e){float v[5]={a,b,c,d,e};for(int i=0;i<5;i++)for(int j=i+1;j<5;j++)if(v[j]<v[i]){float t=v[i];v[i]=v[j];v[j]=t;}return v[2];}

void analyseCapture(float &df,float &amp){
  float x[CAPTURE_SAMPLES]; float mean=0; for(int i=0;i<CAPTURE_SAMPLES;i++)mean+=cap[i]; mean/=CAPTURE_SAMPLES;
  for(int i=0;i<CAPTURE_SAMPLES;i++){float raw=cap[i]-mean;float v=raw;if(i>=2&&i<CAPTURE_SAMPLES-2)v=median5(cap[i-2]-mean,cap[i-1]-mean,cap[i]-mean,cap[i+1]-mean,cap[i+2]-mean);float w=0.54f-0.46f*cosf(2*PI*i/(CAPTURE_SAMPLES-1));x[i]=v*w;lastWave[i]=cap[i]-baselineResultantG;}
  float bestA=0;int bestK=0;for(int k=1;k<=CAPTURE_SAMPLES/2;k++){float re=0,im=0;for(int n=0;n<CAPTURE_SAMPLES;n++){float a=2*PI*k*n/CAPTURE_SAMPLES;re+=x[n]*cosf(a);im-=x[n]*sinf(a);}float mag=2.0f*sqrtf(re*re+im*im)/CAPTURE_SAMPLES;lastSpectrum[k]=mag;if(mag>bestA){bestA=mag;bestK=k;}}lastSpectrum[0]=0;
  df=(float)bestK*SAMPLE_RATE_HZ/CAPTURE_SAMPLES;amp=bestA;lastDominantFreq=df;lastDominantAmp=amp;lastWaveN=CAPTURE_SAMPLES;
}

void saveTest(){if(tapCount!=TAPS_PER_TEST)return;float avgP=0,maxP=0,minP=999,avgM=0;int avgS=0;float sumF=0,sumA=0;for(int i=0;i<tapCount;i++){avgP+=taps[i].peak;avgM+=taps[i].moisturePercent;avgS+=taps[i].tapScore;maxP=max(maxP,taps[i].peak);minP=min(minP,taps[i].peak);sumF+=taps[i].dominantFrequency;sumA+=taps[i].dominantAmplitude;}avgP/=5;avgM/=5;avgS=(int)roundf(avgS/5.0f);
  JsonDocument d;d["testId"]=currentTestId;d["building"]=currentBuilding;d["wall"]=currentWall;d["date"]=nowDate();d["time"]=nowTime();d["tapCount"]=5;d["avgPeak"]=avgP;d["maxPeak"]=maxP;d["minPeak"]=minP;d["avgMoisture"]=avgM;d["score"]=avgS;d["condition"]=cond(avgS);d["sampleRate"]=SAMPLE_RATE_HZ;d["sampleCount"]=CAPTURE_SAMPLES;d["captureWindowMs"]=CAPTURE_WINDOW_MS;d["frequencyResolution"]=(float)SAMPLE_RATE_HZ/CAPTURE_SAMPLES;d["dominantFrequency"]=sumF/5.0f;d["dominantAmplitude"]=sumA/5.0f;JsonArray a=d.createNestedArray("taps");for(int i=0;i<5;i++){JsonObject o=a.createNestedObject();o["tapNumber"]=taps[i].tapNumber;o["x"]=taps[i].x;o["y"]=taps[i].y;o["z"]=taps[i].z;o["resultant"]=taps[i].resultant;o["peak"]=taps[i].peak;o["moistureRaw"]=taps[i].moistureRaw;o["moisturePercent"]=taps[i].moisturePercent;o["vibrationLevel"]=taps[i].vibrationLevel;o["tapScore"]=taps[i].tapScore;o["tapCondition"]=taps[i].tapCondition;o["dominantFrequency"]=taps[i].dominantFrequency;o["dominantAmplitude"]=taps[i].dominantAmplitude;}JsonArray wave=d.createNestedArray("waveform");for(int i=0;i<lastWaveN;i++){JsonArray p=wave.createNestedArray();p.add(i/(float)SAMPLE_RATE_HZ);p.add(lastWave[i]-baselineResultantG);}JsonArray sp=d.createNestedArray("spectrum");for(int k=1;k<=CAPTURE_SAMPLES/2;k++){JsonArray p=sp.createNestedArray();p.add(k*(float)SAMPLE_RATE_HZ/CAPTURE_SAMPLES);p.add(lastSpectrum[k]);}
  String path=String(TESTS_DIR)+"/"+currentTestId+".json";File f=LittleFS.open(path,"w");if(f){serializeJson(d,f);f.close();}
  JsonDocument idxDoc;File fi=LittleFS.open(TESTS_INDEX_FILE,"r");if(fi){deserializeJson(idxDoc,fi);fi.close();}if(!idxDoc.is<JsonArray>())idxDoc.to<JsonArray>();JsonArray idx=idxDoc.as<JsonArray>();JsonObject s=idx.createNestedObject();s["testId"]=currentTestId;s["building"]=currentBuilding;s["wall"]=currentWall;s["date"]=nowDate();s["time"]=nowTime();s["tapCount"]=5;s["avgVibration"]=avgP;s["avgMoisture"]=avgM;s["score"]=avgS;s["condition"]=cond(avgS);while(idx.size()>MAX_HISTORY_TESTS){String old=idx[0]["testId"]|"";if(old.length())LittleFS.remove(String(TESTS_DIR)+"/"+old+".json");idx.remove(0);}File fo=LittleFS.open(TESTS_INDEX_FILE,"w");if(fo){serializeJson(idxDoc,fo);fo.close();}
}

void finishTap(){captureActive=false;if(!testRunning||tapCount>=5)return;TapRecord&r=taps[tapCount];r.tapNumber=tapCount+1;r.x=capX;r.y=capY;r.z=capZ;r.resultant=capResultant;r.peak=capPeakDev;r.moistureRaw=capMoistRaw;r.moisturePercent=capMoist;r.vibrationLevel=vibLevel(r.peak);analyseCapture(r.dominantFrequency,r.dominantAmplitude);r.tapScore=tapScore(r.peak,r.moisturePercent);r.tapCondition=cond(r.tapScore);tapCount++;lastTapMs=millis();lastStatusText="TAP "+String(tapCount)+" / 5 - DATA STORED";if(tapCount>=5){testRunning=false;testComplete=true;lastStatusText="TEST COMPLETE";autoRelayState=false;lastRelayToggleMs=0;relay1State=false;relayPin(PIN_RELAY_1,false);saveTest();digitalWrite(PIN_STATUS_LED,LOW);}}

void sensor() {
 liveMoistureRaw = analogRead(PIN_SOIL_MOISTURE);
  liveMoisturePercent = moisturePct(liveMoistureRaw);
  int vibRaw = digitalRead(PIN_VIBRATION);
  liveVibration =
    VIBRATION_ACTIVE_HIGH ?
    (vibRaw == HIGH) :
    (vibRaw == LOW);
  if (!mpuReady) {
    Serial.println("MPU6050 NOT READY");
    return;
  }

  float axMs2, ayMs2, azMs2, gxRad, gyRad, gzRad, tempC;
  if (!mpu6500Read(axMs2, ayMs2, azMs2, gxRad, gyRad, gzRad, tempC)) {
    Serial.println("MPU6500 read error");
    return;
  }

  liveX = axMs2 / 9.80665f;
  liveY = ayMs2 / 9.80665f;
  liveZ = azMs2 / 9.80665f;

  liveResultant = sqrtf(
    liveX * liveX +
    liveY * liveY +
    liveZ * liveZ
  );

  float roll =
    atan2f(liveY, liveZ) * 180.0f / PI;

  float pitch =
    atan2f(
      -liveX,
      sqrtf(liveY * liveY + liveZ * liveZ)
    ) * 180.0f / PI;

  float gyroX = gxRad * 180.0f / PI;
  float gyroY = gyRad * 180.0f / PI;
  float gyroZ = gzRad * 180.0f / PI;

  float temperature = tempC;

  Serial.println();
  Serial.println("======================================");
  Serial.print("ACC X       : ");
  Serial.print(liveX, 3);
  Serial.println(" g");
  Serial.print("ACC Y       : ");
  Serial.print(liveY, 3);
  Serial.println(" g");
  Serial.print("ACC Z       : ");
  Serial.print(liveZ, 3);
  Serial.println(" g");
  Serial.print("RESULTANT   : ");
  Serial.print(liveResultant, 3);
  Serial.println(" g");
  Serial.print("ROLL        : ");
  Serial.print(roll, 2);
  Serial.println(" deg");
  Serial.print("PITCH       : ");
  Serial.print(pitch, 2);
  Serial.println(" deg");
  Serial.print("GYRO X      : ");
  Serial.print(gyroX, 2);
  Serial.println(" deg/s");
  Serial.print("GYRO Y      : ");
  Serial.print(gyroY, 2);
  Serial.println(" deg/s");
  Serial.print("GYRO Z      : ");
  Serial.print(gyroZ, 2);
  Serial.println(" deg/s");
  Serial.print("TEMPERATURE : ");
  Serial.print(temperature, 2);
  Serial.println(" C");
  Serial.print("BASELINE    : ");
  Serial.print(baselineResultantG, 3);
  Serial.println(" g");
  Serial.print("DEVIATION   : ");
  Serial.print(
    fabsf(liveResultant - baselineResultantG),
    3
  );
  Serial.println(" g");
  Serial.print("MOISTURE    : ");
  Serial.print(liveMoisturePercent, 1);
  Serial.println(" %");
  Serial.print("VIBRATION   : ");
  Serial.println(liveVibration ? "DETECTED" : "NO");
  Serial.println("======================================");

  if (!testRunning || tapCount >= 5)
    return;

  unsigned long now = millis();

  float dev =
    fabsf(liveResultant - baselineResultantG);

  if (captureActive) {
    cap[captureIndex++] = liveResultant;
    if (dev > capPeakDev) {
      capPeakDev = dev;
      capX = liveX;
      capY = liveY;
      capZ = liveZ;
      capResultant = liveResultant;
      capMoistRaw = liveMoistureRaw;
      capMoist = liveMoisturePercent;
    }
    if (captureIndex >= CAPTURE_SAMPLES) {
      finishTap();
    }
    return;
  }

  if (now - lastTapMs < TAP_REFRACTORY_MS)
    return;

  if (dev >= tapThresholdG) {
    captureActive = true;
    captureIndex = 0;
    capPeakDev = dev;
    capX = liveX;
    capY = liveY;
    capZ = liveZ;
    capResultant = liveResultant;
    capMoistRaw = liveMoistureRaw;
    capMoist = liveMoisturePercent;
    cap[captureIndex++] = liveResultant;
    lastStatusText =
      "TAP DETECTED - CAPTURING";
    Serial.println();
    Serial.println("******** TAP DETECTED ********");
    Serial.print("Peak deviation: ");
    Serial.print(dev, 3);
    Serial.println(" g");
    Serial.println("*******************************");
  }
}

// Because the line above must stay simple for Arduino C++ parsing, capture peak is taken from the captured buffer in finishTap().

void calibrate(){
  if(!mpuReady)return;
  float sum=0;
  for(int i=0;i<BASELINE_SAMPLES;i++){
    float axMs2, ayMs2, azMs2, gxRad, gyRad, gzRad, tempC;
    if(!mpu6500Read(axMs2, ayMs2, azMs2, gxRad, gyRad, gzRad, tempC)) continue;
    float x=axMs2/9.80665f, y=ayMs2/9.80665f, z=azMs2/9.80665f;
    sum+=sqrtf(x*x+y*y+z*z);
    delay(5);
  }
  baselineResultantG=sum/BASELINE_SAMPLES;
  if(baselineResultantG<0.8||baselineResultantG>1.2)baselineResultantG=1.0f;
}

String makeLive(){JsonDocument d;bool sta=WiFi.status()==WL_CONNECTED;d["ip"]=WiFi.localIP().toString();d["ssid"]=WiFi.SSID();d["mode"]="STA";d["type"]="live";d["x"]=liveX;d["y"]=liveY;d["z"]=liveZ;d["resultant"]=liveResultant;d["moisturePercent"]=liveMoisturePercent;d["vibrationDetected"]=liveVibration;d["tapNumber"]=(testRunning?tapCount:tapCount);d["tapsPerTest"]=5;d["state"]=testRunning?"WAITING_FOR_TAP":(testComplete?"TEST_COMPLETE":"IDLE");d["testId"]=currentTestId;d["building"]=currentBuilding;d["wall"]=currentWall;d["captureActive"]=captureActive;d["lastTap"]=tapCount;d["lastTapPeak"]=tapCount?taps[tapCount-1].peak:0;d["mpuReady"]=mpuReady;String o;serializeJson(d,o);return o;}

static const char CRACKAURA_HTML[] PROGMEM = R"rawliteral(

<!doctype html>

<html lang="en"><head>

<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">

<title>CrackAura | Wall Health Monitoring</title>

<style>

:root{--bg:#071321;--panel:#0f2137;--panel2:#122a44;--line:#27415f;--text:#edf5ff;--muted:#91a7c1;--blue:#2d8cff;--cyan:#25d6e8;--green:#31c779;--amber:#f0b53a;--red:#ff5c68;--white:#fff}*{box-sizing:border-box}body{margin:0;background:linear-gradient(135deg,#06101c,#0a1c30);color:var(--text);font-family:Segoe UI,Arial,sans-serif}button,input,select{font:inherit}.app{display:flex;min-height:100vh}.side{width:220px;background:#071426;border-right:1px solid var(--line);padding:18px;position:sticky;top:0;height:100vh}.brand{font-size:23px;font-weight:800;color:var(--cyan);margin:4px 8px 22px}.brand span{color:#fff}.nav{display:grid;gap:8px}.nav button{border:1px solid transparent;background:transparent;color:#b8c8dd;text-align:left;padding:12px;border-radius:10px;cursor:pointer}.nav button.active,.nav button:hover{background:#11365b;color:#fff;border-color:#235a90}.foot{position:absolute;bottom:18px;left:18px;right:18px;color:var(--muted);font-size:12px}.dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:#687b92;margin-right:7px}.dot.on{background:var(--green);box-shadow:0 0 9px var(--green)}.main{flex:1;padding:22px;max-width:1500px;margin:auto;width:100%}.top{display:flex;justify-content:space-between;gap:12px;align-items:center;margin-bottom:18px}.top h1{margin:0;font-size:25px}.badges{display:flex;gap:10px;align-items:center;flex-wrap:wrap}.badge{padding:8px 11px;border-radius:999px;background:#102843;border:1px solid var(--line);color:#dcecff;font-size:12px}.btn{border:0;border-radius:9px;padding:10px 15px;cursor:pointer;font-weight:700}.primary{background:var(--blue);color:#fff}.danger{background:var(--red);color:#fff}.ghost{background:#183451;color:#fff;border:1px solid var(--line)}.page{display:none}.page.active{display:block}.grid{display:grid;gap:14px}.g3{grid-template-columns:repeat(3,minmax(0,1fr))}.g4{grid-template-columns:repeat(4,minmax(0,1fr))}.g2{grid-template-columns:repeat(2,minmax(0,1fr))}.card,.panel{background:linear-gradient(180deg,#10243a,#0d1f33);border:1px solid var(--line);border-radius:14px;padding:17px;box-shadow:0 12px 30px rgba(0,0,0,.16)}.label{color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.08em}.value{font-size:28px;font-weight:800;margin-top:6px}.sub{color:var(--muted);font-size:12px;margin-top:5px}.title{font-size:16px;font-weight:800;margin-bottom:14px}.steps{display:grid;grid-template-columns:repeat(5,1fr);gap:8px}.step{padding:12px;border:1px solid var(--line);border-radius:10px;text-align:center;background:#0a1828}.step b{display:block;color:var(--cyan);font-size:12px}.hero{background:linear-gradient(135deg,#0e2e4d,#10233a);border:1px solid #2a567f;border-radius:16px;padding:19px;margin-bottom:15px}.hero h2{margin:0 0 7px;font-size:21px}.hero p{margin:0;color:#b8cbe2}.field{display:grid;gap:6px;margin-bottom:12px}.field label{color:#a9bed7;font-size:12px}.field input,.field select{background:#081828;border:1px solid #294664;color:#fff;border-radius:8px;padding:10px}.wallmap{display:grid;grid-template-columns:1fr 1fr;gap:10px}.wall{min-height:105px;border-radius:12px;background:#0b1b2c;border:1px solid #2a4059;color:#d9e9fb;cursor:pointer}.wall.selected{background:#0e5fb7;border-color:#3d9aff;box-shadow:0 0 0 2px rgba(45,140,255,.22)}.statusline{display:flex;justify-content:space-between;gap:10px;align-items:center;padding:11px 0;border-bottom:1px solid #20364e}.statusline:last-child{border-bottom:0}.progress{height:10px;border-radius:999px;background:#091725;overflow:hidden;border:1px solid #1d344c}.bar{height:100%;width:0;background:linear-gradient(90deg,var(--blue),var(--cyan));transition:width .25s}.taprow{display:grid;grid-template-columns:repeat(5,1fr);gap:8px}.tap{padding:10px 4px;border-radius:9px;text-align:center;background:#091827;border:1px solid #1d344b;font-size:12px}.tap.done{background:rgba(49,199,121,.16);border-color:#2d8a60;color:#83efb5}.tap.current{background:rgba(45,140,255,.18);border-color:#2d8cff;color:#91c7ff}.chart{width:100%;height:190px;background:#081522;border-radius:10px;border:1px solid #1d334a}.score{display:grid;grid-template-columns:170px 1fr;gap:16px;align-items:center}.circle{width:150px;height:150px;border-radius:50%;display:grid;place-items:center;background:radial-gradient(circle at center,#0d2135 57%,transparent 58%),conic-gradient(var(--green) 0deg,#27415f 0deg);font-size:32px;font-weight:900}.scoregood{color:var(--green)}.scorefair{color:var(--amber)}.scorepoor{color:var(--red)}table{width:100%;border-collapse:collapse;font-size:12px}th,td{padding:9px;border-bottom:1px solid #20364e;text-align:left}th{color:#a8bed7;background:#0a1828;position:sticky;top:0}.tablewrap{overflow:auto;max-height:360px}.pill{display:inline-block;padding:4px 7px;border-radius:999px;background:#14304d}.good{color:#7ee8af}.fair{color:#f7d67b}.poor{color:#ff9099}.pipeline{display:grid;grid-template-columns:repeat(5,1fr);gap:8px}.pipe{padding:12px;text-align:center;border:1px solid #29435f;border-radius:10px;background:#0b1a2b}.pipe b{display:block;color:var(--cyan);font-size:12px;margin-bottom:5px}.mini-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}.mini{padding:12px;border:1px solid #243e59;border-radius:10px;background:#0a1828}.alert{padding:11px;border-left:4px solid var(--amber);background:#1d2130;border-radius:7px;color:#dbe6f4}.alert.ok{border-left-color:var(--green)}.muted{color:var(--muted)}.actions{display:flex;gap:8px;flex-wrap:wrap}.small{font-size:11px}.hide{display:none}@media(max-width:1000px){.side{width:190px}.g4,.g3,.g2{grid-template-columns:1fr 1fr}.pipeline{grid-template-columns:1fr 1fr}.score{grid-template-columns:1fr}}@media(max-width:700px){.app{display:block}.side{width:auto;height:auto;position:relative}.nav{grid-template-columns:repeat(3,1fr)}.foot{position:relative;bottom:auto;left:auto;right:auto;margin-top:12px}.g4,.g3,.g2,.mini-grid{grid-template-columns:1fr}.steps{grid-template-columns:1fr}.taprow{grid-template-columns:repeat(5,1fr)}.main{padding:12px}}

</style></head><body><div class="app">

<aside class="side"><div class="brand">Crack<span>Aura</span></div><div class="nav">

<button class="active" data-page="home">⌂ Home</button><button data-page="setup">⚙ Test Setup</button><button data-page="run">▶ Run Test</button><button data-page="history">☷ History</button><button data-page="analysis">⌁ Analysis</button><button data-page="reports">▤ Reports</button><button data-page="settings">⚒ Settings</button></div><div class="foot"><span id="connDot" class="dot"></span><span id="connText">Connecting...</span><br><span id="modeText">ESP32</span></div></aside>

<main class="main"><div class="top"><h1 id="pageTitle">System Overview</h1><div class="badges"><span id="ipBadge" class="badge">IP: --</span><button class="btn danger" id="stopBtn">STOP / ALL OFF</button></div></div>

<section class="page active" id="page-home"><div class="hero"><h2>CrackAura — Intelligent Wall Health Monitoring</h2><p>ESP32-based impact, vibration and moisture monitoring for Wall 1–4. Five controlled taps are captured and converted into a health score.</p></div><div class="grid g4"><div class="card"><div class="label">System Status</div><div class="value" id="homeState">IDLE</div><div class="sub">Test controller state</div></div><div class="card"><div class="label">ESP32</div><div class="value" id="homeConn">OFFLINE</div><div class="sub" id="homeMode">Mode: --</div></div><div class="card"><div class="label">Current Wall</div><div class="value" id="homeWall">Wall 1</div><div class="sub" id="homeTest">No active test</div></div><div class="card"><div class="label">Last Score</div><div class="value" id="homeScore">--</div><div class="sub" id="homeCondition">No completed tests</div></div><div class="card"><div class="label">Quality Grade</div><div class="value" id="homeGrade">WAITING</div><div class="sub">GOOD / FAIR / POOR</div></div></div><div class="panel" style="margin-top:14px"><div class="title">CrackAura Signal Flow</div><div class="steps"><div class="step"><b>1</b>Wall selected</div><div class="step"><b>2</b>5 impacts</div><div class="step"><b>3</b>Sensor capture</div><div class="step"><b>4</b>Signal analysis</div><div class="step"><b>5</b>Health grade</div></div></div><div class="grid g4" style="margin-top:14px"><div class="card"><div class="label">MPU6050</div><div class="value" id="homeMpu">--</div><div class="sub">3-axis acceleration</div></div><div class="card"><div class="label">Soil Moisture</div><div class="value" id="homeMoist">--</div><div class="sub">Analog moisture estimate</div></div><div class="card"><div class="label">Vibration Sensor</div><div class="value" id="homeVibration">NO</div><div class="sub">GPIO 35 digital input</div></div><div class="card"><div class="label">Tap Progress</div><div class="value" id="homeTap">0 / 5</div><div class="sub">Captured wall impacts</div></div></div></section>

<section class="page" id="page-setup"><div class="grid g2"><div class="panel"><div class="title">Select Wall & Configure Test</div><div class="field"><label>Building</label><select id="building"><option>Building A</option><option>Building B</option><option>Building C</option><option>Building D</option></select></div><div class="field"><label>Selected Wall</label><input id="wall" value="Wall 1" readonly></div><div class="field"><label>Tap Threshold (g)</label><input id="threshold" type="number" step="0.01" value="0.35"></div><div class="alert ok">Exactly <b>5 valid taps</b> are collected for each test.</div><div class="actions" style="margin-top:14px"><button class="btn primary" id="startBtn">START TEST</button></div></div><div class="panel"><div class="title" id="mapTitle">Building A — Wall Map</div><div class="wallmap"><button class="wall selected" data-wall="Wall 1">WALL 1</button><button class="wall" data-wall="Wall 2">WALL 2</button><button class="wall" data-wall="Wall 3">WALL 3</button><button class="wall" data-wall="Wall 4">WALL 4</button></div><div class="sub" style="margin-top:12px">Selected: <b id="selectedWall">Wall 1</b></div></div></div></section>

<section class="page" id="page-run"><div class="grid g3"><div class="panel"><div class="label">TEST ID</div><div class="value" id="runId">--</div><div class="sub" id="runTarget">Building A / Wall 1</div></div><div class="panel"><div class="label">TEST STATUS</div><div class="value" id="runState">IDLE</div><div class="sub" id="runInstruction">Start a test from Test Setup</div></div><div class="panel"><div class="label">5-TAP PROGRESS</div><div class="value" id="runTap">0 / 5</div><div class="progress"><div id="runBar" class="bar"></div></div></div></div><div class="panel" style="margin-top:14px"><div class="taprow" id="tapRow"><div class="tap current">TAP 1</div><div class="tap">TAP 2</div><div class="tap">TAP 3</div><div class="tap">TAP 4</div><div class="tap">TAP 5</div></div></div><div class="grid g4" style="margin-top:14px"><div class="card"><div class="label">X ACCELERATION</div><div class="value" id="x">0.00 g</div></div><div class="card"><div class="label">Y ACCELERATION</div><div class="value" id="y">0.00 g</div></div><div class="card"><div class="label">Z ACCELERATION</div><div class="value" id="z">0.00 g</div></div><div class="card"><div class="label">RESULTANT / PEAK</div><div class="value" id="peak">0.00 g</div></div></div><div class="grid g2" style="margin-top:14px"><div class="panel"><div class="title">Live Time Domain Signal</div><canvas id="liveCanvas" class="chart" height="190"></canvas></div><div class="panel"><div class="title">Live Sensor / Acquisition</div><div class="mini-grid"><div class="mini"><div class="label">Moisture</div><div class="value" id="moisture">0%</div></div><div class="mini"><div class="label">Vibration Level</div><div class="value" id="vibration">LOW</div></div><div class="mini"><div class="label">Vibration Sensor</div><div class="value" id="vibrationSensor">NO</div></div></div><div class="mini-grid" style="margin-top:10px"><div class="mini"><div class="label">Acquisition</div><div class="value" id="acq">READY</div></div></div><div id="alertBox" class="alert" style="margin-top:12px">Waiting for a physical tap on the selected wall.</div></div></div><div class="panel" id="runResults" style="margin-top:14px"><div class="title">Results & Grading</div><div class="score"><div id="scoreCircle" class="circle">--</div><div><div class="value" id="resultCondition">--</div><div class="sub" id="resultSummary">Complete 5 taps to calculate wall quality.</div><div class="actions" style="margin:10px 0"><span class="pill good">GOOD ≥ 70</span><span class="pill fair">FAIR 40–69</span><span class="pill poor">POOR &lt; 40</span></div><div class="mini-grid" style="margin-top:10px"><div class="mini"><div class="label">Dominant Frequency</div><div class="value" id="domFreq">--</div></div><div class="mini"><div class="label">Peak Amplitude</div><div class="value" id="domAmp">--</div></div><div class="mini"><div class="label">Average Moisture</div><div class="value" id="avgMoist">--</div></div></div></div></div><div id="runAnalysis" style="margin-top:14px"></div><div class="actions" style="margin-top:14px"><button class="btn primary" id="csvBtn">DOWNLOAD CSV</button><button class="btn ghost" id="printBtn">PRINT / SAVE PDF</button></div></div></section>

<section class="page" id="page-analysis"><div class="panel"><div class="title">Signal Processing Pipeline</div><div class="pipeline"><div class="pipe"><b>RAW SIGNAL</b>Captured acceleration samples</div><div class="pipe"><b>DC OFFSET</b>Baseline removal</div><div class="pipe"><b>MEDIAN FILTER</b>Noise reduction</div><div class="pipe"><b>HAMMING WINDOW</b>Spectral window</div><div class="pipe"><b>FFT / DFT</b>Dominant frequency</div></div></div><div class="grid g2" style="margin-top:14px"><div class="panel"><div class="title">Time Domain Signal</div><canvas id="timeCanvas" class="chart" height="220"></canvas><div class="sub" id="timeMeta">No completed test</div></div><div class="panel"><div class="title">Frequency Domain (FFT-style spectrum)</div><canvas id="freqCanvas" class="chart" height="220"></canvas><div class="sub" id="freqMeta">No completed test</div></div></div><div class="panel" style="margin-top:14px"><div class="title">Analysis Summary</div><div id="analysisSummary" class="grid g4"></div></div></section>

<section class="page" id="page-history"><div class="panel"><div class="title">Stored Test Records</div><div class="tablewrap"><table><thead><tr><th>ID</th><th>Building</th><th>Wall</th><th>Date</th><th>Time</th><th>Taps</th><th>Avg Peak</th><th>Moisture</th><th>Score</th><th>Grade</th></tr></thead><tbody id="historyBody"></tbody></table></div></div><div class="panel" id="historyDetail" style="display:none;margin-top:14px"></div></section>

<section class="page" id="page-reports"><div class="panel"><div class="title">Report Generation</div><div class="field"><label>Select Test</label><select id="reportSelect"></select></div><div id="reportView"></div></div></section>

<section class="page" id="page-settings"><div class="grid g2"><div class="panel"><div class="title">Relay / Solenoid Control</div><div class="statusline"><span>Relay 1</span><button class="btn ghost relay" data-relay="1">OFF</button></div><div class="statusline"><span>Relay 2 / Solenoid</span><button class="btn ghost relay" data-relay="2">OFF</button></div><div class="actions" style="margin-top:14px"><button class="btn danger" id="alloff2">STOP / ALL OFF</button></div></div><div class="panel"><div class="title">Calibration</div><div class="field"><label>Tap Threshold (g)</label><input id="setThreshold" type="number" step="0.01"></div><div class="field"><label>Moisture Dry ADC</label><input id="setDry" type="number"></div><div class="field"><label>Moisture Wet ADC</label><input id="setWet" type="number"></div><button class="btn primary" id="saveSettings">SAVE SETTINGS</button></div></div><div class="panel" style="margin-top:14px"><div class="title">Device Information</div><div id="deviceInfo"></div></div></section>

</main></div>

<script>

const $=id=>document.getElementById(id);let currentId=null,lastTap=0,lastState='',timer=null,live=[],currentTest=null;

const pages={home:'System Overview',setup:'Select Wall & Configure Test',run:'Impact (Firing) & Data Acquisition',history:'Stored Test Records',analysis:'Signal Processing & FFT Analysis',reports:'Report Generation',settings:'Settings'};

function page(p){document.querySelectorAll('.page').forEach(x=>x.classList.remove('active'));$('page-'+p).classList.add('active');document.querySelectorAll('.nav button').forEach(b=>b.classList.toggle('active',b.dataset.page===p));$('pageTitle').textContent=pages[p];if(p==='history')loadHistory();if(p==='reports')loadReports();if(p==='settings')loadSettings();}

document.querySelectorAll('.nav button').forEach(b=>b.onclick=()=>page(b.dataset.page));

function setConn(ok,s){$('connDot').classList.toggle('on',ok);$('connText').textContent=ok?'Connected':'Reconnecting...';$('homeConn').textContent=ok?'ONLINE':'OFFLINE';if(s){$('ipBadge').textContent='IP: '+s.ip;$('homeMode').textContent='Mode: '+s.mode;$('modeText').textContent=s.mode+' | '+s.ssid}}

function wallSelect(w){document.querySelectorAll('.wall').forEach(b=>b.classList.toggle('selected',b.dataset.wall===w));$('wall').value=w;$('selectedWall').textContent=w;$('homeWall').textContent=w}document.querySelectorAll('.wall').forEach(b=>b.onclick=()=>wallSelect(b.dataset.wall));$('building').onchange=()=>{$('mapTitle').textContent=$('building').value+' — Wall Map'};

function draw(canvas,data,mode='time'){if(!canvas)return;const dpr=devicePixelRatio||1,w=canvas.clientWidth||600,h=+canvas.getAttribute('height')||200;canvas.width=w*dpr;canvas.height=h*dpr;const c=canvas.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);c.strokeStyle='#1d334a';c.lineWidth=1;for(let i=1;i<5;i++){const yy=i*h/5;c.beginPath();c.moveTo(0,yy);c.lineTo(w,yy);c.stroke()}if(!data||data.length<2)return;let vals=data.map(p=>Array.isArray(p)?p[1]:p.value);let max=Math.max(.05,...vals.map(v=>Math.abs(v)));let min=Math.min(0,...vals);c.strokeStyle='#2d8cff';c.lineWidth=2;c.beginPath();vals.forEach((v,i)=>{const xx=i/(vals.length-1)*w,yy=h/2-(v/max)*(h*.40);if(i===0)c.moveTo(xx,yy);else c.lineTo(xx,yy)});c.stroke();}

function updateLive(m){$('x').textContent=(m.x||0).toFixed(2)+' g';$('y').textContent=(m.y||0).toFixed(2)+' g';$('z').textContent=(m.z||0).toFixed(2)+' g';$('peak').textContent=(m.resultant||0).toFixed(2)+' g';$('moisture').textContent=(m.moisturePercent||0).toFixed(1)+' %';$('homeMoist').textContent=(m.moisturePercent||0).toFixed(1)+' %';$('homeMpu').textContent=m.mpuReady?'READY':'CHECK';$('vibrationSensor').textContent=m.vibrationDetected?'DETECTED':'NO';$('homeVibration').textContent=m.vibrationDetected?'DETECTED':'NO';let lv=m.mpuReady?((m.resultant||0)>0.7?'HIGH':(m.resultant||0)>0.3?'MEDIUM':'LOW'):'WAITING';$('vibration').textContent=lv;live.push((m.resultant||0)-1);if(live.length>140)live.shift();draw($('liveCanvas'),live.map(v=>[0,v]));if(m.state){$('homeState').textContent=m.state.replaceAll('_',' ');$('runState').textContent=m.state.replaceAll('_',' ')}let tc=m.tapNumber||0;$('runTap').textContent=tc+' / 5';$('homeTap').textContent=tc+' / 5';$('runBar').style.width=(tc/5*100)+'%';renderTaps(tc,m.state==='TEST_COMPLETE');if(m.captureActive)$('acq').textContent='CAPTURING';else if(m.state==='WAITING_FOR_TAP')$('acq').textContent='WAITING';else $('acq').textContent='READY';if(m.testId){currentId=m.testId;$('runId').textContent=m.testId;$('homeTest').textContent=m.testId;$('runTarget').textContent=(m.building||'')+' / '+(m.wall||'')}}

function renderTaps(n,complete){document.querySelectorAll('.tap').forEach((x,i)=>{x.classList.remove('done','current');if(i<n)x.classList.add('done');else if(i===n&&!complete)x.classList.add('current')})}

function showAlert(txt,ok=false){$('alertBox').textContent=txt;$('alertBox').classList.toggle('ok',ok)}

async function poll(){try{const r=await fetch('/api/live?x='+Date.now());const m=await r.json();setConn(true,m);updateLive(m);if(m.testId&&m.testId!==currentId)currentId=m.testId;if(m.lastTap&&m.lastTap>lastTap){lastTap=m.lastTap;showAlert('Tap '+lastTap+' captured and stored. Peak '+(m.lastTapPeak||0).toFixed(2)+' g',true)}if(m.state==='TEST_COMPLETE'&&m.testId&&m.testId!==lastState){lastState=m.testId;const r2=await fetch('/api/test?id='+encodeURIComponent(m.testId));if(r2.ok){currentTest=await r2.json();renderResult(currentTest);}}}catch(e){setConn(false)}}

async function start(){try{const b={building:$('building').value,wall:$('wall').value,tapThresholdG:parseFloat($('threshold').value)||0.35};const r=await fetch('/api/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});const d=await r.json();if(!r.ok||!d.ok)throw Error(d.error||'Start failed');currentId=d.testId;$('runId').textContent=d.testId;$('runTarget').textContent=b.building+' / '+b.wall;$('runResults').style.display='block';$('scoreCircle').textContent='--';$('resultCondition').textContent='WAITING';$('resultCondition').className='value';$('resultSummary').textContent='Complete 5 taps to calculate wall quality.';$('homeGrade').textContent='WAITING';$('runAnalysis').innerHTML='';live=[];lastTap=0;lastState='';showAlert('Test armed. Tap Wall physically 5 times, one at a time.');page('run')}catch(e){alert(e.message)}}$('startBtn').onclick=start;$('stopBtn').onclick=stop;$('alloff2').onclick=stop;

async function stop(){try{await fetch('/api/stop',{method:'POST'})}catch(e){}showAlert('Test stopped. Outputs switched OFF.',false)}

function renderResult(t){currentTest=t;const s=t.score||0,c=t.condition||'--';$('runResults').style.display='block';$('scoreCircle').textContent=s;$('scoreCircle').className='circle '+(c==='GOOD'?'scoregood':c==='FAIR'?'scorefair':'scorepoor');$('resultCondition').textContent=c;$('resultCondition').className='value '+(c==='GOOD'?'scoregood':c==='FAIR'?'scorefair':'scorepoor');$('resultSummary').textContent=`${t.building} / ${t.wall} • ${t.tapCount} taps • Avg peak ${(+t.avgPeak).toFixed(2)} g`;$('homeScore').textContent=s+'/100';$('homeCondition').textContent=c;$('homeGrade').textContent=c;$('homeGrade').className='value '+(c==='GOOD'?'scoregood':c==='FAIR'?'scorefair':'scorepoor');$('domFreq').textContent=((t.dominantFrequency||0)).toFixed(2)+' Hz';$('domAmp').textContent=((t.dominantAmplitude||0)).toFixed(2)+' g';$('avgMoist').textContent=((t.avgMoisture||0)).toFixed(1)+' %';$('runAnalysis').innerHTML='<div class="panel"><div class="title">Tap-by-Tap Results</div><div class="tablewrap"><table><thead><tr><th>Tap</th><th>X</th><th>Y</th><th>Z</th><th>Resultant</th><th>Peak</th><th>Frequency</th><th>Moisture</th><th>Score</th><th>Grade</th></tr></thead><tbody>'+t.taps.map(x=>`<tr><td>${x.tapNumber}</td><td>${(+x.x).toFixed(2)}</td><td>${(+x.y).toFixed(2)}</td><td>${(+x.z).toFixed(2)}</td><td>${(+x.resultant).toFixed(2)}</td><td>${(+x.peak).toFixed(2)}</td><td>${(+x.dominantFrequency||0).toFixed(2)} Hz</td><td>${(+x.moisturePercent).toFixed(1)}%</td><td>${x.tapScore}</td><td class="${String(x.tapCondition).toLowerCase()}">${x.tapCondition}</td></tr>`).join('')+'</tbody></table></div></div>';renderAnalysis(t);$('homeState').textContent='TEST COMPLETE'}

function renderAnalysis(t){let td=[];if(t.waveform&&t.waveform.length)td=t.waveform;else td=t.taps.map((x,i)=>[i,x.peak]);draw($('timeCanvas'),td);let spec=(t.spectrum||[]).map(x=>[x[0],x[1]]);draw($('freqCanvas'),spec);$('timeMeta').textContent=`Sampling: ${t.sampleRate||0} Hz • Samples/tap: ${t.sampleCount||0} • Window: ${t.captureWindowMs||0} ms`;$('freqMeta').textContent=`Resolution: ${(t.frequencyResolution||0).toFixed(2)} Hz • Dominant: ${(t.dominantFrequency||0).toFixed(2)} Hz`;$('analysisSummary').innerHTML=`<div class="mini"><div class="label">Raw Samples</div><div class="value">${t.sampleCount||0}</div></div><div class="mini"><div class="label">Baseline</div><div class="value">DC Removed</div></div><div class="mini"><div class="label">Filter</div><div class="value">5-Point Median</div></div><div class="mini"><div class="label">Window</div><div class="value">Hamming</div></div>`}


async function loadHistory(){try{const r=await fetch('/api/history?x='+Date.now());const a=await r.json();$('historyBody').innerHTML=a.map(t=>'<tr><td>'+t.testId+'</td><td>'+t.building+'</td><td>'+t.wall+'</td><td>'+t.date+'</td><td>'+t.time+'</td><td>'+t.tapCount+'</td><td>'+Number(t.avgVibration||0).toFixed(2)+' g</td><td>'+Number(t.avgMoisture||0).toFixed(1)+'%</td><td>'+t.score+'</td><td class="'+String(t.condition||'').toLowerCase()+'">'+t.condition+'</td></tr>').join('')||'<tr><td colspan=10>No tests stored</td></tr>';}catch(e){$('historyBody').innerHTML='<tr><td colspan=10>Unable to load history</td></tr>';}}
async function loadReports(){try{const r=await fetch('/api/history?x='+Date.now());const a=await r.json();$('reportSelect').innerHTML='<option value="">Select a test</option>'+a.map(t=>'<option value="'+t.testId+'">'+t.testId+' — '+t.building+' / '+t.wall+' — '+t.date+'</option>').join('');$('reportSelect').onchange=async()=>{const id=$('reportSelect').value;if(!id){$('reportView').innerHTML='';return;}const rr=await fetch('/api/test?id='+encodeURIComponent(id));const t=await rr.json();$('reportView').innerHTML='<div class="panel"><div class="title">'+t.testId+' Report</div><div class="mini-grid"><div class="mini"><div class="label">Building / Wall</div><div class="value">'+t.building+' / '+t.wall+'</div></div><div class="mini"><div class="label">Score</div><div class="value">'+t.score+'/100</div></div><div class="mini"><div class="label">Grade</div><div class="value">'+t.condition+'</div></div></div><div class="actions" style="margin-top:12px"><button class="btn primary" onclick="window.location.href=\'/download?id='+encodeURIComponent(id)+'\'">DOWNLOAD CSV</button><button class="btn ghost" onclick="printReport()">PRINT / SAVE PDF</button></div></div>'};}catch(e){$('reportView').textContent='Unable to load reports';}}
async function loadSettings(){try{const r=await fetch('/api/status?x='+Date.now());const d=await r.json();$('setThreshold').value=d.tapThresholdG||0.35;$('setDry').value=d.moistureDry||3000;$('setWet').value=d.moistureWet||1200;$('deviceInfo').innerHTML='<div class="sub">IP: '+(d.ip||'--')+' • Mode: '+(d.mode||'--')+' • MPU6050: '+(d.mpuReady?'READY':'NOT DETECTED')+'</div>';}catch(e){$('deviceInfo').textContent='Unable to load device information';}}
async function saveSettings(){try{await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({tapThresholdG:parseFloat($('setThreshold').value)||0.35,moistureDry:parseInt($('setDry').value)||3000,moistureWet:parseInt($('setWet').value)||1200})});alert('Settings saved');}catch(e){alert('Unable to save settings');}}
async function relayToggle(btn){const r=Number(btn.dataset.relay),state=btn.dataset.state!=='1';try{await fetch('/api/relay',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({relay:r,state})});btn.dataset.state=state?'1':'0';btn.textContent=state?'ON':'OFF';}catch(e){alert('Relay command failed');}}
function printReport(){if(!currentTest){alert('Select a test first');return;}const t=currentTest;const w=window.open('','_blank');if(!w){alert('Allow popups in the browser');return;}const rows=(t.taps||[]).map(x=>'<tr><td>'+x.tapNumber+'</td><td>'+Number(x.peak||0).toFixed(2)+'</td><td>'+Number(x.dominantFrequency||0).toFixed(2)+' Hz</td><td>'+Number(x.dominantAmplitude||0).toFixed(2)+' g</td><td>'+Number(x.moisturePercent||0).toFixed(1)+'%</td><td>'+x.tapScore+'</td><td>'+x.tapCondition+'</td></tr>').join('');w.document.write('<html><head><title>CrackAura Report</title><style>body{font-family:Arial;padding:24px}table{width:100%;border-collapse:collapse}th,td{border:1px solid #ccc;padding:8px}</style></head><body><h1>CrackAura Wall Health Monitoring Report</h1><p>Test ID: '+t.testId+'<br>Building: '+t.building+'<br>Wall: '+t.wall+'<br>Date: '+t.date+' '+t.time+'</p><h2>Results</h2><p>Score: '+t.score+'/100 &nbsp; Grade: '+t.condition+'</p><p>Dominant Frequency: '+Number(t.dominantFrequency||0).toFixed(2)+' Hz<br>Dominant Amplitude: '+Number(t.dominantAmplitude||0).toFixed(2)+' g</p><table><tr><th>Tap</th><th>Peak</th><th>Frequency</th><th>Amplitude</th><th>Moisture</th><th>Score</th><th>Grade</th></tr>'+rows+'</table></body></html>');w.document.close();w.focus();w.print();}
$('csvBtn').onclick=()=>{if(currentId)window.location.href='/download?id='+encodeURIComponent(currentId);else alert('No completed test');};$('printBtn').onclick=printReport;$('saveSettings').onclick=saveSettings;document.querySelectorAll('.relay').forEach(b=>b.onclick=()=>relayToggle(b));
async function init(){wallSelect('Wall 1');await loadSettings();await poll();timer=setInterval(poll,300)}init();window.onresize=()=>{if(currentTest)renderAnalysis(currentTest)};

</script></body></html>

)rawliteral";

void handleRoot(){server.send_P(200,"text/html",CRACKAURA_HTML);}void startBody(){String b=server.arg("plain");JsonDocument d;if(deserializeJson(d,b)){server.send(400,"application/json","{\"ok\":false,\"error\":\"Invalid JSON\"}");return;}if(testRunning){server.send(409,"application/json","{\"ok\":false,\"error\":\"Test already running\"}");return;}currentBuilding=String(d["building"]|"Building A");currentBuilding=currentBuilding.substring(0,31);currentWall=String(d["wall"]|"Wall 1");currentWall=currentWall.substring(0,31);float th=d["tapThresholdG"]|tapThresholdG;if(th>=0.05&&th<=3.0)tapThresholdG=th;currentTestId=nextId();tapCount=0;testRunning=true;testComplete=false;captureActive=false;lastTapMs=0;lastRelayToggleMs=0;autoRelayState=false;relay1State=false;relayPin(PIN_RELAY_1,false);for(int i=0;i<5;i++)taps[i]=TapRecord();for(int i=0;i<=CAPTURE_SAMPLES/2;i++)lastSpectrum[i]=0;digitalWrite(PIN_STATUS_LED,HIGH);JsonDocument o;o["ok"]=true;o["testId"]=currentTestId;o["tapCount"]=0;o["tapsPerTest"]=5;o["building"]=currentBuilding;o["wall"]=currentWall;String out;serializeJson(o,out);server.send(200,"application/json",out);}

void handleStop(){testRunning=false;captureActive=false;lastStatusText="STOPPED";lastRelayToggleMs=0;autoRelayState=false;digitalWrite(PIN_STATUS_LED,LOW);allOff();server.send(200,"application/json","{\"ok\":true}");}

void handleHistory(){File f=LittleFS.open(TESTS_INDEX_FILE,"r");if(!f){server.send(200,"application/json","[]");return;}String o=f.readString();f.close();server.send(200,"application/json",o);}

void handleTest(){if(!server.hasArg("id")){server.send(400,"application/json","{\"error\":\"missing id\"}");return;}String id=server.arg("id");String p=String(TESTS_DIR)+"/"+id+".json";File f=LittleFS.open(p,"r");if(!f){server.send(404,"application/json","{\"error\":\"not found\"}");return;}String o=f.readString();f.close();server.send(200,"application/json",o);}

void handleDownload(){if(!server.hasArg("id")){server.send(400,"text/plain","Missing id");return;}String id=server.arg("id"),p=String(TESTS_DIR)+"/"+id+".json";File f=LittleFS.open(p,"r");if(!f){server.send(404,"text/plain","Not found");return;}JsonDocument d;deserializeJson(d,f);f.close();String c="CrackAura Wall Health Monitoring Report\n";c+="Test ID,"+String(d["testId"]|"")+"\nBuilding,"+String(d["building"]|"")+"\nWall,"+String(d["wall"]|"")+"\nDate,"+String(d["date"]|"")+"\nTime,"+String(d["time"]|"")+"\nOverall Score,"+String((int)(d["score"]|0))+"/100\nCondition,"+String(d["condition"]|"")+"\nDominant Frequency,"+String((float)(d["dominantFrequency"]|0),2)+" Hz\nDominant Amplitude,"+String((float)(d["dominantAmplitude"]|0),2)+" g\n\nTap,X,Y,Z,Resultant,Peak,Frequency,Amplitude,Moisture,Score,Condition\n";for(JsonObject t:d["taps"].as<JsonArray>()){c+=String((int)(t["tapNumber"]|0))+","+String((float)(t["x"]|0),3)+","+String((float)(t["y"]|0),3)+","+String((float)(t["z"]|0),3)+","+String((float)(t["resultant"]|0),3)+","+String((float)(t["peak"]|0),3)+","+String((float)(t["dominantFrequency"]|0),3)+","+String((float)(t["dominantAmplitude"]|0),3)+","+String((float)(t["moisturePercent"]|0),2)+","+String((int)(t["tapScore"]|0))+","+String(t["tapCondition"]|"")+"\n";}server.sendHeader("Content-Disposition","attachment; filename="+id+".csv");server.send(200,"text/csv",c);}

void handleLive(){server.send(200,"application/json",makeLive());}

void handleStatus(){JsonDocument d;d["ip"]=WiFi.localIP().toString();d["mode"]="STA";d["ssid"]=WiFi.SSID();d["relay1"]=relay1State;d["relay2"]=relay2State;d["tapThresholdG"]=tapThresholdG;d["moistureDry"]=moistureDry;d["moistureWet"]=moistureWet;d["relayActiveLow"]=(bool)RELAY_ACTIVE_LOW;d["solenoidChannel"]=SOLENOID_RELAY_CHANNEL;d["mpuReady"]=mpuReady;d["state"]=testRunning?"WAITING_FOR_TAP":(testComplete?"TEST_COMPLETE":"IDLE");String o;serializeJson(d,o);server.send(200,"application/json",o);}

void handleSettings(){JsonDocument d;deserializeJson(d,server.arg("plain"));float th=d["tapThresholdG"]|tapThresholdG;int dr=d["moistureDry"]|moistureDry,wt=d["moistureWet"]|moistureWet;if(th>=0.05&&th<=3)tapThresholdG=th;if(dr>=0&&dr<=4095)moistureDry=dr;if(wt>=0&&wt<=4095)moistureWet=wt;saveSettings();server.send(200,"application/json","{\"ok\":true}");}

void handleRelay(){JsonDocument d;deserializeJson(d,server.arg("plain"));int r=d["relay"]|0;bool st=d["state"]|false;if(r==1){relay1State=st;relayPin(PIN_RELAY_1,st);}if(r==2){relay2State=st;relayPin(PIN_RELAY_2,st);}server.send(200,"application/json","{\"ok\":true}");}

void wifi(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  unsigned long t=millis();
  while(WiFi.status()!=WL_CONNECTED&&millis()-t<WIFI_CONNECT_TIMEOUT_MS)delay(250);
  if(WiFi.status()==WL_CONNECTED){
    configTime(GMT_OFFSET_SEC,DAYLIGHT_OFFSET_SEC,"pool.ntp.org","time.nist.gov");
    RealSerial.print("your wifi IP: ");
    RealSerial.println(WiFi.localIP());
  } else {
    RealSerial.print("WiFi connect failed, status=");
    RealSerial.println((int)WiFi.status());
  }
}

void setup() {

  RealSerial.begin(115200);

  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_SOIL_MOISTURE, INPUT);
  pinMode(PIN_VIBRATION, INPUT);

  pinMode(PIN_RELAY_1, OUTPUT);
  pinMode(PIN_RELAY_2, OUTPUT);

  allOff();

  // =====================================================
  // LITTLEFS
  // =====================================================

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS failed");
    while (1) delay(1000);
  }

  storage();
  loadSettings();

  // =====================================================
  // MPU6500-COMPATIBLE SENSOR (raw driver, see notes above)
  // SDA = GPIO 21
  // SCL = GPIO 22
  // ADDRESS = 0x68
  // =====================================================

  Serial.println();
  Serial.println("================================");
  Serial.println("Initializing MPU6500-compatible sensor");
  Serial.println("SDA : GPIO 21");
  Serial.println("SCL : GPIO 22");
  Serial.println("ADDR: 0x68");
  Serial.println("================================");

  Wire.begin(PIN_SDA, PIN_SCL);
  delay(100);

  mpuReady = mpu6500Init();

  if (mpuReady) {
    Serial.println("Sensor initialized!");
    delay(100);
    Serial.println("Calibrating...");
    calibrate();
    Serial.print("Baseline Resultant: ");
    Serial.print(baselineResultantG, 3);
    Serial.println(" g");
    Serial.println("SENSOR READY");
  } else {
    Serial.println("********************************");
    Serial.println("SENSOR INIT FAILED (I2C write error)");
    Serial.println("Check:");
    Serial.println("SDA -> GPIO 21");
    Serial.println("SCL -> GPIO 22");
    Serial.println("VCC -> 3.3V");
    Serial.println("GND -> GND");
    Serial.println("Address -> 0x68");
    Serial.println("********************************");
  }

  wifi();
  server.on("/", HTTP_GET, handleRoot);

  server.on("/api/start", HTTP_POST, startBody);

  server.on("/api/stop", HTTP_POST, handleStop);

  server.on("/api/live", HTTP_GET, handleLive);

  server.on("/api/status", HTTP_GET, handleStatus);

  server.on("/api/history", HTTP_GET, handleHistory);

  server.on("/api/test", HTTP_GET, handleTest);

  server.on("/api/settings", HTTP_POST, handleSettings);

  server.on("/api/relay", HTTP_POST, handleRelay);

  server.on("/api/alloff", HTTP_POST, handleStop);

  server.on("/download", HTTP_GET, handleDownload);

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });

  server.begin();

  // No additional Serial output. IP address is printed by wifi().
}
void loop(){unsigned long n=millis();if(n-lastSensorMs>=SAMPLE_PERIOD_MS){lastSensorMs=n;sensor();}automaticRelayControl();server.handleClient();}
