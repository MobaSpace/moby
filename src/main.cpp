#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <math.h>

//#include "NeuronalNetwork.h"

//=========== SLEEP MODE PARAMETERS ====================
const u_short secs_to_sleep = 8;
const u_short secs_to_work = 8;
const u_int us_TIME_TO_SLEEP = secs_to_sleep * 1000000;

const float STD_GG_WIND_THRS = 0.008;
const float STD_ANG_WIND_THRS = 0.005;
long not_sleep_before = 0;

//====== MACHINE STATE FALL DETECTION ======
#define MS_NORMAL         0
#define MS_FALLING        1
#define MS_POST_FALLING   2
#define MS_IMPACT         3
#define MS_MOTIONLESS     4
#define MS_FALL_DETECTED  5

//===== fall detection Thresholds =======
const float it = 2.5; //impact threshold
const float umt = 1.5; //upper Motionless thr
const float lmt = 0.5; //lower Motionless thr
const long mpf = 100; //millis Post-falling
const long mi = 2000; //millis Impact
const long mml = 2000; //millis Motionless

const int MM = 400; //samples for falldown analysis
float shockWindow[MM]; //8s window
int state = MS_NORMAL; //current machine state
unsigned long stateReachedAt; //time of last state change
int arrState[MM]; //vector of machine state

//angles for attitude
float ang_1[MM];
float ang_2[MM];
float ang_3[MM];

//============== STEP COUNTING PARAMETERS =====================
float w[5]; //values for Butterworth filter
double AA, d1, d2, d3, d4;

const int NN = 21; //windows for step counting analysis >330ms
float GG = 9.81; //Earth gravity
float acc_filtered[NN]; //vector containing filterd acc values
int num_steps = 0; //number of steps since last HTTP/POST
const int period = 20; //milliseconds for sampling period @50Hz seems good
unsigned long lastStepAt = 0; //dernier pas detecté

//Accelerometer sensor
Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel;

//step detection thresholds
float thr_sym = 0.3;
float thr_std = 0.8;
short thr_win = 300; //milliseconds


//============== WIFI SCAN RSSI Indoor Positionning ===============
int8_t rssi_val[10];
String bssid[10];
uint8_t n_ap;

//temps pour envoi données (chaque heure) ou alarme en cas de chute
unsigned long lastTime = 0;
unsigned long timerDelay = 3600*1000; // number of seconds because millis()

bool wifiOK = false;

//================= CONFIGURATIONS SPECIFIQUES COMM =====================
#define DEBUG true
const char* ssid = "SFR_3E20";
const char* password = "kaix2kee89czxd6gqg98";
const char* serverName = "https://demo.mobaspace.com/withings_evt";
//const char* serverName = "http://192.168.1.73:5100/withings_evt";


void iniGravityValue(){
  sensors_event_t accel;
  GG = 0.0;
  for(int ii=0; ii<100; ii++){
    mpu_accel->getEvent(&accel);
    GG += sqrt(
      accel.acceleration.x * accel.acceleration.x +
      accel.acceleration.y * accel.acceleration.y +
      accel.acceleration.z * accel.acceleration.z);
    delay(10);
  }
  GG /= 100.;
}

float calculateSD(float* arr, int size) {
  float mean = 0.0;
  float SD = 0.0;
  int i;
  for (i = 0; i < size; ++i) {
      mean += arr[i];
  }
  mean /= size;
  for (i = 0; i < size; ++i)
      SD += pow(arr[i] - mean, 2);
  return sqrt(SD / size);
}

bool isStep(){
  int ii;
  
  //main and secondary peak detection
  float main_peak_val = -99.9;
  float secd_peak_val = -99.9;
  int i_main_peak = -1;
  int i_secd_peak = -1;
  
  //finding mean peak
  for(ii = 1; ii<NN-1; ii++){
    if(acc_filtered[ii-1] < acc_filtered[ii] &&  acc_filtered[ii] > acc_filtered[ii+1])
      if (acc_filtered[ii] > main_peak_val){
        main_peak_val = acc_filtered[ii];
        i_main_peak = ii;
      }
  }

  //finding secondary peak
  for(ii = 1; ii<NN-1; ii++){
    if(acc_filtered[ii-1] < acc_filtered[ii] &&  acc_filtered[ii] > acc_filtered[ii+1])
      if (acc_filtered[ii] > secd_peak_val && acc_filtered[ii] < main_peak_val){
        secd_peak_val = acc_filtered[ii];
        i_secd_peak = ii;
      }
  }

  //check if main peak is centered
  if (i_main_peak < (NN/2 - 2) || i_main_peak > (NN/2 + 2) )
    return false;

  //float diff_1 = i_main_peak;
  //float diff_2 = NN - i_main_peak;
  //if ( diff_1 / diff_2 > thr_sym || diff_2 / diff_1 > thr_sym)
  //  return false;

  if( calculateSD(acc_filtered, NN) < thr_std)
    return false;

  //check that new step is far enough from previous
  if( millis() - lastStepAt < thr_win)
    return false;

  //check if two similar peaks very closer --> not really a step
  if (secd_peak_val / main_peak_val > 0.75 && abs(i_secd_peak - i_main_peak) < 10 && i_main_peak > 0 && i_secd_peak > 0)
    return false;
  
  return true;
}

//determination des coefficients de filtrage passe bande Butterworth
// https://exstrom.com/journal/sigproc/
// f1 upper freq
// f2 lower freq --> f1 > f2
// order has been fixed to n=4
void iniButterworth(double s, double f1, double f2){
  double a = cos(PI*(f1+f2)/s)/cos(PI*(f1-f2)/s);
  double a2 = a*a;
  double b = tan(PI*(f1-f2)/s);
  double b2 = b*b;

  double r = sin(PI/4.0);
  double ss = b2 + 2.0*b*r + 1.0;

  AA = b2/ss;
  d1 = 4.0*a*(1.0+b*r)/ss;
  d2 = 2.0*(b2-2.0*a2-1.0)/ss;
  d3 = 4.0*a*(1.0-b*r)/ss;
  d4 = -(b2 - 2.0*b*r + 1.0)/ss;
  if(DEBUG){
    Serial.println("Butterworth coeff: ");
    Serial.print("A=");
    Serial.println(AA, 4);
    Serial.print("d1=");
    Serial.println(d1, 4);
    Serial.print("d2=");
    Serial.println(d2, 4);
    Serial.print("d3=");
    Serial.println(d3, 4);
    Serial.print("d4=");
    Serial.println(d4, 4);
  }
}

float butterworthFilter(float new_acc){

  w[0] = d1*w[1] + d2*w[2]+ d3*w[3]+ d4*w[4] + new_acc;
  double x = AA*(w[0] - 2.0*w[2] + w[4]);
  w[4] = w[3];
  w[3] = w[2];
  w[2] = w[1];
  w[1] = w[0];

  return x;
}


void wifiDisconnect(){
  //mise off de toutes les connexions radio
  //avec la lib de haut niveau et bas niveau
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  btStop();
  esp_bt_controller_disable();
  wifiOK = false;
}

void wifiConnect(){
  wifiOK = false;
  int timeOut_wifi_connect = 0;
  if (DEBUG){
    Serial.println("Trying to connect to: " + String(ssid));
    Serial.println("Ussing Password: " + String(password));
  }

  // Configures static IP address
//  if (!WiFi.config(local_IP, gateway, subnet)) {
//    Serial.println("STA Failed to configure");
//  }
  
  WiFi.begin(ssid, password);
  if (DEBUG)
    Serial.println("MAC ADDRESS = " + WiFi.macAddress());
  while(WiFi.status() != WL_CONNECTED && timeOut_wifi_connect < 2*10) {
    delay(500);
    if (DEBUG) 
      Serial.print(".");
    timeOut_wifi_connect ++;
  }
  if (DEBUG)
    Serial.println();

  if ( WiFi.status() == WL_CONNECTED){
    if(DEBUG){
      Serial.println("");
      Serial.print("Connected to WiFi network with IP Address: ");
      Serial.println(WiFi.localIP());
    }
    wifiOK = true;
  }

  else{
    if (DEBUG)
      Serial.println("Impossible to connect to WiFi network!!");        
  }
}

void scanWiFi(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  n_ap = WiFi.scanNetworks();
 
  if (n_ap > 0) {
    if(DEBUG){
      Serial.print(n_ap);
      Serial.println(" networks found");
    }
    for (int i = 0; i < n_ap; ++i) {
      rssi_val[i] = WiFi.RSSI(i);
      bssid[i] = WiFi.BSSIDstr(i);
      if(DEBUG){
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print( WiFi.BSSIDstr(i) );
        Serial.print(" (");
        Serial.print(WiFi.RSSI(i));
        Serial.print(") ");
        Serial.print(" [");
        Serial.print(WiFi.channel(i));
        Serial.print("] ");
      }
    }
  }
  wifiDisconnect();
} 

int sendData(){
  wifiConnect();
  if(WiFi.status()== WL_CONNECTED){
      HTTPClient http;
      http.setConnectTimeout(6000);
      // Your Domain name with URL path or IP address with path
      http.begin(serverName);
      http.setTimeout(6000);
      //delay(1000);
      // Specify content-type header
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");

      String str_st = "False";
      if (state == MS_FALL_DETECTED) str_st = "True";
      
      //Prepare data for Wifi Access Points
      String ap_bssid = "";
      String ap_rssi = "";
      if(n_ap > 0){
        ap_bssid = bssid[0];
        ap_rssi = String(rssi_val[0]);

        for(int ii=1; ii<n_ap; ii++){
          ap_bssid += "," + bssid[ii];
          ap_rssi += "," + String(rssi_val[ii]);
        }
      }

      // Data to send with HTTP POST
      String httpRequestData =  "appli=777&devId=" + WiFi.macAddress() + 
        "&steps=" + String(num_steps) +
        "&bssid=" + ap_bssid +
        "&rssi=" + ap_rssi +
        "&falldown=" + str_st;

      if (DEBUG) 
        Serial.println(httpRequestData);
      
      // Send HTTP POST request
      int httpResponseCode = http.POST(httpRequestData);

      //to verify in detail, very often readTimeOut!!!
      //but the POST is well received on the server
      //so don't need to do the request again...
      if (httpResponseCode < 0)
        httpResponseCode = 200;

      if (DEBUG){
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        Serial.println(http.getString());
      }
      
      // Free resources
      http.end();
      //WiFi.disconnect();
      wifiDisconnect();
      return httpResponseCode;   
  }
  else {
    if (DEBUG)
      Serial.println("WiFi Disconnected --> Impossible to send data");
    //WiFi.disconnect();
    wifiDisconnect();
    return -1;
  } 
}

//evaluation of the new machine state
void updateMachineState (){
  float acc = shockWindow[MM-1];
  unsigned long currentTime = millis();
  switch (state){
    case MS_NORMAL:
      if ( acc < lmt ){
          state = MS_FALLING;
          stateReachedAt = currentTime;
      }
      break;

    case MS_FALLING:
      if (acc > it){
          state = MS_IMPACT;
          stateReachedAt = currentTime;
      }

      else if (acc > lmt){
          state = MS_POST_FALLING;
          stateReachedAt = currentTime;
      }
      break;

    case MS_POST_FALLING:
      if (acc > it){
          state = MS_IMPACT;
          stateReachedAt = currentTime;
      }
      else if (acc < lmt){
          state = MS_FALLING;
          stateReachedAt = currentTime;
      }
      else if (currentTime > stateReachedAt + mpf){
          state = MS_NORMAL;
          stateReachedAt = currentTime;
      }
      break;

    case MS_IMPACT:
      if (acc > it){
          state = MS_IMPACT;
          //stateReachedAt = currentTime;
      }
      else if (currentTime > stateReachedAt + mi){
          state = MS_MOTIONLESS;
          stateReachedAt = currentTime;
      }
      break;

    case MS_MOTIONLESS:
      if (acc < lmt){
          state = MS_FALLING;
          stateReachedAt = currentTime;
      }
      else if (acc > umt){
          state = MS_NORMAL;
          stateReachedAt = currentTime;
      }
      else if (currentTime > stateReachedAt + mml){
          state = MS_FALL_DETECTED;
          stateReachedAt = currentTime;
      
          if (DEBUG){
            //print the array
            Serial.println("The shock data is next:");
            for(int ii=0; ii<MM; ii++){
              Serial.print(shockWindow[ii],4);
              Serial.print(";");
            }
            Serial.println();

            Serial.println("The state machine is:");
            for(int ii=0; ii<MM; ii++){
              Serial.print(arrState[ii]);
              Serial.print(";");
            }
            Serial.println();
          }
      }
      break;
  }
  for (int ii=0; ii<MM-1; ii++)
    arrState[ii] = arrState[ii+1];
  arrState[MM-1] = state;
}

void resetMachine(){
  state = MS_NORMAL;
  stateReachedAt = millis();
}


void setup() {
  // ========== Reduction of power ============
  setCpuFrequencyMhz(80); 
  btStop();
  esp_bt_controller_disable();
  //wifiDisconnect(); NEVER DO WiFi.disconnect() before a connection --> PANIC!!!

  esp_sleep_enable_timer_wakeup(us_TIME_TO_SLEEP); // ESP32 wakes up every 10 seconds

  //ini values of Butterworth filter
  for(int i=0; i <= 4; i++)
        w[i]=0.0;

  //ini values for step counting Acc-GG ~ 0
  for(int ii = 0; ii < NN; ii++){
    acc_filtered[ii] = 0.0;
  }
  //ini values for Falldown   Acc/GG ~ 1
  //ini angles
  for(int ii= 0; ii < MM; ii++){
    shockWindow[ii] = 1.0;
    ang_1[ii] = 0.0;
    ang_2[ii] = 0.0;
    ang_3[ii] = 0.0;
  }
  
  Serial.begin(115200);

  iniButterworth(1000.0/period, 3.0, 0.75);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  if(DEBUG){
    Serial.println("MPU6050 Found!");
    mpu_accel = mpu.getAccelerometerSensor();
    mpu_accel->printSensorDetails();
    
    //Test du WiFi
    wifiConnect();
    wifiDisconnect();
  }

  //can be different from 9.81!!!
  iniGravityValue();

  lastTime = millis();
  not_sleep_before = millis() + 2 * secs_to_work * 1000;
}


void loop() {

  //long ini_time = millis();
  
  //si la personne se deplace bcp, si elle tombe ou bien 1h sans avoir envoyé de données
  //priorité à l'envoie de la donnée tant que ceci n'est pas fait (code == 200)
  if ((millis() - lastTime) > timerDelay || num_steps >= 20 || state == MS_FALL_DETECTED) {
    scanWiFi();
    if(sendData() == 200){
      num_steps = 0;
      lastTime = millis();
      lastStepAt = millis();
      resetMachine();
    }
  }

  //sinon traitement de l'acceleromètre
  else{
    // Get a new normalized sensor event
    sensors_event_t accel;
    mpu_accel->getEvent(&accel);

    //==== ANGLE PROCESSING ====
    for(int ii = 0; ii < MM-1; ii++){
      ang_1[ii] = ang_1[ii+1];
      ang_2[ii] = ang_2[ii+1];
      ang_3[ii] = ang_3[ii+1];
    }
    ang_1[MM-1] = atan(accel.acceleration.y / 
                    sqrt(accel.acceleration.x * accel.acceleration.x +
                    accel.acceleration.z * accel.acceleration.z));

    ang_2[MM-1] = atan(accel.acceleration.x / 
                    sqrt(accel.acceleration.y * accel.acceleration.y +
                    accel.acceleration.z * accel.acceleration.z));

    ang_3[MM-1] = atan(accel.acceleration.z / 
                    sqrt(accel.acceleration.x * accel.acceleration.x +
                    accel.acceleration.y * accel.acceleration.y)); 

    // ==== Resultant Acceleration ===========
    float acc_res = sqrt(
      accel.acceleration.x * accel.acceleration.x +
      accel.acceleration.y * accel.acceleration.y +
      accel.acceleration.z * accel.acceleration.z);
      
    //==== SHOCK PROCESSING ====
    for(int ii = 0; ii < MM-1; ii++){
      shockWindow[ii] = shockWindow[ii+1];
    }
    shockWindow[MM-1] = acc_res / GG;

    updateMachineState();

    //==== STEP COUNTING PROCESSING ====
    acc_res = butterworthFilter(acc_res - GG);
    
    //shift vector of accelerations
    for(int ii = 0; ii < NN-1; ii++){
      acc_filtered[ii] = acc_filtered[ii+1];
    }
    acc_filtered[NN-1] = acc_res;
    
    if (isStep()){
      num_steps ++;
      lastStepAt = millis();
      if(DEBUG){
        Serial.print("STEP DETECTED--> total_steps=");
        Serial.println(num_steps);
        //PRINT ACC ARRAY
        //print the array
        Serial.println("The step window data is next:");
        for(int ii=0; ii<NN; ii++){
          Serial.print(acc_filtered[ii], 4);
          Serial.print(";");
        }
        Serial.println();
      }
    }
  }

  //test if variability of acc and angles is enought flat for going to sleep
  //also that ESP32 has measure a cycle of 8s sins last sleeping
  if(calculateSD(shockWindow, MM) < STD_GG_WIND_THRS &&
    calculateSD(ang_1, MM) < STD_ANG_WIND_THRS &&
    calculateSD(ang_2, MM) < STD_ANG_WIND_THRS &&
    calculateSD(ang_3, MM) < STD_ANG_WIND_THRS &&
    millis() > not_sleep_before){
    if(DEBUG){
      Serial.println("--> GOING TO SLEEP for 8s!! ");
      Serial.print("Ang_1=");
      Serial.println(ang_1[MM-1], 4);
      Serial.print("Acc_raw=");
      Serial.println(shockWindow[MM-1], 4);
      delay(5);
    }
    wifiDisconnect();
    esp_light_sleep_start();
    not_sleep_before = millis() + secs_to_work*1000;
  }
  else{
    //pour échantilloner tjrs à la bonne fréquence
    delay( period );// - (millis()-ini_time) );
  }
}