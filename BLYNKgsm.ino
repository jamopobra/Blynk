/*  V0  BATERIA - V1  TERMINAL - V2  POSICION - V3  SENSORES - V4  RELES  */
/* Completa a información de [Blynk Device Info] aquí */
#define BLYNK_TEMPLATE_ID "TMPLxxxxxxxxx"
#define BLYNK_TEMPLATE_NAME "nome"
#define BLYNK_AUTH_TOKEN "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
#define TINY_GSM_MODEM_SIM808

#include <TinyGsmClient.h>       //https://github.com/vshymanskyy/TinyGSM
#include <BlynkSimpleTinyGSM.h>  //https://github.com/blynkkk/blynk-library
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

char auth[] = BLYNK_AUTH_TOKEN;

const int REL1 = PA12;  //PA12, PA15, PB3, PB4 Saídas para reles
const int REL2 = PA15;
const int REL3 = PB3;
const int REL4 = PB4;
const int PIR1 = PB12;  //PB12, PB13, PB14, PB15 Pins STM32 tolerantes a 5Vcc entradas de sensores PIR
const int PIR2 = PB13;
const int PIR3 = PB14;
const int PIR4 = PB15;

float fijapos_Latitude = 0.0F;
float fijapos_Lonxitude = 0.0F;
float fijapos_exactitude = 0.0F;
int fijapos_ano = 0;
int fijapos_mes = 0;
int fijapos_dia = 0;
int fijapos_hora = 0;
int fijapos_minuto = 0;
int fijapos_segundo = 0;

float gps_latitude = 0.0F;
float gps_lonxitude = 0.0F;
float gps_velocidade = 0.0F;
float gps_altitude = 0.0F;
float gps_exactitude = 0.0F;
int gps_vsat = 0;
int gps_usat = 0;
int gps_ano = 0;
int gps_mes = 0;
int gps_dia = 0;
int gps_hora = 0;
int gps_minuto = 0;
int gps_segundo = 0;
unsigned long duracion = 60000UL;


Adafruit_ADS1115 ads;  //pins B7(SDA) e B6(SCL)
const float multiplier = 0.1875F;
float volt_0;

// As credenciais GPRS
// Deixar valeiro, si falla usuario ou contrasinal
char apn[] = "internet";
char user[] = "";
char pass[] = "";

#define SerialAT Serial1
TinyGsm modem(SerialAT);

BlynkTimer timer;
WidgetTerminal terminal(V1);
BLYNK_WRITE(V1) {
  if (String("?") == param.asStr()) {
    terminal.println("? > Menu general");
    terminal.println("rele? > Informe estado reles");
    terminal.println("sensor? > Informe estado sensores");
    terminal.println("pos? > posicion actual");
    terminal.println("fijapos > fija posicion actual");
    terminal.println("dcham1 dcham2 dcham3 > fija duracion chamada en minutos ");
    terminal.println("rel1 a rel4 1 ou 0 > encende/apaga reles");
    terminal.println("exemplo: rel01 rel00 rel11 rel10... etc");    
    terminal.flush();
  }
  if (String("dcham1") == param.asStr()) {
    terminal.println("Dur. chamadas = 1m");
    duracion = 60000;
    terminal.flush();
  }

  if (String("dcham2") == param.asStr()) {
    terminal.println("Dur. chamadas = 2m");
    duracion = 120000;
    terminal.flush();
  }

  if (String("dcham3") == param.asStr()) {
    terminal.println("Dur. chamadas = 3m");
    duracion = 180000;
    terminal.flush();
  }

  if (String("rele?") == param.asStr()) {
    terminal.println("Informe estado reles...");
    estado_reles();
    terminal.flush();
  }
  if (String("sensor?") == param.asStr()) {
    terminal.println("Informe estado sensores...");
    estado_sensores();
    terminal.flush();
  }
  if (String("pos?") == param.asStr()) {
    terminal.println("Informe posicion actual...");
    posicion_actual();
    terminal.flush();
  }
  if (String("fijapos") == param.asStr()) {
    terminal.println("fija posicion...");
    fija_posicion();
    terminal.flush();
  }
  if (String("rel10") == param.asStr()) {
    terminal.println("Apaga Rele 1...");
    if (digitalRead(REL1 == HIGH)) { digitalWrite(REL1, LOW); }
    terminal.flush();
  }
  if (String("rel11") == param.asStr()) {
    terminal.println("Encende Rele 1...");
    if (digitalRead(REL1 == LOW)) { digitalWrite(REL1, HIGH); }
    terminal.flush();
  }
  if (String("rel20") == param.asStr()) {
    terminal.println("Apaga Rele 2...");
    if (digitalRead(REL2 == HIGH)) { digitalWrite(REL2, LOW); }
    terminal.flush();
  }
  if (String("rel21") == param.asStr()) {
    terminal.println("Encende Rele 2...");
    if (digitalRead(REL2 == LOW)) { digitalWrite(REL2, HIGH); }
    terminal.flush();
  }
  if (String("rel30") == param.asStr()) {
    terminal.println("Apaga Rele 3...");
    if (digitalRead(REL3 == HIGH)) { digitalWrite(REL3, LOW); }
    terminal.flush();
  }
  if (String("rel31") == param.asStr()) {
    terminal.println("Encende Rele 3...");
    if (digitalRead(REL3 == LOW)) { digitalWrite(REL3, HIGH); }
    terminal.flush();
  }
  if (String("rel40") == param.asStr()) {
    terminal.println("Apaga Rele 4...");
    if (digitalRead(REL4 == HIGH)) { digitalWrite(REL4, LOW); }
    terminal.flush();
  }
  if (String("rel41") == param.asStr()) {
    terminal.println("Encende Rele 4...");
    if (digitalRead(REL4 == LOW)) { digitalWrite(REL4, HIGH); }
    terminal.flush();
  }
}

void setup() {
  //Serial.begin(115200);  //Rx(PA10), Rx(PA9) Consola de depuración:
  SerialAT.begin(9600);  //Rx(PA3), Tx(PA2)
  delay(3000);
  ads.setGain(GAIN_TWOTHIRDS);  // +/- 6.144V  1 bit = 0.1875mV (default)
  ads.begin();
  delay(10);

  pinMode(REL1, OUTPUT);  //Rele 1
  pinMode(REL2, OUTPUT);  //Rele 2
  pinMode(REL3, OUTPUT);  //Rele 3
  pinMode(REL4, OUTPUT);  //Rele 4

  pinMode(PIR1, INPUT);  //PIR 1
  pinMode(PIR2, INPUT);  //PIR 2
  pinMode(PIR3, INPUT);  //PIR 3
  pinMode(PIR4, INPUT);  //PIR 4

  modem.init();
  delay(1000L);
  modem.enableGPS();
  delay(15000L);

  modem.simUnlock("9999");  //PIN SIM 

  Blynk.config(modem, BLYNK_AUTH_TOKEN);
  Blynk.begin(BLYNK_AUTH_TOKEN, modem, apn, user, pass);
  
  // Define a función para refrescar cada X segundos
  timer.setInterval(20000L, voltimetro);
  timer.setInterval(10000L, distancia);
  timer.setInterval(10000L, rele);
  timer.setInterval(2000L, sensorPIR);
}

void loop() {
  Blynk.run();
  timer.run();                     // Inicia BlynkTimer
  if (SerialAT.available() > 0) {  // Check si hai algunha chamada entrante
    String chamada = SerialAT.readStringUntil('\n');
    if (chamada.indexOf("RING") >= 0) {
      Atende_Chamada();
    }
  }
}

void Atende_Chamada() {
  SerialAT.println("ATA");
  delay(duracion);
  SerialAT.println("ATH");
}

void estado_reles() {
  int REL_1 = digitalRead(REL1);
  int REL_2 = digitalRead(REL2);
  int REL_3 = digitalRead(REL3);
  int REL_4 = digitalRead(REL4);
  if (REL_1 == HIGH) { terminal.println("Rele 1 ENCENDIDO "); }
  if (REL_1 == LOW) { terminal.println("Rele 1 Apagado "); }
  if (REL_2 == HIGH) { terminal.println("Rele 2 ENCENDIDO "); }
  if (REL_2 == LOW) { terminal.println("Rele 2 Apagado "); }
  if (REL_3 == HIGH) { terminal.println("Rele 3 ENCENDIDO "); }
  if (REL_3 == LOW) { terminal.println("Rele 3 Apagado "); }
  if (REL_4 == HIGH) { terminal.println("Rele 4 ENCENDIDO "); }
  if (REL_4 == LOW) { terminal.println("Rele 4 Apagado "); }
  terminal.flush();
}

void estado_sensores() {
  int pir_1 = digitalRead(PIR1);
  int pir_2 = digitalRead(PIR2);
  int pir_3 = digitalRead(PIR3);
  int pir_4 = digitalRead(PIR4);
  if (pir_1 == HIGH) { terminal.println("¡ Sensor 1 ENCENDIDO ! "); }
  if (pir_1 == LOW) { terminal.println("Sensor 1 APAGADO "); }
  if (pir_2 == HIGH) { terminal.println("¡ Sensor 2 ENCENDIDO ! "); }
  if (pir_2 == LOW) { terminal.println("Sensor 2 APAGADO "); }
  if (pir_3 == HIGH) { terminal.println("¡ Sensor 3 ENCENDIDO ! "); }
  if (pir_3 == LOW) { terminal.println("Sensor 3 APAGADO "); }
  if (pir_4 == HIGH) { terminal.println("¡ Sensor 4 ENCENDIDO ! "); }
  if (pir_4 == LOW) { terminal.println("Sensor 4 APAGADO "); }
  terminal.flush();
}

void posicion_actual() {
  modem.getGPS(&gps_latitude, &gps_lonxitude, &gps_velocidade, &gps_altitude, &gps_vsat, &gps_usat, &gps_exactitude, &gps_ano, &gps_mes, &gps_dia, &gps_hora, &gps_minuto, &gps_segundo);
  terminal.print("Latitude: ");
  terminal.println(String(gps_latitude, 8));
  terminal.print("Lonxitude: ");
  terminal.println(String(gps_lonxitude, 8));
  terminal.print("Velocidade: ");
  terminal.println(String(gps_velocidade));
  terminal.flush();
}

void fija_posicion() {
  modem.getGPS(&gps_latitude, &gps_lonxitude, &gps_velocidade, &gps_altitude, &gps_vsat, &gps_usat, &gps_exactitude, &gps_ano, &gps_mes, &gps_dia, &gps_hora, &gps_minuto, &gps_segundo);
  fijapos_Latitude = gps_latitude;
  fijapos_Lonxitude = gps_lonxitude;
  fijapos_exactitude = gps_exactitude;
  fijapos_ano = gps_ano;
  fijapos_mes = gps_mes;
  fijapos_dia = gps_dia;
  fijapos_hora = gps_hora;
  fijapos_minuto = gps_minuto;
  fijapos_segundo = gps_segundo;
  terminal.println("¡ Posicion fijada !");
  terminal.print("lat: ");
  terminal.println(String(fijapos_Latitude, 8));
  terminal.print("Lon: ");
  terminal.println(String(fijapos_Lonxitude, 8));
  terminal.println("Ano: " + String(fijapos_ano) + "\tMes: " + String(fijapos_mes) + "\tDia: " + String(fijapos_dia));
  terminal.println("Hora: " + String(fijapos_hora) + "\tMinuto: " + String(fijapos_minuto) + "\tSegundo :" + String(fijapos_segundo));
  terminal.flush();
}

void sensorPIR() {
  unsigned long sensor = 10203040UL;
  int pir_1 = digitalRead(PIR1);  //sensores = 10203040 + ?
  int pir_2 = digitalRead(PIR2);
  int pir_3 = digitalRead(PIR3);
  int pir_4 = digitalRead(PIR4);
  if (pir_1 == HIGH) { sensor = sensor + 1000000; }
  if (pir_2 == HIGH) { sensor = sensor + 10000; }
  if (pir_3 == HIGH) { sensor = sensor + 100; }
  if (pir_4 == HIGH) { sensor = sensor + 1; }
  Blynk.virtualWrite(V3, sensor);
}

void rele() {
  unsigned long reles = 10203040UL;
  int REL_1 = digitalRead(REL1);  //reles = 10203040 + ?
  int REL_2 = digitalRead(REL2);
  int REL_3 = digitalRead(REL3);
  int REL_4 = digitalRead(REL4);
  if (REL_1 == HIGH) { reles = reles + 1000000; }
  if (REL_2 == HIGH) { reles = reles + 10000; }
  if (REL_3 == HIGH) { reles = reles + 100; }
  if (REL_4 == HIGH) { reles = reles + 1; }
  Blynk.virtualWrite(V4, reles);
}

void distancia() {
  modem.getGPS(&gps_latitude, &gps_lonxitude, &gps_velocidade, &gps_altitude, &gps_vsat, &gps_usat, &gps_exactitude, &gps_ano, &gps_mes, &gps_dia, &gps_hora, &gps_minuto, &gps_segundo);
  /*  Devolve a distancia en metros entre duas posicións, a latitude e lonxitude
   fixadas no fondeo ou atraque e a posicion actual. Debido a que a terra non e 
   perfectamente esferica podense dar erros de ata un 0.5%  */
  float lat1 = radians(fijapos_Latitude);
  float lat2 = radians(gps_latitude);
  float delta = radians(fijapos_Lonxitude - gps_lonxitude);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);  // sq -> eleva ao cadrado
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);  //sqrt -> Raiz cadrada
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  float Distancia = (delta * 6372795);
  Blynk.virtualWrite(V2, Distancia,0);

  /* Radio terrestre:
    Radio Ecuatorial 6,378.137 km
    Radio Polar 6,356.752 km
    Radio Medio 6,372.795 km
    -- (sin, cos, tan, asin, acos, atan) -- */
}

void voltimetro() {
  int16_t adc0;
  adc0 = ads.readADC_SingleEnded(0);
  volt_0 = ads.computeVolts(adc0);
  Blynk.virtualWrite(V0, volt_0);
}
