#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
//#include <MechaQMC5883.h>
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <Adafruit_HMC5883_U.h>
//#include <Adafruit_Sensor.h>
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

/*
//Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//O calculo do desvio magnético pode ser calculado pelos dados encontrados no site https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
#define mag_dev_g 19 //Desvio magnético calculado na cidade de blumenau de 19° 51' W -- Pelo mapa obtido no WMM, o desvio é negativo
#define mag_dev_m 51 //Se é W deve subtrair do heading
#define mag_dev_p 'W'
double desvio;


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//MechaQMC5883 bussola; //Criacao do objeto para o sensor 
TinyGPSPlus gps;

int x = 0, y = 0, z = 0, a = 0;
double angulo = 0;


double gms_graus(double graus, double minutos, char direcao){
  int sinal = 1;
  if(direcao == 'W'){
    sinal = -1;
  }else if(direcao == 'E'){
    sinal = 1;
  }
  return sinal * (graus + (minutos/60)); //após este passo, deve-se somar com o valor medido este obtido
}

void PreparaGPS(){

}

void PreparaMagn(){
compass.init();
compass.setSmoothing(10,true);  


}

void PreparaDisplay(){
   if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){ 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2000); // Pause for 2 seconds
  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(1);         // set text size
  display.setTextColor(WHITE);    // set text color
  display.setCursor(0, 0);       // set position to display
}


void displayInfo(){
  Serial.print(F("Location: "));

  if (gps.location.isValid()){
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print("Lng: ");
    Serial.print(gps.location.lng(), 6);
    Serial.println();
  }else{
    Serial.println(F("INVALID"));
  }
}


void calibration(){

compass.init();
int calibrationData[3][2];
bool changed = false;
bool done = false;
int t = 0;
int c = 0;
Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
Serial.println("Calibration will begin in 5 seconds.");
delay(5000);
while(done == false){
  int x, y, z;
  
  // Read compass values
  compass.read();

  // Return XYZ readings
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  changed = false;

  if(x < calibrationData[0][0]) {
    calibrationData[0][0] = x;
    changed = true;
  }
  if(x > calibrationData[0][1]) {
    calibrationData[0][1] = x;
    changed = true;
  }

  if(y < calibrationData[1][0]) {
    calibrationData[1][0] = y;
    changed = true;
  }
  if(y > calibrationData[1][1]) {
    calibrationData[1][1] = y;
    changed = true;
  }

  if(z < calibrationData[2][0]) {
    calibrationData[2][0] = z;
    changed = true;
  }
  if(z > calibrationData[2][1]) {
    calibrationData[2][1] = z;
    changed = true;
  }

  if (changed && !done) {
    Serial.println("CALIBRATING... Keep moving your sensor around.");
    c = millis();
  }
    t = millis();
  
  
  if((t - c > 5000) && !done) {
    done = true;
    Serial.println("DONE. Copy the line below and paste it into your projects sketch.);");
    Serial.println();
      
    Serial.print("compass.setCalibration(");
    Serial.print(calibrationData[0][0]);
    Serial.print(", ");
    Serial.print(calibrationData[0][1]);
    Serial.print(", ");
    Serial.print(calibrationData[1][0]);
    Serial.print(", ");
    Serial.print(calibrationData[1][1]);
    Serial.print(", ");
    Serial.print(calibrationData[2][0]);
    Serial.print(", ");
    Serial.print(calibrationData[2][1]);
    Serial.println(");");
    compass.setCalibration(calibrationData[0][0], calibrationData[0][1], calibrationData[1][0], calibrationData[1][1], calibrationData[2][0], calibrationData[2][1]);
    }
}
}


void setup() {
  desvio = gms_graus(mag_dev_g, mag_dev_m, mag_dev_p); //A medida real é o polo magnético menos o desvio
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial.println(desvio);
  Wire.begin();
  PreparaGPS();
  PreparaMagn();
  PreparaDisplay();
  bool calibrar = true;
  if (!calibrar){
    calibration();
  }else{
    PreparaMagn();
  }
}

void loop() {

//  while (Serial2.available() > 0)
//    if(gps.encode(Serial2.read()))
//      displayInfo(); //Mostrar as informações quando tiver
//  if (millis() > 5000 && gps.charsProcessed() < 10)
//  {
//    Serial.println(F("No GPS detected: check wiring."));
//    while (true);
//  }

 
// Display the results (magnetic vector values are in micro-Tesla (uT)) 
//Serial.print("X: ");
//Serial.print(event.magnetic.x);
//Serial.print(" ");
//Serial.print("Y: ");
//Serial.print(event.magnetic.y);
//Serial.print(" ");
//Serial.print("Z: ");
//Serial.print(event.magnetic.z);
//Serial.print(" ");
//Serial.println("uT");
  compass.read();
  byte a = compass.getAzimuth();

  char myArray[3];
  compass.getDirection(myArray, a);
  
  Serial.print(myArray[0]);
  Serial.print(myArray[1]);
  Serial.print(myArray[2]);
  Serial.println();
  
  // Return XYZ readings
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  //bussola.read(&x,&y,&z,&a); //Obter o valor dos eixos X, Y e Z do Sensor
  angulo = atan2(x, y)/0.0174532925; //Calculo do angulo usando os eixos X e Y atraves da formula = atan2(x, y)*180/pi
  angulo = angulo - desvio;

  
  //Ajuste do angulo entre 0 e 360 graus
  if(angulo < 0) {
  angulo+=360;}
  
  angulo = 360-angulo;
  Serial.println(angulo); //Imprime o valor do angulo na Serial do Arduino
  Serial.println("x = "+(String)x+" , y = "+(String)y+" , z = "+(String)z+" , a = "+(String)a);

  delay(1000); //Delay de 500 ms entre novas leituras

  // put your main code here, to run repeatedly:
}

*/






void calibrar_qmc5883(){

  Serial.println("A calibrção começa em 5 segundos, movimente o dispositivo");
  delay(5000);

  //int t_atual = millis(); //marca o instante de inicio agora
  int tempo_final = millis() + 5000; //marca o instante de final da calibração

  int max_x = -32768;
  int min_x = 32767;
  int max_y = -32768;
  int min_y = 32767;
  //int max_z = -32768;
  //int min_z = 32767;
  bool novo_v = false;
  bool pronto = false;
  int x, y; //, z;

  while(!pronto){
    compass.read();
    x = compass.getX();
    y = compass.getY();
    //z = compass.getZ();
    novo_v = false;
    if (x > max_x) max_x = x; novo_v = true;
    if (x < min_x) min_x = x; novo_v = true;
    if (y > max_y) max_y = y; novo_v = true;
    if (y < min_y) min_y = y; novo_v = true;
    //if (z > max_z) max_z = z; novo_v = true;
    //if (z < min_z) min_z = z; novo_v = true;
    if (novo_v && !pronto) tempo_final = millis() + 5000; Serial.println("Calibrando . . .");
    if(millis() > tempo_final) pronto = true; compass.setCalibration(min_x, max_x, min_y, max_y, -32768, 32767);
  }
}

int btn_1 = 32;
int btn_2 = 33;
int btn_3 = 34;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  pinMode(btn_1,INPUT_PULLUP);
  pinMode(btn_2,INPUT_PULLUP);
  pinMode(btn_3,INPUT_PULLUP);
}

void loop() {
  int x, y, z;
  //compass.read(&x, &y, &z);
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();


  // Calibrate the readings
  //int offset_x = 161; // Replace with your own calibration value
  //int offset_y = -1541; // Replace with your own calibration value
  //int offset_z = 0; // Replace with your own calibration value

  //x = x - offset_x;
  //y = y - offset_y;
  //z = z - offset_z;

  // Calculate the heading
  float heading = atan2(y, x);
  if (heading < 0) {
    heading = 2 * PI + heading;
  }
  heading = heading + radians(-19); // Magnetic declination of -19 degrees
  if (heading < 0) {
    heading = 2 * PI + heading;
  }
  float degrees = heading * 180 / PI;

  Serial.print("Heading: ");
  Serial.println(degrees);

  delay(100);
}