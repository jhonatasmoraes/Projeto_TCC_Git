#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <MechaQMC5883.h>
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

MechaQMC5883 bussola; //Criacao do objeto para o sensor 
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
    bussola.init();

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


void setup() {
  desvio = gms_graus(mag_dev_g, mag_dev_m, mag_dev_p); //A medida real é o polo magnético menos o desvio
  Serial.begin(115200);
  Serial2.begin(9600);
  PreparaGPS();
  PreparaMagn();
  PreparaDisplay();
}

void loop() {

  while (Serial2.available() > 0)
    if(gps.encode(Serial2.read()))
      //displayInfo(); //Mostrar as informações quando tiver
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }


  bussola.read(&x,&y,&z,&a); //Obter o valor dos eixos X, Y e Z do Sensor
  angulo = atan2(x, y)/0.0174532925; //Calculo do angulo usando os eixos X e Y atraves da formula = atan2(x, y)*180/pi
  angulo = angulo + desvio;

  
  //Ajuste do angulo entre 0 e 360 graus
  if(angulo < 0) 
  angulo+=360;
  
  angulo = 360-angulo;




  // put your main code here, to run repeatedly:
}