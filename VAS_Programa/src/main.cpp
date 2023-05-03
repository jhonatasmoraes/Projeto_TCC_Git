#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>
#include <telas_menu.h>
#include <math.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>


/*
  SPI Pins of ESP32 SD card as follows:
  SS    = 5;
  MOSI  = 23;
  MISO  = 19;
  SCK   = 18; 
*/

File myFile;

const int CS = 5; //Pino Chipselect
QMC5883LCompass compass;
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
Servo ESCA; //motor direito
Servo ESCB; //motor esquerdo
ESP32PWM pwm;
#define EEPROM_SIZE 8
TinyGPSPlus gps;




//auxiliares
float alvo = 50;
float apontar = 0;
bool partiu = false;
int tempoblink = 0;
bool estadoblink = false;


//pinos

int ESCA_Pin = 12;   //25
int ESCB_Pin = 13;  //26
int btn_1 = 14; //Botao SOBE14 laranja btn2
int btn_2 = 4; //Botao DESCE12 vermelho btn1 
int btn_3 = 15; //Botao entra13 amarelo btn3
int bomba1 = 33;
int bomba2 = 25;
int bomba3 = 26;
int bomba4 = 27;
int ds1820 = 32;

//Parametros Motores

int minUs = 1000; //tempo de pulso menor
int maxUs = 2000; //tempo de pulso maior
int ESCA_Vel = 90; //velocidade inicial
int ESCB_Vel = 90;

//Parametros controle
float Kp = 0.0;
float Ki = 0.0;
//float Kd = 0.0;
float erro = 0;


//Variaveis Globais
int selectedMenuItem = 0;
int numMenuItems = 3;
int NivelMenu = 0;
int SelecN1 = 0;
int SelecN2 = 0;

int16_t max_x = -32768;
int16_t min_x = 32767;
int16_t max_y = -32768;
int16_t min_y = 32767;
int16_t x, y; //, z;

const int NUM_ITENS = 9;

char menu_items[NUM_ITENS] [20] = {
  {"Cal. QMC5883"},
  {"GPS"},
  {"Direcao"},
  {"Iniciar"},
  {"Ler Dados"},
  {"Turbina 1"},
  {"Turbina 2"},
  {"Nave"},
  {"Info"}

};

int item_sel = 1; //Qual item do menu está selecionado
int item_ant; //Item anterior ao selecionado
int item_pos; //Item posterior ao selecionado

int btn_1_clic=0;
int btn_2_clic=0;
int btn_3_clic=0;
String saida = "";

int TelaAtual=0;

float direc = 0;

//#define PI 3.14159265359

const float declinacao = -19.85; //O ajuste deve ser negativo, assim o norte é apontado pela direção X

float Latitude = 0;
float Longitude = 0;

float dLat = 0;
float dLon = 0;
float lat1_rad = 0;
float lat2_rad = 0;
float yPoint = 0;
float xPoint = 0;
float bearing = 0;
float distancia = 0;
float raioterra = 6371000;



int coordenada = 0;
//int NUMCOORD = 6;
//float LatAlvo[] = {-26.809797, -26.810044, -26.810049, -26.810387, -26.810471, -26.810723, -26.810944};
//float LonAlvo[] = {-49.103512, -49.103228, -49.102701, -49.102745, -49.101955, -49.102366, -49.101547};
const int QtdPontos = 20;
float LatAlvo[QtdPontos]; //Prepara as arrays para receberem até 20 pontos
float LonAlvo[QtdPontos];
bool PontoLeitura[QtdPontos]; //Separa os pontos em que deve ser feita a leitura dos valores
bool PontoColeta[QtdPontos]; //Separa os pontos em que devem ser coletados a água 
int PontosLidos = 0;


float LatZero = 0;
float LonZero = 0;


//Variaveis PID

float setpoint = 0.0;  // Valor desejado
float kp = 0.5;        // Coeficiente proporcional
float ki = 0.2;        // Coeficiente integral
float kd = 0.1;        // Coeficiente derivativo
float sensor_value = 0.0;  // Valor do sensor
float output = 0.0;    // Saída do controlador
float error = 0.0;     // Erro atual
float last_error = 0.0; // Último erro
float proportional = 0.0;
float integral = 0.0;  // Soma do erro
float derivative = 0.0;  // Variação do erro
unsigned long last_time = 0;  // Último tempo de atualização
unsigned long dt = 10;  // Intervalo de tempo em milissegundos







//Lê as 20 coordenadas disponibilizadas no arquivo "coordenadas.txt" dentro do cartão SD e armazena as instruções nas variaveis LatAlvo, LonAlvo, PontoLeitura e PontoColeta
void LerCoordenadas(){
  myFile = SD.open("/coordenadas.txt", FILE_READ);
  if (myFile) {
    while (myFile.available()) {
      PontosLidos = 0;
      for (int i=0; i<QtdPontos; i++){
      String coordenadas = myFile.readStringUntil(';');
      sscanf(coordenadas.c_str(), "%f, %f, %d, %d", &LatAlvo[i], &LonAlvo[i], &PontoLeitura[i], &PontoColeta[i]); //Armazena em cada um dos elementos do array a coordenada alvo desejada
      if(LatAlvo[i] !=0 && LatAlvo[i]!=0){PontosLidos += 1;}
      Serial.print("Parametros[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.print(LatAlvo[i],10);
      Serial.print(" ");
      Serial.print(LonAlvo[i],10);
      Serial.print(" ");
      Serial.print(PontoLeitura[i]);
      Serial.print(" ");
      Serial.println(PontoColeta[i]);
      }
    }
    myFile.close();
  } else {
    Serial.println("Erro ao abrir o arquivo de coordenadas.");
  }
}

void CalcDirecDist(float lat1, float lon1, float lat2, float lon2){ //Recebe coordenadas em graus decimais
  dLat = (lat2 - lat1) * PI / 180; // Diferença de latitude em radianos
  dLon = (lon2 - lon1) * PI / 180; // Diferença de longitude em radianos
  lat1_rad = lat1 * PI / 180; // Latitude da primeira coordenada em radianos
  lat2_rad = lat2 * PI / 180; // Latitude da segunda coordenada em radianos
  yPoint = sin(dLon) * cos(lat2_rad);
  xPoint = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dLon);
  bearing = atan2(yPoint, xPoint) * 180 / PI - declinacao; // Cálculo do ângulo em graus
  if(bearing < 0){bearing += 360;}

  distancia = 2 * raioterra * asin(sqrt(pow(sin(dLat/2),2) + cos(lat1_rad)*cos(lat2_rad)*pow(sin(dLon/2),2)));

  //return bearing;
}

void CalculaPIDDirec(){
  //dt = millis() - last_time;//***atualiza dt
  if(millis() - last_time > dt){
  sensor_value = direc; //Leitura da direção atual
  error = bearing - sensor_value; //Compara com a direção desejada
  proportional = kp * error;
  integral += ki * error * dt;
  derivative = kd * (error - last_error) / dt;
  output = proportional + integral + derivative;
  if (output > 255) {
    output = 255;
  } else if (output < 0) {
    output = 0;
  }
  last_error = error;
  last_time = millis();

}
  //analogWrite(9, output);
  
  // Espera o intervalo de tempo especificado
  //while (millis() - last_time < dt) {
  //  delay(1);
  //}

}


void Navegar(){
  //Armazena as coordenadas de partida
 // if(!partiu){leGPS();  LatZero = Latitude;  LonZero = Longitude;  partiu = true;}

}

//------------------------------------------------Le GPS------------------------------------------------------------
void leGPS(){

  while (Serial2.available()){gps.encode(Serial2.read());}

  if(gps.location.isValid()){
    Latitude  = gps.location.lat(); // float
    Longitude = gps.location.lng(); // float
  }
}
//----------------------------------------------Calibra magnetometro------------------------------------------------
void calibrar_qmc5883(){
  ////////u8g2.clearDisplay();
  //u8g2.setCursor(0,0);
  //u8g2.println("Calibracao em 5s");
  //u8g2.println();
  //Serial.println("A calibrção começa em 5 segundos, movimente o dispositivo");
  compass.setCalibration(-32768, 32767, -32768, 32767, -32768, 32767);

  //delay(5000);

  //int t_atual = millis(); //marca o instante de inicio agora
  int tempo_final = millis() + 5000; //marca o instante de final da calibração

  max_x = -32768;
  min_x = 32767;
  max_y = -32768;
  min_y = 32767;
  //int max_z = -32768;
  //int min_z = 32767;
  bool novo_v = false;
  bool pronto = false;
  x = y = 0; //, z;

  while(!pronto){
    compass.read();
    x = compass.getX();
    y = compass.getY();
    //z = compass.getZ();
    novo_v = false;
    if (x > max_x){max_x = x; novo_v = true;}
    if (x < min_x){min_x = x; novo_v = true;}
    if (y > max_y){max_y = y; novo_v = true;}
    if (y < min_y){min_y = y; novo_v = true;}
    //if (z > max_z) max_z = z; novo_v = true;
    //if (z < min_z) min_z = z; novo_v = true;
    if (novo_v == true && pronto == false){ 
      tempo_final = millis() + 5000; 
      Serial.println("Calibrando . . ."); //u8g2.setCursor(0,9); u8g2.println("Calibrando...");  u8g2.println(); u8g2.display();
      novo_v = false;
      }

    if (millis() > tempo_final){
      pronto = true; 
      compass.setCalibration(min_x, max_x, min_y, max_y, -32768, 32767); 
      EEPROM.put(0, min_x >> 8);
      EEPROM.put(1, min_x & 0x00FF);
      EEPROM.put(2, max_x >> 8);
      EEPROM.put(3, max_x & 0x00FF);
      EEPROM.put(4, min_y >> 8);
      EEPROM.put(5, min_y & 0x00FF);
      EEPROM.put(6, max_y >> 8);
      EEPROM.put(7, max_y & 0x00FF);

      EEPROM.commit();
      Serial.println("Pronto!");//u8g2.setCursor(0,9); u8g2.println("Pronto!");  u8g2.println(); u8g2.display();
      }
  }
}
//-------------------------------------------Calcula graus----------------------------------------------------------
void CalculaGraus(){

  compass.read();
  x = compass.getX();
  y = compass.getY();
  direc = atan2(y,x) * 180 / 3.141592653;
  direc = direc - declinacao;
  if (direc < 0) {
    direc += 360;
  }
}
//-----------------------------------------------Controla ESCs------------------------------------------------------
void ControlaESC(){

  CalculaGraus();

  erro = direc - alvo;

  if(erro > 180){
    erro = 360 - direc + alvo;
  }
  erro = erro/2;

ESCA_Vel = 90 + erro;
ESCB_Vel = 90 - erro;

if(ESCA_Vel > 180){ESCA_Vel = 180;}else if(ESCA_Vel<0){ESCA_Vel = 0;}
if(ESCB_Vel > 180){ESCB_Vel = 180;}else if(ESCB_Vel<0){ESCB_Vel = 0;}

  ESCA.write(ESCA_Vel);
  ESCB.write(ESCB_Vel);
}
//------------------------------------------Mostra Tela-------------------------------------------------------------
void MostraTela(){
u8g2.setBitmapMode(1); //faz os bitmaps transparentes

switch (TelaAtual)
{
case 0:

  u8g2.firstPage();
  do{
    //Item Anterior  
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(26,15,menu_items[item_ant]);
    u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[item_ant]); //Desenha Icone Pos(0)
    //Item Selecionado
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(26,37,menu_items[item_sel]);
    u8g2.drawBitmap(4,24,16/8,16, Bitmap_Icons[item_sel]); //Desenha Icone GPS Pos(1)
    //Item Posterior
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(26,59,menu_items[item_pos]);
    u8g2.drawBitmap(4,46,16/8,16, Bitmap_Icons[item_pos]); //Desenha Iconeo Pos(2)

    u8g2.drawBitmap(0,22,128/8,21, epd_bitmap_BordaSelec); //Desenha selecionado
    //u8g2.drawBitmap(120,0,8/8,64, epd_bitmap_BarraRolagem); //Desenha Barra de rolagem
  }while(u8g2.nextPage());
  break;

case 1:
  u8g2.firstPage();
  do{
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(4,15,"A calibrar em");//cabe 12
    u8g2.drawStr(4,37,"5 segundos");//cabe 12
  }while(u8g2.nextPage());
  delay(5000);
  u8g2.firstPage();
  do{
    u8g2.drawStr(4,15,"Calibrando");//cabe 12
    //u8g2.drawStr(4,15,"5 segundos");//cabe 12
  }while(u8g2.nextPage());
  calibrar_qmc5883();
  u8g2.firstPage();
  do{
    u8g2.drawStr(4,15,"Calibrando");//cabe 12
    u8g2.drawStr(4,37,"Pronto!");//cabe 12
  }while(u8g2.nextPage());
  delay(2000);
  TelaAtual = 0; //volta para o menu principal
  break;

case 2: //Reservado GPS
    leGPS();
    CalcDirecDist(Latitude,Longitude, LatAlvo[coordenada], LonAlvo[coordenada]);
    u8g2.firstPage();
  do{
    u8g2.setFont(u8g2_font_squeezed_r6_tr);
    u8g2.drawStr(1,8,"Latitude / Longitude Atual");//cabe 12
    u8g2.setCursor(0, 16);
    u8g2.print(Latitude,6);
    u8g2.setCursor(52, 16);
    u8g2.print("/");
    u8g2.setCursor(60, 16);
    u8g2.print(Longitude,6);
    u8g2.drawStr(1,24,"Latitude / Longitude Alvo");//cabe 12
    u8g2.setCursor(0,32);
    u8g2.print(LatAlvo[coordenada],6);
    u8g2.setCursor(52, 32);
    u8g2.print("/");
    u8g2.setCursor(60, 32);
    u8g2.print(LonAlvo[coordenada],6);
    u8g2.drawStr(1,40,"Mag Alvo / Mag Atual");//cabe 12
    u8g2.setCursor(0,48);
    u8g2.print(bearing,3);
    u8g2.setCursor(52, 48);
    u8g2.print("/");
    u8g2.setCursor(60, 48);
    u8g2.print(direc,3);
    u8g2.setCursor(60, 56);
    u8g2.print(coordenada);


  }while(u8g2.nextPage());

  break;


case 3:
    u8g2.firstPage();
  do{
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(4,15,"Apontando para");//cabe 12
    
    u8g2.setCursor(4, 37);
    u8g2.print(direc);
  }while(u8g2.nextPage());
  break;


case 4:
    u8g2.firstPage();
  do{
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(4,15,"Escrevendo...");//cabe 12
    //WriteFile("/test.txt", "ElectronicWings.com");
    u8g2.setCursor(4, 37);
    u8g2.print("Pronto");
  }while(u8g2.nextPage());
  break;

case 5:
    u8g2.firstPage();
  do{
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(4,15,"Lendo...");//cabe 12
  }while(u8g2.nextPage());


  LerCoordenadas();
  delay(500);
  TelaAtual = 0;

      u8g2.firstPage();
  do{
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(4,15,"Coordenadas lidas");//cabe 12
    u8g2.setCursor(4, 37);
    u8g2.print(PontosLidos);
    u8g2.print(" pontos");
  }while(u8g2.nextPage());
  delay(2000);
  break;



case 6:
      u8g2.firstPage();
      do{
        u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[5]); //Desenha Icone GPS Pos(1)
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(26,15,"Vel Motor 1");
        u8g2.setCursor(50, 37);
        u8g2.print(ESCA_Vel);
        u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec); //Desenha selecionado
        //u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec); //Desenha selecionado
        u8g2.drawBox(4,45,map(ESCA_Vel, 0, 180, 0,114),16);
        //u8g2.drawBox(4,45,64,16);
      }while(u8g2.nextPage());
  break;
  
case 7:
      u8g2.firstPage();
      do{
        u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[6]); //Desenha Icone GPS Pos(1)
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(26,15,"Vel Motor 2");
        u8g2.setCursor(50, 37);
        u8g2.print(ESCB_Vel);
        u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec); //Desenha selecionado
        //u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec); //Desenha selecionado
        u8g2.drawBox(4,45,map(ESCB_Vel, 0, 180, 0,114),16);
        //u8g2.drawBox(4,45,64,16);
      }while(u8g2.nextPage());
  break;

case 8:
      u8g2.firstPage();
  do{
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(4,15,"Caminhando");//cabe 12
    u8g2.setCursor(4, 30);
    u8g2.print(alvo);
    u8g2.setCursor(48, 30);
    u8g2.print(direc);
    u8g2.setCursor(4, 45);
    u8g2.print(ESCA_Vel);
    u8g2.setCursor(35, 45);
    u8g2.print(ESCB_Vel);
  }while(u8g2.nextPage());
  //Escreve os valores nos ESCs
    ControlaESC();
  break;

case 9:
    u8g2.firstPage();
  do{
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(4,15,"A Desenvolver");//cabe 12

  }while(u8g2.nextPage());
  break;



  
default:
  TelaAtual = 0;
  break;
}
}
//----------------------------------------------Setup---------------------------------------------------------------
void setup(void) {


  //Inicializa diplay
  u8g2.begin();
  //Inicializa portas seriais (Serial0 = USB; Serial2 = GPS)
  Serial.begin(115200);
  Serial2.begin(9600);
  //Inicializa EEPROM e carrega os parâmetros de calibração do magnetômetro
  EEPROM.begin(EEPROM_SIZE);
  min_x=EEPROM.read(0)<<8|EEPROM.read(1);
  max_x=EEPROM.read(2)<<8|EEPROM.read(3);
  min_y=EEPROM.read(4)<<8|EEPROM.read(5);
  max_y=EEPROM.read(6)<<8|EEPROM.read(7);
  //Inicializa o magnetômetro e aplica os parâmetros de calibração carregados da EEPROM (Apenas valores de x e y são importantes)
  compass.init();
  compass.setCalibration(min_x, max_x, min_y, max_y, -32768, 32767);
  //Prepara os pinos de entrada para os botões
  pinMode(btn_1, INPUT_PULLUP);
  pinMode(btn_2, INPUT_PULLUP);
  pinMode(btn_3, INPUT_PULLUP);
  pinMode(bomba1, OUTPUT);
  pinMode(bomba2, OUTPUT);
  pinMode(bomba3, OUTPUT);
  pinMode(bomba4, OUTPUT);
  TelaAtual = 0;

  //Prepara os parâmtros dos ESCs conectados 
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
  ESCA.setPeriodHertz(50);      // Standard 50hz servo
	ESCB.setPeriodHertz(50);      // Standard 50hz servo
  // pwm.attachPin(4, 10000);//10khz
  ESCA.attach(ESCA_Pin, minUs, maxUs);
  ESCB.attach(ESCB_Pin, minUs, maxUs);
  delay(500);
  ESCA_Vel = 90; //velocidade inicial dos ESCs (ponto "zero", meio do curso)
  ESCB_Vel = 90;  
  ESCA.write(ESCA_Vel);
  ESCB.write(ESCB_Vel);
  //Inicializa cartão SD
  Serial.println("Initializing SD card...");
  if (!SD.begin(CS)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
}
//----------------------------------------------Loop----------------------------------------------------------------
void loop(void) {
//Comandos para botão 1 (desce)
  if((digitalRead(btn_1) == LOW)&&(btn_1_clic == 0)){
    if(TelaAtual == 0){
      item_sel = item_sel-1;
      if(item_sel<0){item_sel = NUM_ITENS-1;}
    }else if(TelaAtual == 6){
      ESCA_Vel-= 10;
      if(ESCA_Vel<0){ESCA_Vel = 0;}
      ESCA.write(ESCA_Vel);
    }else if(TelaAtual == 7){
      ESCB_Vel-= 10;
      if(ESCB_Vel<0){ESCB_Vel = 0;}
      ESCB.write(ESCB_Vel);
    }else if(TelaAtual == 2){
        coordenada = coordenada-1;
        if(coordenada<0){coordenada = PontosLidos;}
    }

    btn_1_clic = 1;
  }
//Comandos para botão 2 (sobe)
  if((digitalRead(btn_2) == LOW)&&(btn_2_clic == 0)){
    if(TelaAtual == 0){
      item_sel = item_sel+1;
      if(item_sel>= NUM_ITENS){item_sel = 0;}
    }else if(TelaAtual == 6){
      ESCA_Vel += 10;
      if(ESCA_Vel > 180){ESCA_Vel = 180;}
      ESCA.write(ESCA_Vel);
    }else if(TelaAtual == 7){
      ESCB_Vel += 10;
      if(ESCB_Vel > 180){ESCB_Vel = 180;}
      ESCB.write(ESCB_Vel);
    }else if(TelaAtual == 2){
        coordenada = coordenada+1;
        if(coordenada>= PontosLidos){coordenada = 0;}
    }

    btn_2_clic = 1;
  }
//Comandos para botão 3 (entra)
  if((digitalRead(btn_3) == LOW)&&(btn_3_clic == 0)){
    
    if(TelaAtual == 0){
    switch (item_sel)
    {
    case 0:
      TelaAtual = 1;
      break;

    case 1:
      TelaAtual = 2;
      break;

    case 2:
      TelaAtual = 3;
      break;

    case 3:
      TelaAtual = 4;
      break;

    case 4:
      TelaAtual = 5;
      break;

    case 5:
      TelaAtual = 6;
      break;

    case 6:
      TelaAtual = 7;
      break;
    case 7:
      TelaAtual = 8;
      break;
    case 8:
      TelaAtual = 9;
      break;
    
    default:
      TelaAtual = 0;
      break;
    }      }else{
    TelaAtual = 0;
  }
    btn_3_clic = 1;

  }

//Debounce dos botões
if((digitalRead(btn_1) == HIGH)&&(btn_1_clic == 1)){btn_1_clic = 0;}
if((digitalRead(btn_2) == HIGH)&&(btn_2_clic == 1)){btn_2_clic = 0;}
if((digitalRead(btn_3) == HIGH)&&(btn_3_clic == 1)){btn_3_clic = 0;}
//Corrige os limites da tela inicial
  item_ant = item_sel-1;
  if(item_ant<0){item_ant = NUM_ITENS-1;}
  item_pos = item_sel+1;
  if(item_pos >= NUM_ITENS){item_pos = 0;}
//



CalculaGraus();
MostraTela();

if (millis() > (tempoblink + 1000)){
  if(estadoblink == false){
    estadoblink = true;
  }else{
    estadoblink = false;
  }
  digitalWrite(bomba1, estadoblink);
  digitalWrite(bomba2, estadoblink);
  digitalWrite(bomba3, estadoblink);
  digitalWrite(bomba4, estadoblink);

  tempoblink = millis();
}

  //delay(1000);
}
