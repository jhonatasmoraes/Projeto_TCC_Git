#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>
#include <telas_menu.h> //Ícones e telas
#include <math.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>


/*PINOS ESP32
  Os pinos do ESP32 são denominados da seguinte forma:

    Pinos I2C
      -> GPIO 21 -- SDA
      -> GPIO 22 -- SCL
    Pinos UART
      -> GPIO 1  -- TXD0 (NC)
      -> GPIO 3  -- RXD0 (NC)
      -> GPIO 16 -- RXD2 (TX do GPS)
      -> GPIO 17 -- TXD2 (RX do GPS)
    Pinos SPI (Conectados ao leitor de cartão)
      -> GPIO 5  -- SS
      -> GPIO 18 -- SCK
      -> GPIO 19 -- MISO
      -> GPIO 23 -- MOSI
    Pinos conectados aos botões (Pull-up interno)
      -> GPIO 4  -- BTN1
      -> GPIO 14 -- BTN3 (Apresenta PWM durante boot)
      -> GPIO 15 -- BTN2 (Apresenta PWM durante boot)
    Pinos conectados as bombas
      -> GPIO 25 -- BMB3
      -> GPIO 26 -- BMB2
      -> GPIO 27 -- BMB1
      -> GPIO 33 -- BMB4
    Pino conectado ao termometro D18S20 (OneWire)
      -> GPIO 32 -- D18S20
    Pinos conectados aos ESCs
      -> GPIO 12 -- ESC1 ou ESCA  (precisa estar com resistor de pull-down se conectado ao servo)
      -> GPIO 13 -- ESC2 ou ESCB
    Pinos não conectados (abertos)
      -> GPIO 2  -- NC  (LED on board)
      -> GPIX 34 -- NC  
      -> GPIX 35 -- NC
      -> GPIX 36 -- NC (VP)
      -> GPIX 39 -- NC (VN)
    GPIOs não disponíveis
      -> GPIO 6  -- #N/D 
      -> GPIO 7  -- #N/D 
      -> GPIO 8  -- #N/D 
      -> GPIO 9  -- #N/D 
      -> GPIO 10 -- #N/D 
      -> GPIO 11 -- #N/D 
      -> GPIO 20 -- #N/D
      -> GPIO 24 -- #N/D
      -> GPIO 28 -- #N/D
      -> GPIO 29 -- #N/D
      -> GPIO 30 -- #N/D
      -> GPIO 31 -- #N/D
      -> GPIO 37 -- #N/D
*/

//Inicialização dos parâmetros das bibliotecas
// -> SD
File Arquivo; 
const int CS = 5;
// -> Magnetometro QMC5883
QMC5883LCompass compass;
#define EEPROM_SIZE 12  //Define o tamanho da EEPROM para armazenamento dos parametros de calibração do magnetômetro (0-7), ajustes PID(8-10) e percentual Vmax (11)
// ->Display OLED
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE); //U8G2_R2 faz o display girar 180graus
// -> Parâmetros de servo para os ESCs
Servo ESCA; //motor direito
Servo ESCB; //motor esquerdo
ESP32PWM pwm;
// -> GPS
TinyGPSPlus gps;
// ->Termohigrometro BME280
Adafruit_BME280 bme;

// ->Parâmetros para termômetro D18S20
const int oneWireBus = 32; 
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
//


const float declinacao = -19.85; //O ajuste deve ser negativo, assim o norte é apontado pela direção X

//auxiliares
float alvo = 50;
float apontar = 0;
bool partiu = false;
int tempoblink = 0;
bool estadoblink = false;
bool StartScreen = true;


//Pinos GPIO

const int ESCA_Pin = 12;
const int ESCB_Pin = 13;
const int btn_1 = 15; //Botao anterior ou desce laranja btn2 se possível, trocar no plug
const int btn_2 = 4; //Botao seguinte ou sobe vermelho btn1 
const int btn_3 = 14; //Botao entra13 amarelo btn3
const int bomba1 = 33;
const int bomba2 = 25;
const int bomba3 = 26;
const int bomba4 = 27;

//Parametros Motores

int minUs = 1000; //tempo de pulso menor
int maxUs = 2000; //tempo de pulso maior
int ESCA_Vel = 90; //velocidade inicial
int ESCB_Vel = 90;
//O angulo do servo se dá de 0 a 180, onde 90 é o motor parado.

//Parametros PID controle de direção
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
float erro = 0;

//***Verificar necessidade e apagar
//Outras variaveis
//int selectedMenuItem = 0;
//int numMenuItems = 3;
//int NivelMenu = 0;
//int SelecN1 = 0;
//int SelecN2 = 0;

//Parâmetros para magnetômetro

int16_t max_x = -32768;
int16_t min_x = 32767;
int16_t max_y = -32768;
int16_t min_y = 32767;
int16_t x, y; //, z;

const int NUM_ITENS = 13; //Número de itens do menu
char menu_items[NUM_ITENS] [20] = { //Itens disponíveis
  {"Cal. QMC5883"},
  {"GPS"},
  {"Direcao"},
  {"Iniciar"},
  {"Ler Dados"},
  {"Turbina 1"},
  {"Turbina 2"},
  {"Nave"},
  {"Info"},
  {"Ajuste Vel."},
  {"Ajuste P"},
  {"Ajuste I"},
  {"Ajuste D"}

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
const int QtdPontos = 20; //Capacidade determinada de até 20 pontos
float LatAlvo[QtdPontos+1]; //Prepara as arrays para receberem até 20 pontos +1 (origem)
float LonAlvo[QtdPontos+1];
bool PontoLeitura[QtdPontos]; //Separa os pontos em que deve ser feita a leitura dos valores
bool PontoColeta[QtdPontos]; //Separa os pontos em que devem ser coletados a água 
int PontosLidos = 0;

int pontoteste = 0;


float LatZero = 0;
float LonZero = 0;


//Variaveis PID

float setpoint = 0.0;  // Valor desejado
float kp = 0.5;        // Coeficiente proporcional
float ki = 0.0;        // Coeficiente integral
float kd = 0.0;        // Coeficiente derivativo
float output = 0.0;    // Saída do controlador
float error = 0.0;     // Erro atual
float last_error = 0.0; // Último erro
float proportional = 0.0;
float integral = 0.0;  // Soma do erro
float derivative = 0.0;  // Variação do erro
unsigned long last_time = 0;  // Último tempo de atualização
unsigned long dt = 10;  // Intervalo de tempo em milissegundos


//Variaveis BME
float bme_temperatura = 0;
float bme_umidade = 0;
float bme_pressao = 0;

//Variáveis bombas
bool bomba1_livre = true;
bool bomba2_livre = true;
bool bomba3_livre = true;
bool bomba4_livre = true;
int tempo_enchimento = 5000; //Tempo que a bomba fica ligada (ms)


//Variaveis de deslocamento
int velocidade_atual = 0; //-100 a 100, usar até 50


//bearing = angulo que deve ser apontado
//direc = angulo atual
//


int erro2 = 0;  

int8_t percentualP = 100;
int8_t percentualI = 100;
int8_t percentualD = 100;
int8_t percentualV = 50;
int8_t Vmax = 80;



bool percursoConcluido = false;

//--------------------------------------------Le coordenadas--------------------------------------------------------OK
//Lê até 20 coordenadas disponibilizadas no arquivo "coordenadas.txt" dentro do cartão SD e armazena as instruções nas variaveis LatAlvo, LonAlvo, PontoLeitura e PontoColeta
bool LerCoordenadas(){
  Arquivo = SD.open("/coordenadas.txt", FILE_READ);
  if (Arquivo) {
    while (Arquivo.available()) {
      PontosLidos = 0;
      for (int i=0; i<QtdPontos; i++){
      String coordenadas = Arquivo.readStringUntil(';');
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
    Arquivo.close();
    return true;
  } else {
    Serial.println("Erro ao abrir o arquivo de coordenadas.");
    PontosLidos = 0;  
    return false;
  }
}
//---------------Calcula o angulo que deve ser apontado e a distancia do alvo--------------------OK
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
//-------------------------------Algoritmo PID e controle dos motores-----------------------------------------------OK
void AtuaESCs(int ajuste){ //Envia velocidade e ajuste de direção aos motores (velocidade_atual)
//ajuste vai de 0 a 255
//-100 é gira para UM LADO =, 100 é gira PARA O OUTRO
//0 não faz correção

  ESCA_Vel = map(velocidade_atual,-100,100,0,180) + map(ajuste,-100,100,0,180);
  ESCB_Vel = map(velocidade_atual,-100,100,0,180) - map(ajuste,-100,100,0,180);

  if(ESCA_Vel > 180){ESCA_Vel = 180;}else if(ESCA_Vel<0){ESCA_Vel = 0;}
  if(ESCB_Vel > 180){ESCB_Vel = 180;}else if(ESCB_Vel<0){ESCB_Vel = 0;}

  ESCA.write(ESCA_Vel);
  ESCB.write(ESCB_Vel);

}
void CalculaPIDDirec(){ //Algoritmo genérico de controle PID com tempo de intervalo de correção "dt" 
  //O objetivo do erro é ser 0, então os valores precisam ir de -255 a 255

  if(millis() - last_time > dt){
    error = bearing - direc; //Compara a direção apontada com a direção desejada
    proportional = percentualP/100 * kp * error;
    integral += percentualI/100 * ki * error * dt;
    derivative = percentualD/100 * kd * (error - last_error) / dt;
    output = proportional + integral + derivative;
    if(output > 100){output = 100;}else if(output < -100){output = -100;} // Limita a saída do PID de -100 a 100 (%)
    last_error = error;
    last_time = millis();
    AtuaESCs(output);
}

}
void parar_motores(){ //Função auxiliar para parar os motores
  ESCA.write(90);
  ESCB.write(90);
  velocidade_atual = 0;
}

//------------------------------------------------Le GPS------------------------------------------------------------OK
void leGPS(){

  while (Serial2.available()){gps.encode(Serial2.read());} //Lê os dados da porta serial

  if(gps.location.isValid()){
    Latitude  = gps.location.lat(); // float ->aponta os dados atuais de latitude e longitude
    Longitude = gps.location.lng(); // float 
  }
}
//----------------------------------------------Calibra magnetometro------------------------------------------------OK
void calibrar_qmc5883(){

  compass.setCalibration(-32768, 32767, -32768, 32767, -32768, 32767);

  int tempo_final = millis() + 5000; //marca o instante de final da calibração

  max_x = -32768;
  min_x = 32767;
  max_y = -32768;
  min_y = 32767;
  //int max_z = -32768; //Não é calibrado z, pois não é usado
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
      EEPROM.put(0, min_x >> 8);  //Armazena valores de calibração no EEPROM
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
//-------------------------------------------Calcula graus----------------------------------------------------------OK
void CalculaGraus(){ //Calcula a quantos graus está do norte geográfico (direção que está apontando)
  compass.read();
  x = compass.getX();
  y = compass.getY();
  direc = atan2(y,x) * 180 / 3.141592653;
  direc = direc - declinacao;
  if (direc < 0) {
    direc += 360;
  }
}
//-----------------------------------------------Controla ESCs------------------------------------------------------Retirar, não vai fazer sentido já que foi implementado o PID
void ControlaESC(){
  CalculaGraus();
  erro2 = direc - alvo;
  if(erro2 > 180){
    erro2 = 360 - direc + alvo;
  }
  erro2 = erro2/2;

ESCA_Vel = 90 + erro2;
ESCB_Vel = 90 - erro2;

if(ESCA_Vel > 180){ESCA_Vel = 180;}else if(ESCA_Vel<0){ESCA_Vel = 0;}
if(ESCB_Vel > 180){ESCB_Vel = 180;}else if(ESCB_Vel<0){ESCB_Vel = 0;}

  ESCA.write(ESCA_Vel);
  ESCB.write(ESCB_Vel);
}
//------------------------------------------Fazer atividade no ponto-------------------------------------------------OK
void AtividadePonto(int ponto){
  
  String texto_bomba;
  String texto_bme;
  String texto_D18S20;
  String texto_GPS;
  
  if (PontoLeitura[ponto]){
    bme_temperatura = bme.readTemperature();
    bme_umidade = bme.readHumidity();
    bme_pressao = bme.readPressure() / 100.0F;
    sensors.requestTemperatures(); 
    texto_bme = "Temperatura = " + String(bme_temperatura) + " ºC, " + "Umidade = " + String(bme_umidade) + " %, " + "Pressão = " + String(bme_pressao) + " hPa";
    texto_D18S20 = "Temperatura da água = " + String(sensors.getTempCByIndex(0)) + " ºC";

  }else{
    bme_temperatura = 0;
    bme_umidade = 0;
    bme_pressao = 0;
    texto_bme = "Dados não lidos";
    texto_D18S20 = "Dados não lidos";
  }
  

  if(PontoColeta[ponto]){
    if(bomba1_livre){//Bomba 1 livre
      digitalWrite(bomba1, HIGH);
      delay(tempo_enchimento);
      digitalWrite(bomba1, LOW);
      bomba1_livre = false;
      texto_bomba = "Reservatório 1 enchido";
    }else if(bomba2_livre){//Bomba 2 livre
      digitalWrite(bomba2, HIGH);
      delay(tempo_enchimento);
      digitalWrite(bomba2, LOW);
      bomba2_livre = false;
      texto_bomba = "Reservatório 2 enchido";
      }else if(bomba3_livre){//Bomba 3 livre
      digitalWrite(bomba3, HIGH);
      delay(tempo_enchimento);
      digitalWrite(bomba3, LOW);
      bomba3_livre = false;
      texto_bomba = "Reservatório 3 enchido";
    }else if(bomba4_livre){//Bomba 4 livre
      digitalWrite(bomba4, HIGH);
      delay(tempo_enchimento);
      digitalWrite(bomba4, LOW);
      bomba4_livre = false;
      texto_bomba = "Reservatório 4 enchido";
    }else{
      texto_bomba = "Todos os reservatórios cheios, nenhum enchido";
  }}else{
    texto_bomba = "Amostra não coletada";
  }
  leGPS();
  texto_GPS = "Latitude, Longitude: "  + String(Latitude) + ", " + String(Longitude);
  Arquivo = SD.open("/resultados.txt", FILE_APPEND,true);
//Se abriu, acrescenta no arquivo
  if(Arquivo) {
    //Serial.print("Writing to test.txt...");
    Arquivo.println("Ponto de controle");
    Arquivo.println(texto_GPS);
    Arquivo.println(texto_bme);
    Arquivo.println(texto_D18S20);
    Arquivo.println(texto_bomba);
    Arquivo.println("-----------------------------------------------------------------");
    Arquivo.close();


        u8g2.firstPage();
    do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(1,15,"Dados armazenados");
        u8g2.drawStr(29,30," no cartao");
        
      }while(u8g2.nextPage());
      delay(2000);

  }else{
    // if the file didn't open, print an error:
    u8g2.firstPage();
    do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(4,15,"Erro ao gravar dados");
        u8g2.drawStr(4,30,"Verifique o cartao");
        
      }while(u8g2.nextPage());
      delay(2000);
  }


}
//------------------------------------------Função Navegar-------------------------------------------------
void Navegar(){
  delay(1000);
  percursoConcluido = true;


}

//------------------------------------------Mostra Tela-------------------------------------------------------------
void MostraTela(){
  u8g2.setBitmapMode(1); //faz os bitmaps transparentes
  switch (TelaAtual){
    case 0: //Tela inicial (menu) OK
      u8g2.firstPage();
      do{
        //Item Anterior  
        u8g2.setFont(u8g2_font_7x14_tf);
        u8g2.drawStr(26,15,menu_items[item_ant]);
        u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[item_ant]); //Desenha Icone (0)
        //Item Selecionado
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(26,37,menu_items[item_sel]);
        u8g2.drawBitmap(4,24,16/8,16, Bitmap_Icons[item_sel]); //Desenha Icone (1)
        //Item Posterior
        u8g2.setFont(u8g2_font_7x14_tf);
        u8g2.drawStr(26,59,menu_items[item_pos]);
        u8g2.drawBitmap(4,46,16/8,16, Bitmap_Icons[item_pos]); //Desenha Icone (2)

        u8g2.drawBitmap(0,22,128/8,21, epd_bitmap_BordaSelec); //Desenha selecionado
        //u8g2.drawBitmap(120,0,8/8,64, epd_bitmap_BarraRolagem); //Desenha Barra de rolagem (Não implementado)
      }while(u8g2.nextPage());
      break;

    case 1: //Calibra Magnetometro OK
      u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14_tf);
        u8g2.drawStr(4,15,"A calibrar em");
        u8g2.drawStr(4,37,"5 segundos");
      }while(u8g2.nextPage());
      delay(5000);
      u8g2.firstPage();
      do{
        u8g2.drawStr(4,15,"Calibrando");
      }while(u8g2.nextPage());
      calibrar_qmc5883();//Chama a função para calibrar
      u8g2.firstPage();
      do{
        u8g2.drawStr(4,15,"Calibrando");
        u8g2.drawStr(4,37,"Pronto!"); 
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
    if(!percursoConcluido){
        u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(4,15,"Navegando...");//cabe 12
      }while(u8g2.nextPage());
      Navegar();
      }else{
        u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(4,15,"Percurso Finalizado!");//cabe 12
      }while(u8g2.nextPage());
  }
      break;

    case 5: //Tela ler coordenadas do cartão SD 
        u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(4,15,"Lendo...");//cabe 12
      }while(u8g2.nextPage());

      delay(500);

      if(LerCoordenadas()){
          u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(4,15,"Coordenadas lidas");//cabe 12
        u8g2.setCursor(4, 37);
        u8g2.print(PontosLidos);
        u8g2.print(" pontos");
      }while(u8g2.nextPage());
      }else{
          u8g2.firstPage();
        do{
          u8g2.setFont(u8g2_font_7x14B_tf);
          u8g2.drawStr(1,15,"Falha na leitura");//cabe 12
          u8g2.drawStr(1,30,"Verifique o cartao");//cabe 12
        }while(u8g2.nextPage());

      }
      TelaAtual = 0;
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

    case 9://Tela de informações
        u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(1,15,"Veiculo Autonomo");
        u8g2.drawStr(1,30,"de Superficie");
        u8g2.setFont(u8g2_font_7x14_tf);
        u8g2.drawStr(1,45,"Trab Conc Curso");
        u8g2.drawStr(1,60,"Jhonatas WG Moraes");
      }while(u8g2.nextPage());
      break;
      
    case 10: //Ajuste V
          u8g2.firstPage();
          do{
            u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[9]);
            u8g2.setFont(u8g2_font_7x14B_tf);
            u8g2.drawStr(26,15,"Ajuste Vel.");
            u8g2.setCursor(50, 37);
            u8g2.print(percentualV);
            u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec); //Desenha selecionado
            u8g2.drawBox(4,45,map(percentualV, 0, 100, 0,114),16);
          }while(u8g2.nextPage());
      break;

    case 11: //Ajuste P
          u8g2.firstPage();
          do{
            u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[10]);
            u8g2.setFont(u8g2_font_7x14B_tf);
            u8g2.drawStr(26,15,"Ajuste P");
            u8g2.setCursor(50, 37);
            u8g2.print(percentualP);
            u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec); //Desenha selecionado
            u8g2.drawBox(4,45,map(percentualP, 0, 100, 0,114),16);
          }while(u8g2.nextPage());
      break;

      case 12: //Ajuste I
          u8g2.firstPage();
          do{
            u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[11]);
            u8g2.setFont(u8g2_font_7x14B_tf);
            u8g2.drawStr(26,15,"Ajuste I");
            u8g2.setCursor(50, 37);
            u8g2.print(percentualI);
            u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec); //Desenha selecionado
            u8g2.drawBox(4,45,map(percentualI, 0, 100, 0,114),16);
          }while(u8g2.nextPage());
      break;

      case 13: //Ajuste D
          u8g2.firstPage();
          do{
            u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[12]);
            u8g2.setFont(u8g2_font_7x14B_tf);
            u8g2.drawStr(26,15,"Ajuste D");
            u8g2.setCursor(50, 37);
            u8g2.print(percentualD);
            u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec); //Desenha selecionado
            u8g2.drawBox(4,45,map(percentualD, 0, 100, 0,114),16);
          }while(u8g2.nextPage());
      break;

      
    default://Caso alguma tela ainda não implementada
      TelaAtual = 0;
      break;
  }
}
//----------------------------------------------Setup---------------------------------------------------------------OK -> Implementar PID na EEPROM
void setup(void) {


  //Inicializa diplay, bme e cartão SD
  u8g2.begin();
  bme.begin(0x76);  
  SD.begin(10);
  //Inicializa portas seriais (Serial0 = USB; Serial2 = GPS)
  Serial.begin(115200);
  Serial2.begin(9600);
  //Inicializa EEPROM e carrega os parâmetros de calibração do magnetômetro
  EEPROM.begin(EEPROM_SIZE);
  min_x=EEPROM.read(0)<<8|EEPROM.read(1);
  max_x=EEPROM.read(2)<<8|EEPROM.read(3);
  min_y=EEPROM.read(4)<<8|EEPROM.read(5);
  max_y=EEPROM.read(6)<<8|EEPROM.read(7);
  percentualP = EEPROM.read(8); //Carrega valores P
  percentualI = EEPROM.read(9);  //Carrega valores I
  percentualD = EEPROM.read(10); //Carrega valores D
  percentualV = EEPROM.read(11); //Carrega valores Velocidade
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
  ESCA.attach(ESCA_Pin, minUs, maxUs);
  ESCB.attach(ESCB_Pin, minUs, maxUs);

  delay(1000);
  ESCA_Vel = 90; //velocidade inicial dos ESCs (ponto "zero", meio do curso)
  ESCB_Vel = 90;  
  ESCA.write(ESCA_Vel);
  ESCB.write(ESCB_Vel);

  //Inicializa cartão SD
  Serial.println("Inicializando cartão SD...");
  if (!SD.begin(CS)) {
    Serial.println("Falha na inicialização! Verificar os cabos e o cartão.");
    return;
  }

  Serial.println("Inicializado.");


}
//----------------------------------------------Loop----------------------------------------------------------------
void loop(void) {
//Splash Screen
  if(StartScreen){
    u8g2.firstPage();
    do{
      u8g2.drawBitmap(0,0,128/8,64, epd_bitmap_StartScreen); //Desenha selecionado
    }while(u8g2.nextPage());
    delay(2000);
    StartScreen = false;
  }


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
    }else if(TelaAtual == 10){//Ajusta V
      percentualV-= 1;
      if(percentualV<0){percentualV = 0;}
    }else if(TelaAtual == 11){//Ajusta P
      percentualP-= 1;
      if(percentualP<0){percentualP = 0;}
    }else if(TelaAtual == 12){//Ajusta P
      percentualI-= 1;
      if(percentualI<0){percentualI = 0;}
    }else if(TelaAtual == 13){//Ajusta P
      percentualD-= 1;
      if(percentualD<0){percentualD = 0;}
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
    }else if(TelaAtual == 10){//Ajusta Vmax
      percentualV += 1;
      if(percentualV > Vmax){percentualV = Vmax;}
    }else if(TelaAtual == 11){//Ajusta P
      percentualP += 1;
      if(percentualP > 100){percentualP = 100;}
    }else if(TelaAtual == 12){//Ajusta I
      percentualI += 1;
      if(percentualI > 100){percentualI = 100;}
    }else if(TelaAtual == 13){//Ajusta D
      percentualD += 1;
      if(percentualD > 100){percentualD = 100;}
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

      case 9: 
      TelaAtual = 10;
      break;

      case 10:  //Ajuste P
      TelaAtual = 11;
      break;

      case 11:  //Ajuste I
      TelaAtual = 12;
      break;

      case 12: //Ajuste D 
      TelaAtual = 13;
      break;
    
    default:
      TelaAtual = 0;
      break;
    }  
    }else if(TelaAtual == 4){
      parar_motores();
      u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(1,15,"Motores parados");
      }while(u8g2.nextPage());
      delay(2000);
      TelaAtual = 0;
    }else if(TelaAtual == 10){
      EEPROM.put(11, percentualV);
      EEPROM.commit();
      u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(1,15,"Ajuste Vmax armazenado");
      }while(u8g2.nextPage());
      delay(2000);
      TelaAtual = 0;
    }else if(TelaAtual == 11){
      EEPROM.put(8, percentualP);
      EEPROM.commit();
      u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(1,15,"Ajuste P armazenado");
      }while(u8g2.nextPage());
      delay(2000);
      TelaAtual = 0;
    }else if(TelaAtual == 12){
      EEPROM.put(9, percentualI);
      EEPROM.commit();
      u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(1,15,"Ajuste I armazenado");
      }while(u8g2.nextPage());
      delay(2000);
      TelaAtual = 0;
    }else if(TelaAtual == 13){
      EEPROM.put(10, percentualD);
      EEPROM.commit();
      u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(1,15,"Ajuste D armazenado");
      }while(u8g2.nextPage());
      delay(2000);
      TelaAtual = 0;
    }else{
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



CalculaGraus(); //calcula a direção em cada ciclo
MostraTela(); //usa a função para mostrar dados no display, e as funções do display para executar as operações

}
