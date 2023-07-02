/*Codigo principal
Jhonatas Willian Gonçalves de Moraes
Trabalho de Conclusao de Curso
Universidade Tecnologica Federal do Parana
Campus Apucarana
Engenharia Eletrica*/
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>
#include <telas_menu.h> //icones e telas
#include <math.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*PINOS ESP32
  Os pinos do ESP32 sao denominados da seguinte forma:

    Pinos I2C
      -> GPIO 21 -- SDA
      -> GPIO 22 -- SCL
    Pinos UART
      -> GPIO 1  -- TXD0 (NC)
      -> GPIO 3  -- RXD0 (NC)
      -> GPIO 16 -- RXD2 (TX do GPS)
      -> GPIO 17 -- TXD2 (RX do GPS)
    Pinos SPI (Conectados ao leitor de cartao)
      -> GPIO 5  -- SS
      -> GPIO 18 -- SCK
      -> GPIO 19 -- MISO
      -> GPIO 23 -- MOSI
    Pinos conectados aos botoes (Pull-up interno)
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
    Pinos nao conectados (abertos)
      -> GPIO 2  -- NC  (LED on board)
      -> GPIX 34 -- NC  
      -> GPIX 35 -- NC
      -> GPIX 36 -- NC (VP)
      -> GPIX 39 -- NC (VN)
    GPIOs nao disponiveis
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

//Inicializacao dos parametros das bibliotecas
// -> SD
File Arquivo; 
const int CS = 5;
// -> Magnetometro QMC5883
QMC5883LCompass compass;
#define EEPROM_SIZE 16  //Define o tamanho da EEPROM para armazenamento dos parametros de calibracao do magnetometro (0-7), ajustes PID(8-10) e percentual Vmax (11)
// ->Display OLED
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE); //U8G2_R2 faz o display girar 180graus
// -> Parametros de servo para os ESCs
Servo ESCA; //motor direito
Servo ESCB; //motor esquerdo
ESP32PWM pwm;
// -> GPS
TinyGPSPlus gps;
// ->Termohigrometro BME280
Adafruit_BME280 bme;

// ->Parametros para termometro D18S20
const int oneWireBus = 32; 
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
//

const float declinacao = -19.85; //O ajuste deve ser negativo, assim o norte e apontado pela direcao X

//auxiliares
float alvo = 50;
bool StartScreen = true;


//Pinos GPIO
const int ESCA_Pin = 12;
const int ESCB_Pin = 13;
const int btn_1 = 15; //Botao anterior ou desce laranja btn2 se possivel, trocar no plug
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
//O angulo do servo se da de 0 a 180, onde 90 e o motor parado.

//Parametros para magnetometro
int16_t max_x = -32768;
int16_t min_x = 32767;
int16_t max_y = -32768;
int16_t min_y = 32767;
int16_t x, y; //, z;

const int NUM_ITENS = 13; //Numero de itens do menu
char menu_items[NUM_ITENS] [20] = { //Itens disponiveis
  {"Cal. QMC5883"},
  {"GPS"},
  {"Direcao"},
  {"Iniciar"},
  {"Ler Dados"},
  {"Turbina 1"},
  {"Turbina 2"},
  {"Teste"},
  {"Info"},
  {"Ajuste Vel."},
  {"Ajuste P"},
  {"Ajuste I"},
  {"Ajuste D"}
};

int item_sel = 1; //Qual item do menu esta selecionado
int item_ant; //Item anterior ao selecionado
int item_pos; //Item posterior ao selecionado

int btn_1_clic=0;
int btn_2_clic=0;
int btn_3_clic=0;

int TelaAtual=0;
float direc = 0;
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
const int QtdPontos = 20; //Capacidade determinada de ate 20 pontos
float LatAlvo[QtdPontos+1]; //Prepara as arrays para receberem ate 20 pontos +1 (origem)
float LonAlvo[QtdPontos+1];
bool PontoLeitura[QtdPontos]; //Separa os pontos em que deve ser feita a leitura dos valores
bool PontoColeta[QtdPontos]; //Separa os pontos em que devem ser coletados a agua 
int PontosLidos = 0;

//Variaveis PID

float kp = 1;        // Coeficiente proporcional
float ki = 0.005;        // Coeficiente integral
float kd = 1;        // Coeficiente derivativo
float saida = 0.0;    // Saida do controlador
float erro = 0.0;     // Erro atual
float ultimo_erro = 0.0; // ultimo erro
float proporcional = 0.0;
float integral = 0.0;  // Soma do erro
float derivativo = 0.0;  // Variacao do erro
unsigned long last_time = 0;  // ultimo tempo de atualizacao
unsigned long dt = 50;  // Intervalo de tempo em milissegundos
//unsigned long dt2 = 1;  // Intervalo de tempo em milissegundos

//Variaveis BME
float bme_temperatura = 0;
float bme_umidade = 0;
float bme_pressao = 0;

//Variaveis bombas
bool bomba1_livre = true;
bool bomba2_livre = true;
bool bomba3_livre = true;
bool bomba4_livre = true;
int tempo_enchimento = 14000; //Tempo que a bomba fica ligada (ms)

//Variaveis de deslocamento
float velocidade_atual = 0; //-100 a 100, usar ate 50

//bearing = angulo que deve ser apontado
//direc = angulo atual

float erro2 = 0;  

int8_t percentualP = 100;
int8_t percentualI = 100;
int8_t percentualD = 100;
int8_t percentualV = 50;
int8_t Vmax = 80;

bool percursoConcluido = false;
bool gpsvalid = false;
int satconectados = 0;
float precisao = 3;//metros
int p = 0; //auxiliar de contagem

uint32_t horaagora = 0; //Hora que e apontada pelo GPS
uint32_t datahoje = 0; //Data do GPS
String TextoLog; //Texto para gravacao do log
bool aux; //variavel auxiliar
unsigned long last_time_disp = 0; //variavel auxiliar para gravacao no display
unsigned long last_time_cartao = 0; //variavel auxiliar para gravacao no SD


//----------------------------------------------LOG de dados no SO--------------------------------------------------
void LogSD(){
  if(millis() - last_time_cartao > 500){
    TextoLog = String(horaagora) + "; " + String(Latitude, 7) + "; " + String(Longitude, 7) + "; " + String(LatAlvo[p], 7) + "; " + String(LonAlvo[p], 7) + "; " + String(direc) + "; " + String(bearing) + "; " + String(erro) + "; " + String(ESCA_Vel) + "; " + String(ESCB_Vel) + ";";
    Arquivo = SD.open("/log.txt", FILE_APPEND,true);
    if(Arquivo) {
      Arquivo.println(TextoLog);
       Arquivo.close();
      }
    last_time_cartao = millis();
    }
}
//--------------------------------------------Le coordenadas--------------------------------------------------------
//Le ate 20 coordenadas disponibilizadas no arquivo "coordenadas.txt" dentro do cartao SD e armazena as instrucoes nas variaveis LatAlvo, LonAlvo, PontoLeitura e PontoColeta
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
  }else{
    Serial.println("Erro ao abrir o arquivo de coordenadas.");
    PontosLidos = 0;  
    return false;
  }

}
//---------------Calcula o angulo que deve ser apontado e a distancia do alvo--------------------OK
void CalcDirecDist(float lat1, float lon1, float lat2, float lon2){ //Recebe coordenadas em graus decimais
  dLat = (lat2 - lat1) * PI / 180; // Diferenca de latitude em radianos
  dLon = (lon2 - lon1) * PI / 180; // Diferenca de longitude em radianos
  lat1_rad = lat1 * PI / 180; // Latitude da primeira coordenada em radianos
  lat2_rad = lat2 * PI / 180; // Latitude da segunda coordenada em radianos
  yPoint = sin(dLon) * cos(lat2_rad);
  xPoint = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dLon);
  bearing = atan2(yPoint, xPoint) * 180 / PI - declinacao; // Calculo do angulo em graus
  if(bearing < 0){bearing += 360;}
  distancia = 2 * raioterra * asin(sqrt(pow(sin(dLat/2),2) + cos(lat1_rad)*cos(lat2_rad)*pow(sin(dLon/2),2)));

}
//-------------------------------Algoritmo PID e controle dos motores-----------------------------------------------
void AtuaESCs(float ajuste){ //Envia velocidade e ajuste de direcao aos motores (velocidade_atual)
//ajuste vai de 0 a 255
//-100 e gira para UM LADO =, 100 e gira PARA O OUTRO
//0 nao faz correcao

  ESCA_Vel = int(map(velocidade_atual,-100,100,0,180) + map(ajuste,-100,100,-90,90));
  ESCB_Vel = int(map(velocidade_atual,-100,100,0,180) - map(ajuste,-100,100,-90,90));

  if(ESCA_Vel > 180){ESCA_Vel = 180;}else if(ESCA_Vel<0){ESCA_Vel = 0;}
  if(ESCB_Vel > 180){ESCB_Vel = 180;}else if(ESCB_Vel<0){ESCB_Vel = 0;}
  
  //ESCA_Vel = map(ESCA_Vel,0,180,180,0);
 //ESCB_Vel = map(ESCA_Vel,0,180,180,0);//inverte a rotacao dos motores
  ESCA.write(180- ESCA_Vel);
  ESCB.write(180-ESCB_Vel);
}

void CalculaPIDDirec(){ //Algoritmo generico de controle PID com tempo de intervalo de correcao "dt" 
  //O objetivo do erro e ser 0, entao os valores precisam ir de -255 a 255

  if((millis() - last_time) > dt){
    erro = bearing - direc; //Compara a direcao apontada com a direcao desejada
    if(erro > 180.00){
      erro -= 360.0;
    }else if(erro < -180.0){
      erro += 360.0;
    }
    erro = erro/1.80; //mapeia de 0 a 180 para 0 a 100
    proporcional = (float(percentualP)/100) * kp * erro;
    integral += (float(percentualI)/100) * ki * erro * float(dt);
    derivativo = (float(percentualD)/100) * kd * (erro - ultimo_erro) / float(dt);
    saida = proporcional + integral + derivativo;
    if(saida > 100){saida = 100;}else if(saida < -100){saida = -100;} // Limita a saida do PID de -100 a 100 (%)
    ultimo_erro = erro;
    last_time = millis();
    AtuaESCs(saida);
  }
}

void parar_motores(){ //Funcao auxiliar para parar os motores
  ESCA.write(90);
  ESCB.write(90);
  velocidade_atual = 0;
}

//------------------------------------------------Le GPS------------------------------------------------------------
void leGPS(){
  while (Serial2.available()){gps.encode(Serial2.read());} //Le os dados da porta serial
  if(gps.location.isValid()){
    Latitude  = gps.location.lat(); // float ->aponta os dados atuais de latitude e longitude
    Longitude = gps.location.lng(); // float 
    gpsvalid = true;
    satconectados = gps.satellites.value();
    horaagora = gps.time.value();
    datahoje = gps.date.value();
  }else{gpsvalid = false;}
}
//----------------------------------------------Calibra magnetometro------------------------------------------------
void calibrar_qmc5883(){
  compass.setCalibration(-32768, 32767, -32768, 32767, -32768, 32767);
  int tempo_final = millis() + 5000; //marca o instante de final da calibracao
  max_x = -32768;
  min_x = 32767;
  max_y = -32768;
  min_y = 32767;
  //int max_z = -32768; //Nao e calibrado z, pois nao e usado
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

    Arquivo = SD.open("/dados_calib.txt", FILE_APPEND,true);
    //Se abriu, acrescenta no arquivo
    if(Arquivo) {
    //Latitude; Longitude; LatAlvo[p]; lonAlvo[p]; direc; bearing; erro; ESCA_Vel; ESCB_Vel;
      Arquivo.print(x);
      Arquivo.print(", ");
      Arquivo.print(y);
      Arquivo.println("; ");
      Arquivo.close();
    }
    //if (z > max_z) max_z = z; novo_v = true;
    //if (z < min_z) min_z = z; novo_v = true;
    if (novo_v == true && pronto == false){ 
      tempo_final = millis() + 5000; 
      novo_v = false;
    }

    if (millis() > tempo_final){
      pronto = true; 
      compass.setCalibration(min_x, max_x, min_y, max_y, -32768, 32767); 
      EEPROM.put(0, min_x >> 8);  //Armazena valores de calibracao no EEPROM
      EEPROM.put(1, min_x & 0x00FF);
      EEPROM.put(2, max_x >> 8);
      EEPROM.put(3, max_x & 0x00FF);
      EEPROM.put(4, min_y >> 8);
      EEPROM.put(5, min_y & 0x00FF);
      EEPROM.put(6, max_y >> 8);
      EEPROM.put(7, max_y & 0x00FF);
      EEPROM.commit();

      Arquivo = SD.open("/dados_calib.txt", FILE_APPEND,true);
      if(Arquivo) {
        Arquivo.println("Pronto!");
        Arquivo.close();
      } 
    }
  }
}
//-------------------------------------------Calcula graus----------------------------------------------------------
void CalculaGraus(){ //Calcula a quantos graus esta do norte geografico (direcao que esta apontando)
  compass.read();
  x = compass.getX();
  y = compass.getY();
  direc = atan2(y,x) * 180 / 3.141592653;
  direc = direc - declinacao;
  if (direc < 0) {
    direc += 360;
  }
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
    texto_bme = "Temperatura = " + String(bme_temperatura) + " *C, " + "Umidade = " + String(bme_umidade) + " %, " + "Pressao = " + String(bme_pressao) + " hPa";
    texto_D18S20 = "Temperatura da agua = " + String(sensors.getTempCByIndex(0)) + " *C";

  }else{
    bme_temperatura = 0;
    bme_umidade = 0;
    bme_pressao = 0;
    texto_bme = "Dados nao lidos";
    texto_D18S20 = "Dados nao lidos";
  }

  if(PontoColeta[ponto]){
    if(bomba1_livre){//Bomba 1 livre
      digitalWrite(bomba1, HIGH);
      delay(tempo_enchimento);
      digitalWrite(bomba1, LOW);
      bomba1_livre = false;
      texto_bomba = "Reservatorio 1 enchido";
    }else if(bomba2_livre){//Bomba 2 livre
      digitalWrite(bomba2, HIGH);
      delay(tempo_enchimento-500);//desconta 500ms pois a bomba 2 e um pouco mais forte
      digitalWrite(bomba2, LOW);
      bomba2_livre = false;
      texto_bomba = "Reservatorio 2 enchido";
      }else if(bomba3_livre){//Bomba 3 livre
      digitalWrite(bomba3, HIGH);
      delay(tempo_enchimento);
      digitalWrite(bomba3, LOW);
      bomba3_livre = false;
      texto_bomba = "Reservatorio 3 enchido";
    }else if(bomba4_livre){//Bomba 4 livre
      digitalWrite(bomba4, HIGH);
      delay(tempo_enchimento);
      digitalWrite(bomba4, LOW);
      bomba4_livre = false;
      texto_bomba = "Reservatorio 4 enchido";
    }else{
      texto_bomba = "Todos os reservatorios cheios, nenhum enchido";
  }}else{
    texto_bomba = "Amostra nao coletada";
  }
  leGPS();
  texto_GPS = "Latitude, Longitude: "  + String(Latitude,7) + ", " + String(Longitude,7);
  Arquivo = SD.open("/resultados.txt", FILE_APPEND,true);
//Se abriu, acrescenta no arquivo
  if(Arquivo) {
    Arquivo.println("Ponto de controle");
    Arquivo.print(datahoje);
    Arquivo.print("; ");
    Arquivo.print(horaagora);
    Arquivo.println("; ");
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
    
    u8g2.firstPage();
    do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(4,15,"Erro ao gravar dados");
        u8g2.drawStr(4,30,"Verifique o cartao");
        
      }while(u8g2.nextPage());
      delay(2000);
  }
}
//------------------------------------------Funcao Navegar-------------------------------------------------
void Navegar(){

leGPS();
if(gpsvalid){

LatAlvo[PontosLidos] = Latitude; //Armazena o ponto atual como ultimo ponto (verificar depois de deixar como opcional)
LonAlvo[PontosLidos] = Longitude;
velocidade_atual = 100 * percentualV/100;

for (p = 0; p <= PontosLidos;p++){
velocidade_atual = 100 * percentualV/100;
  do{
    
    leGPS();
    CalculaGraus();
    CalcDirecDist(Latitude,Longitude, LatAlvo[p], LonAlvo[p]); //retorna distancia e angulo do alvo
    CalculaPIDDirec();
    LogSD();
    if(digitalRead(btn_3) == LOW){
      parar_motores();
      TelaAtual=0;
      percursoConcluido = true;
      distancia = precisao-1;
      p = PontosLidos+1;
    }
  if(millis() - last_time_disp > 300){//roda a cada 300ms
    u8g2.firstPage();
    do{//mostra status no display
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.drawStr(16,15,"Atual | Alvo");
      u8g2.setCursor(110, 15);
      u8g2.print(p);//Indica qual ponto esta como alvo
      u8g2.setFont(u8g2_font_squeezed_r6_tr);
      u8g2.setCursor(1, 23);
      u8g2.print(Latitude,8); //Coordenada atual Lat
      u8g2.setCursor(60, 23);
      u8g2.print("|");
      u8g2.setCursor(65, 23);
      u8g2.print(LatAlvo[p],8); //coordenada alvo Lat
      u8g2.setCursor(1, 31);
      u8g2.print(Longitude,8);  //Coordenada atual Lon
      u8g2.setCursor(60, 31);
      u8g2.print("|");
      u8g2.setCursor(65, 31);
      u8g2.print(LonAlvo[p],8); //coordenada alvo Lon
      u8g2.setCursor(1, 38);
      u8g2.print(direc,3);  //mostra direcao apontada
      u8g2.setCursor(60, 38);
      u8g2.print("|");
      u8g2.setCursor(65, 38);
      u8g2.print(bearing,3);  //mostra direcao que deve apontar
      u8g2.setFont(u8g2_font_7x14_tf);
      u8g2.setCursor(1, 53);
      u8g2.print(distancia,1); //mostra distancia em metros
      u8g2.setCursor(86, 53);
      u8g2.print("metros");
      u8g2.setFont(u8g2_font_squeezed_r6_tr);
      u8g2.setCursor(10, 64);
      u8g2.print(ESCB_Vel);
      u8g2.setCursor(86, 64);
      u8g2.print(ESCA_Vel);
    }while(u8g2.nextPage());
      last_time_disp = millis();
      }
  }while(distancia>precisao); //Fazer enquanto a distancia do ponto for maior que 10m
  parar_motores();
  AtividadePonto(p);
  if(p >= PontosLidos){
    percursoConcluido = true; //Sinaliza que a navegacao terminou 
  }
}

}else{
    u8g2.firstPage();
    do{      
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.drawStr(1,15,"GPS invalido");
      u8g2.drawStr(1,30,"Sats conectados");
      u8g2.setCursor(110,30);
      u8g2.print(satconectados);
     }while(u8g2.nextPage());
}
}

//------------------------------------------Mostra Tela-------------------------------------------------------------
void MostraTela(){
  switch (TelaAtual){
    case 0: //Tela inicial (menu) OK
      u8g2.setBitmapMode(1); //faz os bitmaps transparentes
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
        //u8g2.drawBitmap(120,0,8/8,64, epd_bitmap_BarraRolagem); //Desenha Barra de rolagem (Nao implementado)
      }while(u8g2.nextPage());

      break;

    case 1: //Calibra Magnetometro
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
      calibrar_qmc5883();//Chama a funcao para calibrar
      u8g2.firstPage();
      do{
        u8g2.drawStr(4,15,"Calibrando");
        u8g2.drawStr(4,37,"Pronto!"); 
      }while(u8g2.nextPage());
      delay(2000);
      TelaAtual = 0; //volta para o menu principal
      break;

    case 2: //GPS
        leGPS();
        CalcDirecDist(Latitude,Longitude, LatAlvo[coordenada], LonAlvo[coordenada]);
        u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_squeezed_r6_tr);
        u8g2.drawStr(1,8,"Latitude / Longitude Atual");
        u8g2.setCursor(0, 16);
        u8g2.print(Latitude,6);
        u8g2.setCursor(52, 16);
        u8g2.print("/");
        u8g2.setCursor(60, 16);
        u8g2.print(Longitude,6);
        u8g2.drawStr(1,24,"Latitude / Longitude Alvo");
        u8g2.setCursor(0,32);
        u8g2.print(LatAlvo[coordenada],6);
        u8g2.setCursor(52, 32);
        u8g2.print("/");
        u8g2.setCursor(60, 32);
        u8g2.print(LonAlvo[coordenada],6);
        u8g2.drawStr(1,40,"Mag Alvo / Mag Atual");
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

    case 3:  //teste magnetometro
        u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(4,15,"Apontando para");
        u8g2.setCursor(4, 37);
        u8g2.print(direc);
      }while(u8g2.nextPage());
      CalculaGraus();
      break;


    case 4: //Iniciar
    if(PontosLidos == 0){
        u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(29,15,"SEM DADOS!");
        u8g2.setFont(u8g2_font_7x14_tf);
        u8g2.drawStr(4,34,"Reinsira o cartao");
        u8g2.drawStr(15,49,"com os dados e");
        u8g2.drawStr(2,64,"reinicie o sistema");
      }while(u8g2.nextPage());
    }else{
    if(!percursoConcluido){
      Navegar();
      }else{
        u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(4,15,"Percurso");
        u8g2.drawStr(47,30,"Finalizado!");
        u8g2.setFont(u8g2_font_7x14_tf);
        u8g2.drawStr(1,47,"Reinicie o sistema");
        u8g2.drawStr(22,62,"para refazer");
      }while(u8g2.nextPage());
  }}
      break;

    case 5: //Tela ler coordenadas do cartao SD 
        u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(4,15,"Lendo...");
      }while(u8g2.nextPage());

      delay(500);

      if(LerCoordenadas()){
          u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(4,15,"Coordenadas lidas");
        u8g2.setCursor(4, 37);
        u8g2.print(PontosLidos);
        u8g2.print(" pontos");
      }while(u8g2.nextPage());
      }else{
          u8g2.firstPage();
        do{
          u8g2.setFont(u8g2_font_7x14B_tf);
          u8g2.drawStr(1,15,"Falha na leitura");
          u8g2.drawStr(1,30,"Verifique o cartao");
        }while(u8g2.nextPage());
      }
      TelaAtual = 0;
      delay(2000);
      break;

    case 6: //Ajuste manual motor 1
          u8g2.setBitmapMode(1); //faz os bitmaps transparentes
          u8g2.firstPage();
          do{
            u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[5]); //Desenha Icone turbina 1
            u8g2.setFont(u8g2_font_7x14B_tf);
            u8g2.drawStr(26,15,"Vel Motor 1");
            u8g2.setCursor(50, 37);
            u8g2.print(ESCA_Vel);
            u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec);
            u8g2.drawBox(4,45,map(ESCA_Vel, 0, 180, 0,114),16);
          }while(u8g2.nextPage());
      break;
      
    case 7: //Ajuste manual motor 2
          u8g2.setBitmapMode(1); //faz os bitmaps transparentes
          u8g2.firstPage();
          do{
            u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[6]); //Desenha Icone  turbina 2
            u8g2.setFont(u8g2_font_7x14B_tf);
            u8g2.drawStr(26,15,"Vel Motor 2");
            u8g2.setCursor(50, 37);
            u8g2.print(ESCB_Vel);
            u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec);
            u8g2.drawBox(4,45,map(ESCB_Vel, 0, 180, 0,114),16);
          }while(u8g2.nextPage());
      break;

    case 8: //nave
    bearing = 50;
      if(millis() - last_time_disp > 300){//roda a cada 300ms
      u8g2.firstPage();
      do{
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(1,15,"Menu teste");
        u8g2.setCursor(1, 30);
        u8g2.print(bearing);
        u8g2.setCursor(48, 30);
        u8g2.print(direc);
        u8g2.setCursor(4, 45);
        u8g2.print(ESCA_Vel);
        u8g2.setCursor(35, 45);
        u8g2.print(ESCB_Vel);
        u8g2.setCursor(1, 60);
        u8g2.print("V");
        u8g2.setCursor(8, 60);
        u8g2.print(percentualV);
        u8g2.setCursor(32, 60);
        u8g2.print("PID");
        u8g2.setCursor(56, 60);
        u8g2.print(percentualP);
        u8g2.setCursor(80, 60);
        u8g2.print(percentualI);
        u8g2.setCursor(104, 60);
        u8g2.print(percentualD);
      }while(u8g2.nextPage());
      last_time_disp = millis();
      }
      //Escreve os valores nos ESCs
        CalculaGraus();
        CalculaPIDDirec();
        if(aux){
          TextoLog = String(millis()) + "; " + String(proporcional) + "; " + String(integral) + "; " + String(derivativo) + "; " + String(saida) + "; " + String(direc) + "; " + String(bearing) + "; " + String(erro) + "; " + String(ESCA_Vel) + "; " + String(ESCB_Vel) + ";";
        Arquivo = SD.open("/logparado.txt", FILE_APPEND,true);
        if(Arquivo) {
        Arquivo.println(TextoLog);
        Arquivo.close();
        }}
        aux = !aux;

      break;

    case 9://Tela de informacoes
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
          u8g2.setBitmapMode(1); //faz os bitmaps transparentes
          u8g2.firstPage();
          do{
            u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[9]);
            u8g2.setFont(u8g2_font_7x14B_tf);
            u8g2.drawStr(26,15,"Ajuste Vel.");
            u8g2.setCursor(50, 37);
            u8g2.print(percentualV);
            u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec);
            u8g2.drawBox(4,45,map(percentualV, 0, 100, 0,114),16);
          }while(u8g2.nextPage());
      break;

    case 11: //Ajuste P
          u8g2.setBitmapMode(1); //faz os bitmaps transparentes
          u8g2.firstPage();
          do{
            u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[10]);
            u8g2.setFont(u8g2_font_7x14B_tf);
            u8g2.drawStr(26,15,"Ajuste P");
            u8g2.setCursor(50, 37);
            u8g2.print(percentualP);
            u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec);
            u8g2.drawBox(4,45,map(percentualP, 0, 100, 0,114),16);
          }while(u8g2.nextPage());
      break;

      case 12: //Ajuste I
          u8g2.setBitmapMode(1); //faz os bitmaps transparentes
          u8g2.firstPage();
          do{
            u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[11]);
            u8g2.setFont(u8g2_font_7x14B_tf);
            u8g2.drawStr(26,15,"Ajuste I");
            u8g2.setCursor(50, 37);
            u8g2.print(percentualI);
            u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec);
            u8g2.drawBox(4,45,map(percentualI, 0, 100, 0,114),16);
          }while(u8g2.nextPage());
      break;

      case 13: //Ajuste D
          u8g2.setBitmapMode(1); //faz os bitmaps transparentes
          u8g2.firstPage();
          do{
            u8g2.drawBitmap(4,2,16/8,16, Bitmap_Icons[12]);
            u8g2.setFont(u8g2_font_7x14B_tf);
            u8g2.drawStr(26,15,"Ajuste D");
            u8g2.setCursor(50, 37);
            u8g2.print(percentualD);
            u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec);
            u8g2.drawBox(4,45,map(percentualD, 0, 100, 0,114),16);
          }while(u8g2.nextPage());
      break;

      
    default://Caso alguma tela ainda nao implementada
      TelaAtual = 0;
      break;
  }
}

//----------------------------------------------Setup---------------------------------------------------------------OK
void setup(void) {

  //Inicializa diplay, bme e cartao SD
  u8g2.begin();
  bme.begin(0x76);  
  SD.begin(10);
  //Inicializa portas seriais (Serial0 = USB; Serial2 = GPS)
  Serial.begin(115200);
  Serial2.begin(9600);
  //Inicializa EEPROM e carrega os parametros de calibracao do magnetometro
  EEPROM.begin(EEPROM_SIZE);
  min_x=EEPROM.read(0)<<8|EEPROM.read(1);
  max_x=EEPROM.read(2)<<8|EEPROM.read(3);
  min_y=EEPROM.read(4)<<8|EEPROM.read(5);
  max_y=EEPROM.read(6)<<8|EEPROM.read(7);
  percentualP = EEPROM.read(8); //Carrega valores P
  percentualI = EEPROM.read(9);  //Carrega valores I
  percentualD = EEPROM.read(10); //Carrega valores D
  percentualV = EEPROM.read(11); //Carrega valores Velocidade
  //Inicializa o magnetometro e aplica os parametros de calibracao carregados da EEPROM (Apenas valores de x e y sao importantes)
  compass.init();
  compass.setCalibration(min_x, max_x, min_y, max_y, -32768, 32767);
  //Prepara os pinos de entrada para os botoes
  pinMode(btn_1, INPUT_PULLUP);
  pinMode(btn_2, INPUT_PULLUP);
  pinMode(btn_3, INPUT_PULLUP);
  pinMode(bomba1, OUTPUT);
  pinMode(bomba2, OUTPUT);
  pinMode(bomba3, OUTPUT);
  pinMode(bomba4, OUTPUT);
  TelaAtual = 0;

  //Prepara os paramtros dos ESCs conectados 
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

  //Inicializa cartao SD
  Serial.println("Inicializando cartao SD...");
  if (!SD.begin(CS)) {
    Serial.println("Falha na inicializacao! Verificar os cabos e o cartao.");
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

//Comandos para botao 1 (desce)
  if((digitalRead(btn_1) == LOW)&&(btn_1_clic == 0)){
    if(TelaAtual == 0){
      item_sel = item_sel-1;
      if(item_sel<0){item_sel = NUM_ITENS-1;}
    }else if(TelaAtual == 6){
      ESCA_Vel-= 10;//10
      if(ESCA_Vel<0){ESCA_Vel = 0;}
      ESCA.write(180-ESCA_Vel);
    }else if(TelaAtual == 7){
      ESCB_Vel-= 10;//10
      if(ESCB_Vel<0){ESCB_Vel = 0;}
      
      ESCB.write(180-ESCB_Vel);
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
//Comandos para botao 2 (sobe)
  if((digitalRead(btn_2) == LOW)&&(btn_2_clic == 0)){
    if(TelaAtual == 0){
      item_sel = item_sel+1;
      if(item_sel>= NUM_ITENS){item_sel = 0;}
    }else if(TelaAtual == 6){
      ESCA_Vel += 10;//10
      if(ESCA_Vel > 180){ESCA_Vel = 180;}
      ESCA.write(180-ESCA_Vel);
    }else if(TelaAtual == 7){
      ESCB_Vel += 10;
      if(ESCB_Vel > 180){ESCB_Vel = 180;}
      ESCB.write(180-ESCB_Vel);
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
//Comandos para botao 3 (entra)
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
    }else if(TelaAtual == 4 || TelaAtual == 8){
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

//Debounce dos botoes
if((digitalRead(btn_1) == HIGH)&&(btn_1_clic == 1)){btn_1_clic = 0;}
if((digitalRead(btn_2) == HIGH)&&(btn_2_clic == 1)){btn_2_clic = 0;}
if((digitalRead(btn_3) == HIGH)&&(btn_3_clic == 1)){btn_3_clic = 0;}
//Corrige os limites da tela inicial
  item_ant = item_sel-1;
  if(item_ant<0){item_ant = NUM_ITENS-1;}
  item_pos = item_sel+1;
  if(item_pos >= NUM_ITENS){item_pos = 0;}

MostraTela(); //usa a funcao para mostrar dados no display, e as funcoes do display para executar as operacoes
}
