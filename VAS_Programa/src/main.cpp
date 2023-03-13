#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>
#include <telas_menu.h>
#include <math.h>
#include <EEPROM.h>
#include <ESP32Servo.h>

QMC5883LCompass compass;
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Servo ESCA; //motor direito
Servo ESCB; //motor esquerdo

int ESCA_Pin = 18;
int ESCB_Pin = 19;

int minUs = 1000; //tempo de pulso menor
int maxUs = 2000; //tempo de pulso maior
int ESCA_Vel = 0; //velocidade inicial
int ESCB_Vel = 0;


#define EEPROM_SIZE 8

//Variaveis Globais
int selectedMenuItem = 0;
int numMenuItems = 3;
int NivelMenu = 0;
int SelecN1 = 0;
int SelecN2 = 0;
int btn_1 = 12; //Botao desce32
int btn_2 = 13; //Botao sobe33
int btn_3 = 14; //Botao entra34
int16_t max_x = -32768;
int16_t min_x = 32767;
int16_t max_y = -32768;
int16_t min_y = 32767;
int16_t x, y; //, z;

const int NUM_ITENS = 6;

char menu_items[NUM_ITENS] [20] = {
  {"Cal. QMC5883"},
  {"GPS"},
  {"Outro"},
  {"Iniciar"},
  {"Ler Dados"},
  {"Motores"}
};

int item_sel = 1; //Qual item do menu está selecionado
int item_ant; //Item anterior ao selecionado
int item_pos; //Item posterior ao selecionado

int btn_1_clic=0;
int btn_2_clic=0;
int btn_3_clic=0;

int TelaAtual=0;

float direc = 0;

float declinacao = 19.85;



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

case 2:
      u8g2.firstPage();
      do{
        u8g2.drawBitmap(0,21,128/8,21, epd_bitmap_BordaSelec); //Desenha selecionado
        u8g2.drawBitmap(0,43,128/8,21, epd_bitmap_BordaSelec); //Desenha selecionado
        u8g2.drawBox(4,23,map(ESCA_Vel, 0, 180, 0,114),16);
        u8g2.drawBox(4,45,64,16);
      }while(u8g2.nextPage());
  break;

case 3:
    u8g2.firstPage();
  do{
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(4,15,"Apontando para");//cabe 12
    
    u8g2.setCursor(4, 37);
    u8g2.print(direc);
  }while(u8g2.nextPage());
  break;
  
default:
  TelaAtual = 0;
  break;
}
}


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


void ControlaESC(){

  ESCA.write(ESCA_Vel);
  ESCB.write(ESCB_Vel);
}

ESP32PWM pwm;
void setup(void) {
  compass.init();
  u8g2.begin();
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  min_x=EEPROM.read(0)<<8|EEPROM.read(1);
  max_x=EEPROM.read(2)<<8|EEPROM.read(3);
  min_y=EEPROM.read(4)<<8|EEPROM.read(5);
  max_y=EEPROM.read(6)<<8|EEPROM.read(7);
  compass.setCalibration(min_x, max_x, min_y, max_y, -32768, 32767); 
  pinMode(btn_1, INPUT_PULLUP);
  pinMode(btn_2, INPUT_PULLUP);
  pinMode(btn_3, INPUT_PULLUP);
  TelaAtual = 0;

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
  ESCA.setPeriodHertz(50);      // Standard 50hz servo
	ESCB.setPeriodHertz(50);      // Standard 50hz servo
  ESCA.attach(ESCA_Pin, minUs, maxUs);
  ESCB.attach(ESCB_Pin, minUs, maxUs);

}

void loop(void) {


  if((digitalRead(btn_1) == LOW)&&(btn_1_clic == 0)){
    if(TelaAtual == 0){
      item_sel = item_sel-1;
      if(item_sel<0){item_sel = NUM_ITENS-1;}
    }else if(TelaAtual == 2){
      ESCA_Vel-= 10;
    if(ESCA_Vel<0){ESCA_Vel = 0;}
    }

    btn_1_clic = 1;
  }

  if((digitalRead(btn_2) == LOW)&&(btn_2_clic == 0)){
    if(TelaAtual == 0){
      item_sel = item_sel+1;
      if(item_sel>= NUM_ITENS){item_sel = 0;}
    }else if(TelaAtual == 2){
      ESCA_Vel += 10;
      if(ESCA_Vel > 180){ESCA_Vel = 180;}
    }
    btn_2_clic = 1;
  }

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

    case 5:
      TelaAtual = 2;
      break;
    
    default:
      TelaAtual = 0;
      break;
    }      }else{
    TelaAtual = 0;
  }
    btn_3_clic = 1;

  }

if((digitalRead(btn_1) == HIGH)&&(btn_1_clic == 1)){btn_1_clic = 0;}
if((digitalRead(btn_2) == HIGH)&&(btn_2_clic == 1)){btn_2_clic = 0;}
if((digitalRead(btn_3) == HIGH)&&(btn_3_clic == 1)){btn_3_clic = 0;}

  item_ant = item_sel-1;
  if(item_ant<0){item_ant = NUM_ITENS-1;}
  item_pos = item_sel+1;
  if(item_pos >= NUM_ITENS){item_pos = 0;}

CalculaGraus();
MostraTela();






  //delay(1000);
}
