#include <Arduino.h>

//O calculo do desvio magnético pode ser calculado pelos dados encontrados no site https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
#define mag_dev_g 19 //Desvio magnético calculado na cidade de blumenau de 19° 51' W -- Pelo mapa obtido no WMM, o desvio é negativo
#define mag_dev_m 51 //Se é W deve subtrair do heading
#define mag_dev_p 'W'



double gms_graus(double graus, double minutos, char direcao){
  int sinal = 1;
  if(direcao == 'W'){
    sinal = -1;
  }else if(direcao == 'E'){
    sinal = 1;
  }
  return sinal * (graus + (minutos/60)); //após este passo, deve-se somar com o valor medido este obtido
}


void setup() {
  double graus = gms_graus(mag_dev_g, mag_dev_m, mag_dev_p); //A medida real é o polo magnético menos o desvio
}

void loop() {
  // put your main code here, to run repeatedly:
}