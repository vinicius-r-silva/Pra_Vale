#include "headers/sidesinfo.h"

void SidesInfo::getInfo(cv::Mat *img, int LENGHT, int X, int Y, int SizeX, int SizeY){
  int line;
  int column;
  float sumX = 0;
  float sumY = 0;

  uchar* map = img->data;


  area = 0;

  //processa as informacoes do retangulo
  for (line = Y; line < Y + SizeY; line++){ 
      for (column = X; column < X + SizeX; column++){
          if (map[line * LENGHT + column] == 255){
              sumX += column;
              sumY += line;
              (area)++;
          }
      }
  }

  medX = (area < _MIN_AREA_REC) ? 10 : (sumX/(area*_SCALE)  - _MAX_DIST);
  if(medX < 0) medX = -medX;

  medY = (area < _MIN_AREA_REC) ? 10 : sumY/(area*_SCALE)  - _MAX_DIST;
  distance = (medX == 10 || medY == 10) ? 10 : 
  (float) sqrt(pow(medX, 2) + pow(medY, 2));
  
}