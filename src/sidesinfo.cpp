#include "headers/sidesinfo.h"

void SidesInfo::getInfo(cv::Mat *img, int LENGHT, int X, int Y, int SizeX, int SizeY){
  int line;
  int column;
  float sumX = 0;
  float sumY = 0;

  uchar* map = img->data;


  this->area = 0;

  for (line = Y; line < Y + SizeY; line++){
      for (column = X; column < X + SizeX; column++){
          if (map[line * LENGHT + column] == 255){
              sumX += column;
              sumY += line;
              (this->area)++;
          }
      }
  }



  
  this->medX = (this->area < _MIN_AREA_REC) ? 10 : (sumX/(this->area*_SCALE)  - _MAX_DIST);
  if(this->medX < 0) this->medX = -this->medX;

  this->medY = (this->area < _MIN_AREA_REC) ? 10 : sumY/(this->area*_SCALE)  - _MAX_DIST;
  this->distance = (this->medX == 10 || this->medY == 10) ? 10 : 
  (float) sqrt(pow(this->medX, 2) + pow(this->medY, 2));

 //std::cout << "AREA: " << this->area << "  MEDX: " << this->medX << "  MEDY: " << this->medY << std::endl;
      
}