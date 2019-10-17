#ifndef SIDES_INFO_H
#define SIDES_INFO_H

#include "consts.h"

class SidesInfo {
    public:
        int area;           //Area em pixels
        float medX;         //Media no eixo X
        float medY;         //Media no eixo Y
        float distance;     //Distancia do robo as medias

        void getInfo(cv::Mat *imgProcessed, int LENGHT, int X, int Y, int SizeX, int SizeY);    //processa a imagem e pega a informacao dos lados
};

#endif