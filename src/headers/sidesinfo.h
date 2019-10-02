#ifndef SIDES_INFO_H
#define SIDES_INFO_H

#include "consts.h"

class SidesInfo {
    public:
        int area;
        float medX;
        float medY;
        float distance;

        void getInfo(cv::Mat *imgProcessed, int LENGHT, int X, int Y, int SizeX, int SizeY);    
};

#endif