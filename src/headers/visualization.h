#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "headers.h"
#include "sidesinfo.h"

class Visualization {
    private:
        //Imagens
        cv::Mat *img;
        cv::Mat *imgProcessed;

        //Janelas
        std::string windowOriginal = "Original";
        std::string windowProcessed = "Threshold";

        //Dimensões da tela
        int HEIGHT;
        int LENGHT;

        //Dimensões do robo para a imagem
        int ROBOT_X;
        int ROBOT_Y;
        int ROBOT_SIZE_X;
        int ROBOT_SIZE_Y;

        //Retangulos para visualizações 
        cv::Rect *robotRect;
        cv::Rect *frontRect;
        cv::Rect *leftRect;
        cv::Rect *rightRect;

        //Informações para os retângulos
        SidesInfo *sidesInfo;

    public:
        Visualization();
        void createRectangles();
        void showImages();
        void processImages(const sensor_msgs::PointCloud2::ConstPtr msg);
        void printImages();
        void getInfo();
        bool isImportant(geometry_msgs::Point32 pointInput);
        SidesInfo* getSidesInfo();

};

#endif