#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "consts.h"
#include "sidesinfo.h"

class Visualization {
    private:
        //Imagens
        cv::Mat *img;           //Imagem original
        cv::Mat *imgProcessed;  //Imagem processada

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
        int FRONT_SIZE_X = 1.0*_SCALE;

        //Retangulos para visualizações 
        cv::Rect *robotRect;
        cv::Rect *frontRect;
        cv::Rect *leftRect;
        cv::Rect *rightRect;

        //Informações para os retângulos
        SidesInfo *sidesInfo;   //Guarda todas as informacoes dos lados
        bool avoidingObs;       //Muda o tamanho da janela caso tenha detectado um obstaculo
        bool nothing;           //Muda o tamanho da janela caso nao tenha detectado nada perto do robo


    public:
        Visualization(); //Construtor do robo

        void createRectangles();                                            //cria os retangulos onde a imagem sera processada
        void setAvoidingObs(bool avoid);                                    //seta se tem um obstaculo ou nao
        void setNothing(bool _nothing);                                     //se nao pegou nada ao analisar a imagem
        void showImages();                                                  //mostra as imagens na tela
        void processImages(const sensor_msgs::PointCloud2::ConstPtr msg);   //processa a imagem, controlando o robo
        void printRect();                                                   //printa os retangulos na imagem
        void getInfo();                                                     //compila as informacoes importantes
        bool isImportant(geometry_msgs::Point32 pointInput);                //decide se a informacao do veloyne e valida
        SidesInfo* getSidesInfo();                                          //retorna as informacoes dos lados

};

#endif