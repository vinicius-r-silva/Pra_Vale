#include "headers/visualization.h"

using namespace cv;

Visualization::Visualization(){
    
    //variaveis:

    HEIGHT = 2 * _MAX_DIST * _SCALE;
    LENGHT = 2 * _MAX_DIST * _SCALE;

    img = new Mat(HEIGHT, LENGHT, CV_8UC1, Scalar(0));
    imgProcessed = new Mat(HEIGHT, LENGHT, CV_8UC1, Scalar(0));

    ROBOT_X = _MAX_DIST * _SCALE - 5;
    ROBOT_Y = _MAX_DIST * _SCALE - 5;
    ROBOT_SIZE_X = 10;
    ROBOT_SIZE_Y = 10;

    robotRect = new Rect(ROBOT_X, ROBOT_Y, ROBOT_SIZE_X, ROBOT_SIZE_Y);

    sidesInfo = (SidesInfo*) malloc(5 * sizeof(SidesInfo));

    avoidingObs = false;
    nothing = false;
}

void Visualization::createRectangles(){
    float pSizeX = (avoidingObs) ? 0.3 : 1.0;
    float pSizeY = (avoidingObs) ? 0.6 : 1.0;
    

    pSizeX = (nothing) ? _MAX_DIST/2 : pSizeX;
    pSizeY = (nothing) ? _MAX_DIST/2 : pSizeY;
    //float sizeFX = (avoidingObs) ? 1.2 : 1.0;


    //retangulo da frente
    const int FRONT_SIZE_X = 1.0*_SCALE;
    const int FRONT_SIZE_Y = 1.5*_SCALE;
    const int FRONT_X = _MAX_DIST*_SCALE - FRONT_SIZE_X/2;
    const int FRONT_Y = (_MAX_DIST+0.65)*_SCALE;

    //printa o retangulo da frente
    frontRect = new Rect(FRONT_X, FRONT_Y, FRONT_SIZE_X, FRONT_SIZE_Y);

    //retangulo da lateral esquerda
    const int LEFT_SIZE_X = 2*_SCALE * pSizeX; 
    const int LEFT_SIZE_Y = 2.2*_SCALE * pSizeY;
    const int LEFT_X = (_MAX_DIST+0.25)*_SCALE;
    const int LEFT_Y = _MAX_DIST*_SCALE - LEFT_SIZE_Y/3;

    //printa o retangulo da esquerda
    leftRect = new Rect(LEFT_X, LEFT_Y, LEFT_SIZE_X, LEFT_SIZE_Y);

    //retangulo da lateral direita
    const int RIGHT_SIZE_X = 2*_SCALE * pSizeX;
    const int RIGHT_SIZE_Y = 2.2*_SCALE * pSizeY;
    const int RIGHT_X = (_MAX_DIST-0.25)*_SCALE - RIGHT_SIZE_X;
    const int RIGHT_Y = _MAX_DIST*_SCALE - RIGHT_SIZE_Y/3;

    //printa o retangulo da direita
    rightRect = new Rect(RIGHT_X, RIGHT_Y, RIGHT_SIZE_X, RIGHT_SIZE_Y);
}

void Visualization::processImages(const sensor_msgs::PointCloud2::ConstPtr msg){
    int MAX = 255 - _ADD_GRAY_SCALE;

    //Converter o publisher
    sensor_msgs::PointCloud out_pointcloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud);

    geometry_msgs::Point32 pointInput;

    //Preencher as imagens de preto
    *img = Mat::zeros(img->size(), img->type());
    *imgProcessed = Mat::zeros(imgProcessed->size(), imgProcessed->type());

    //pega as informacoes publicadas pelo velodyne
    for(register int i = 0 ; i < out_pointcloud.points.size(); ++i){ 
    
        pointInput = out_pointcloud.points[i];

        //remove os dados que nao sao importantes
        if(isImportant(pointInput)){

            //x é vertical
            pointInput.x += _MAX_DIST;
            pointInput.x *= _SCALE;
            if(pointInput.x > HEIGHT)
                pointInput.x = HEIGHT;
            
            //y é a horizontal
            pointInput.y +=  _MAX_DIST;
            pointInput.y *= _SCALE;
            if(pointInput.y > LENGHT)
                pointInput.y = LENGHT;
            
            //reinforca os pixels com "mais z's" sobre ele
            if(img->at<uchar>((int) pointInput.x +_MAX_DIST_NEG, (int) pointInput.y) < MAX)
                img->at<uchar>((int) pointInput.x +_MAX_DIST_NEG, (int) pointInput.y) += _ADD_GRAY_SCALE;

        }
    }
    //remove os que estao com uma intensidade menor
    inRange(*img,_MIN_GRAY_S,Scalar(255),*imgProcessed);

    //remove as linhas
    erode(*imgProcessed, *imgProcessed, getStructuringElement(MORPH_ELLIPSE, Size(1, 3)));  
}

void Visualization::printRect(){
    //desenha os retangulos
    rectangle(*imgProcessed, *robotRect, 100, 7);
    rectangle(*imgProcessed, *frontRect, 100, 1);
    rectangle(*imgProcessed, *leftRect, 100, 1);
    rectangle(*imgProcessed, *rightRect, 100, 1);

    imshow(windowProcessed, *imgProcessed);
    waitKey(1);
}


void Visualization::getInfo(){
    //pega as informacoes
    sidesInfo[_FRONT].getInfo(imgProcessed, LENGHT, frontRect->x, frontRect->y, frontRect->width, frontRect->height);
    sidesInfo[_LEFT].getInfo(imgProcessed, LENGHT, leftRect->x, leftRect->y, leftRect->width, leftRect->height);
    sidesInfo[_RIGHT].getInfo(imgProcessed, LENGHT, rightRect->x, rightRect->y, rightRect->width, rightRect->height);
    sidesInfo[_FRONT_LEFT].getInfo(imgProcessed, LENGHT, frontRect->x + frontRect->width/2, frontRect->y, frontRect->width/2, frontRect->height);
    sidesInfo[_FRONT_RIGHT].getInfo(imgProcessed, LENGHT, frontRect->x, frontRect->y, frontRect->width/2, frontRect->height);

}

//decide se os pontos do velodyne sao importantes ou nao
bool Visualization::isImportant(geometry_msgs::Point32 pointInput){
  return pointInput.z > _MIN_HIGHT && pointInput.x < _MAX_DIST && pointInput.x > _MAX_DIST_NEG 
        && pointInput.y < _MAX_DIST && pointInput.y > -_MAX_DIST;
}

SidesInfo* Visualization::getSidesInfo(){
    Visualization::getInfo();
    return this->sidesInfo;
}

void Visualization::setAvoidingObs(bool avoid){
    avoidingObs = avoid;
}

void Visualization::setNothing(bool _nothing){
    nothing = _nothing;
}