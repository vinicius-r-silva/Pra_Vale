#include "headers/visualization.h"

using namespace cv;

Visualization::Visualization(){
    
    HEIGHT = 2 * _MAX_DIST * _SCALE;
    LENGHT = 2 * _MAX_DIST * _SCALE;

    img = new Mat(HEIGHT, LENGHT, CV_8UC1, Scalar(0));
    imgProcessed = new Mat(HEIGHT, LENGHT, CV_8UC1, Scalar(0));

    ROBOT_X = _MAX_DIST * _SCALE - 5;
    ROBOT_Y = _MAX_DIST * _SCALE - 5;
    ROBOT_SIZE_X = 10;
    ROBOT_SIZE_Y = 10;

    robotRect = new Rect(ROBOT_X, ROBOT_Y, ROBOT_SIZE_X, ROBOT_SIZE_Y);

    sidesInfo = (SidesInfo*) malloc(3 * sizeof(SidesInfo));
}

void Visualization::createRectangles(){

    //retangulo da frente
    const int FRONT_SIZE_X = 2.0*_SCALE - 20;
    const int FRONT_SIZE_Y = 1.5*_SCALE;
    const int FRONT_X = _MAX_DIST*_SCALE - FRONT_SIZE_X/2;
    const int FRONT_Y = (_MAX_DIST+0.85)*_SCALE + 10;
    //const int FRONT_Y = (_MAX_DIST+0.75)*_SCALE;

    frontRect = new Rect(FRONT_X, FRONT_Y, FRONT_SIZE_X, FRONT_SIZE_Y);

    //retangulo da lateral esquerda
    const int LEFT_SIZE_X = 4*_SCALE;
    const int LEFT_SIZE_Y = 2.2*_SCALE;
    const int LEFT_X = (_MAX_DIST+0.6)*_SCALE;
    const int LEFT_Y = _MAX_DIST*_SCALE - LEFT_SIZE_Y/3;

    leftRect = new Rect(LEFT_X, LEFT_Y, LEFT_SIZE_X, LEFT_SIZE_Y);

    //retangulo da lateral direita
    const int RIGHT_SIZE_X = 4*_SCALE;
    const int RIGHT_SIZE_Y = 2.2*_SCALE;
    const int RIGHT_X = (_MAX_DIST-0.6)*_SCALE - RIGHT_SIZE_X;
    const int RIGHT_Y = _MAX_DIST*_SCALE - RIGHT_SIZE_Y/3;

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
    rectangle(*imgProcessed, *robotRect, 100, 7);
    rectangle(*imgProcessed, *frontRect, 100, 1);
    rectangle(*imgProcessed, *leftRect, 100, 1);
    rectangle(*imgProcessed, *rightRect, 100, 1);

    imshow(windowProcessed, *imgProcessed);
    imshow(windowOriginal, *img);
    waitKey(1);

}

void Visualization::getInfo(){
    sidesInfo[_FRONT].getInfo(imgProcessed, LENGHT, frontRect->x, frontRect->y, frontRect->width, frontRect->height);
    sidesInfo[_LEFT].getInfo(imgProcessed, LENGHT, leftRect->x, leftRect->y, leftRect->width, leftRect->height);
    sidesInfo[_RIGHT].getInfo(imgProcessed, LENGHT, rightRect->x, rightRect->y, rightRect->width, rightRect->height);
}

bool Visualization::isImportant(geometry_msgs::Point32 pointInput){
  return pointInput.z > _MIN_HIGHT && pointInput.x < _MAX_DIST && pointInput.x > _MAX_DIST_NEG 
        && pointInput.y < _MAX_DIST && pointInput.y > -_MAX_DIST;
}

SidesInfo* Visualization::getSidesInfo(){
    Visualization::getInfo();
    return this->sidesInfo;
}