//includes para o ros:
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

//opencv:
#include <opencv2/opencv.hpp>

//c++:
#include <iostream>



//defines----------------------------

//tela:
#define HEIGHT 600 //altura da tela
#define LENGHT 600 //compriemnto

//Analise do veloyne:
#define _MIN_HIGHT 0 //altura minima para capturar dados do velodyne
#define _SCALE  40 //aumenta a resolucao dos dados do velodyne
#define _ADD_GRAY_SCALE 15 //deixa mais definido quais tem mais pontos em z
#define _MAX_DIST 6.5 //distancia maxima negativa que sera tolerada
#define _MIN_GRAY_S 100



//namespaces-------------------------

using namespace cv;
using namespace std;



//struct para os pontos do velodyne
typedef struct{
  float x;
  float y;
  float z;
}points_t;




//desenha o mapa com as informacoes
void drawMap(points_t *pointData, int quant/*, float minX, float minY*/){

  int MAX = 255-_ADD_GRAY_SCALE;
  int i=0;
  
  Mat img(HEIGHT, LENGHT, CV_8UC1, Scalar(0)); //imagem com o isImportant() aplicado
  Mat imgProcessed(HEIGHT, LENGHT, CV_8UC1, Scalar(0)); //imagem depois do processamento do opencv
  
  String windowOriginal = "original"; 
  String windowProcessed = "Threshold";
  namedWindow(windowOriginal); //cria a janela
  namedWindow(windowProcessed); 

  for(i = 0 ; i < quant;  i++){

    pointData[i].x += _MAX_DIST;
    pointData[i].x *= _SCALE;
    if(pointData[i].x > HEIGHT)
      pointData[i].x = HEIGHT;
    
    
    pointData[i].y +=  _MAX_DIST;
    pointData[i].y *= _SCALE;
    if(pointData[i].y > LENGHT)
      pointData[i].y = LENGHT;
    
    //reinforca os pixels com "mais z's" sobre ele
    if(img.at<uchar>((int) pointData[i].x, (int) pointData[i].y) < MAX)
      img.at<uchar>((int) pointData[i].x, (int) pointData[i].y) += _ADD_GRAY_SCALE;

  }

  //remove os pixels mais fracos
  //inRange(img,_MIN_GRAY_S,Scalar(255),imgProcessed);

  erode(img, imgProcessed, getStructuringElement(MORPH_ELLIPSE, Size(1, 3)));


  Rect robotRect(_MAX_DIST*_SCALE-5, _MAX_DIST*_SCALE-5, 10, 15);
  Rect robotRectFront(_MAX_DIST*_SCALE-5, _MAX_DIST*_SCALE+15, 10, 5);
  rectangle(imgProcessed,robotRect,100,7);
  rectangle(imgProcessed,robotRectFront,255,7);



  imshow(windowOriginal, img); //mostra a imagem original
  imshow(windowProcessed, imgProcessed); //mostra a imagem depois de ser processada
  waitKey(1);

}


//remove os pontos nao necessarios
bool isImportant(geometry_msgs::Point32 pointInput){

  return pointInput.z > _MIN_HIGHT && pointInput.x < _MAX_DIST && pointInput.x > -_MAX_DIST 
        && pointInput.y < _MAX_DIST && pointInput.y > -_MAX_DIST;
}



bool dataProcessing(const sensor_msgs::PointCloud out_pointcloud){

  int quant = 0;
  //float minX = 0;
  //float minY = 0;
  points_t *pointData;

  pointData = (points_t*) malloc(sizeof(points_t)*out_pointcloud.points.size());


  for(int i = 0 ; i < out_pointcloud.points.size(); ++i){
    geometry_msgs::Point32 pointInput;  
    pointInput = out_pointcloud.points[i];


    //remove os dados que nao sao importantes
    if(isImportant(pointInput)){
      
      pointData[quant].x = (float) pointInput.x;
      pointData[quant].y = (float) pointInput.y;
      pointData[quant].z = (float) pointInput.z;

      quant++;
    }
  }

  //desenha o mapa
  drawMap(pointData, quant-1);

  free(pointData);

}



void velodyneCallback(const  sensor_msgs::PointCloud2::ConstPtr msg){

  //converte o publisher
  sensor_msgs::PointCloud2 input_pointcloud;
  sensor_msgs::PointCloud out_pointcloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud);


  dataProcessing(out_pointcloud);
}



int main(int argc, char **argv){
  //inicia o node
  ros::init(argc, argv, "velodyme");

  
  ros::NodeHandle n;

  //le o publisher do vrep
  ros::Subscriber sub = n.subscribe("/sensor/velodyne", 1, velodyneCallback);

  
  ros::spin();

  //destroyWindow("create_image");
  return 0;
}
