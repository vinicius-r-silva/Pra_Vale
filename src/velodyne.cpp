//includes para o ros:
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "pra_vale/RosiMovementArray.h"

//opencv:
#include <opencv2/opencv.hpp>


//c++:
#include <iostream>


//defines----------------------------

//Analise do veloyne:
#define _QUANT_POINTS 200000 //quantidade de pontos processados pelo velodyne
#define _MIN_HIGHT -1 //altura minima para capturar dados do velodyne
#define _SCALE  40 //aumenta a resolucao dos dados do velodyne
#define _ADD_GRAY_SCALE 10 //deixa mais definido quais tem mais pontos em z
#define _MAX_DIST 5 //distancia maxima que sera processada
#define _MAX_DIST_NEG -2 //distancia maxima negativa que sera processada
#define _MIN_GRAY_S 100 //minimo de luminosiade para ser processado


//namespaces-------------------------
using namespace cv;
using namespace std;



//struct para os pontos do velodyne
typedef struct{
  float x;
  float y;
}points_t;


//variaveis globais------------------

//dimensoes da tela
int HEIGHT = 2*_MAX_DIST*_SCALE;
int LENGHT = 2*_MAX_DIST*_SCALE;


//imagens
Mat img(HEIGHT, LENGHT, CV_8UC1, Scalar(0)); //imagem com o isImportant() aplicado
Mat imgProcessed(HEIGHT, LENGHT, CV_8UC1, Scalar(0)); //imagem depois do processamento do opencv


//vetor x, y
points_t *pointData;


//velocidade do robo
pra_vale::RosiMovementArray tractionCommandList;


//controlo o robo a partir do mapa
void processMap(Mat map){


  return;
}

//desenha o mapa com as informacoes
Mat drawMap(points_t *pointData, int quant/*, float minX, float minY*/){

  int MAX = 255-_ADD_GRAY_SCALE;
  int i=0;
  

  //preenche as imagens de preto
  img = Mat::zeros(img.size(), img.type());
  imgProcessed = Mat::zeros(imgProcessed.size(), imgProcessed.type());

  
  String windowOriginal = "original"; 
  String windowProcessed = "Threshold";
  namedWindow(windowOriginal); //cria a janela
  namedWindow(windowProcessed); 

  for(i = 0 ; i < quant;  i++){

    //x é vertical
    pointData[i].x += _MAX_DIST;
    pointData[i].x *= _SCALE;
    if(pointData[i].x > HEIGHT)
      pointData[i].x = HEIGHT;
    
    //y é a horizontal
    pointData[i].y +=  _MAX_DIST;
    pointData[i].y *= _SCALE;
    if(pointData[i].y > LENGHT)
      pointData[i].y = LENGHT;
    
    //reinforca os pixels com "mais z's" sobre ele
    if(img.at<uchar>((int) pointData[i].x +_MAX_DIST_NEG, (int) pointData[i].y) < MAX)
      img.at<uchar>((int) pointData[i].x +_MAX_DIST_NEG, (int) pointData[i].y) += _ADD_GRAY_SCALE;

  }


  //remove os pixels mais fracos
  inRange(img,_MIN_GRAY_S,Scalar(255),imgProcessed);

  erode(imgProcessed, imgProcessed, getStructuringElement(MORPH_ELLIPSE, Size(1, 3)));  
  
  
  Rect robotRect(_MAX_DIST*_SCALE-5, _MAX_DIST*_SCALE-5, 10, 15);
  rectangle(imgProcessed,robotRect,100,7);


  imshow(windowOriginal, img); //mostra a imagem original
  imshow(windowProcessed, imgProcessed); //mostra a imagem depois de ser processada
  waitKey(1);

  return imgProcessed;

}


//remove os pontos nao necessarios
bool isImportant(geometry_msgs::Point32 pointInput){

  return pointInput.z > _MIN_HIGHT && pointInput.x < _MAX_DIST && pointInput.x > _MAX_DIST_NEG 
        && pointInput.y < _MAX_DIST && pointInput.y > -_MAX_DIST;
}



bool dataProcessing(const sensor_msgs::PointCloud out_pointcloud){

  int quant = 0;
  geometry_msgs::Point32 pointInput;  
  pra_vale::RosiMovement tractionCommand;


  for(int i = 0 ; i < out_pointcloud.points.size(); ++i){
    
    pointInput = out_pointcloud.points[i];


    //remove os dados que nao sao importantes
    if(isImportant(pointInput)){
      pointData[quant].x = (float) pointInput.x;
      pointData[quant].y = (float) pointInput.y;
      quant++;
    }
  }


  //desenha o mapa
  Mat map = drawMap(pointData, quant-1);

  processMap(map);

  tractionCommand.joint_var = 50;
  tractionCommand.nodeID = 1;
  tractionCommandList.movement_array.push_back(tractionCommand);
  tractionCommand.nodeID = 2;
  tractionCommandList.movement_array.push_back(tractionCommand);
  tractionCommand.nodeID = 3;
  tractionCommandList.movement_array.push_back(tractionCommand);
  tractionCommand.nodeID = 4;
  tractionCommandList.movement_array.push_back(tractionCommand);
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

  pointData = (points_t*) malloc(sizeof(points_t)*_QUANT_POINTS);
  ros::init(argc, argv, "velodyme");

  
  ros::NodeHandle n;

  //le o publisher do vrep
  ros::Subscriber sub = n.subscribe("/sensor/velodyne", 1, velodyneCallback);
  ros::Publisher speedPub = n.advertise<pra_vale::RosiMovementArray>("/rosi/command_traction_speed",3);

  ros::Rate loop_rate(10);
 
 
  while (ros::ok()){
    speedPub.publish(tractionCommandList);
    ros::spinOnce();
    loop_rate.sleep();
  }

  //ros::spin();

  
  return 0;
}
