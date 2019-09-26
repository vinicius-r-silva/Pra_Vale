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
#include <math.h>


//defines----------------------------

//Analise do veloyne:
#define _QUANT_POINTS 100000 //quantidade de pontos processados pelo velodyne
#define _MIN_HIGHT -1 //altura minima para capturar dados do velodyne
#define _SCALE  40 //aumenta a resolucao dos dados do velodyne
#define _ADD_GRAY_SCALE 10 //deixa mais definido quais tem mais pontos em z
#define _MAX_DIST 5 //distancia maxima que sera processada
#define _MAX_DIST_NEG -2 //distancia maxima negativa que sera processada
#define _MIN_GRAY_S 100 //minimo de luminosiade para ser processado


//controle do robo
#define _NO_OBSTACLE 0
#define _FRONT_OBSTACLE 1
#define _RIGHT_OBSTACLE 2
#define _LEFT_OBSTACLE 3
#define _V0 4
#define _KP 4
#define _MIN_DIST_FRONT 2.5

//informacoes da struct
#define _FRONT 0
#define _LEFT 1
#define _RIGHT 2

//processamento da imagem
#define _MIN_AREA 20


//namespaces-------------------------
using namespace cv;
using namespace std;



//struct para os pontos do velodyne
typedef struct{
  float x;
  float y;
}points_t;


typedef struct{
  int area;
  float medX;
  float medY;
  float distance;
}Sides_Info_t;

//variaveis globais------------------

//dimensoes da tela
int HEIGHT = 2*_MAX_DIST*_SCALE;
int LENGHT = 2*_MAX_DIST*_SCALE;

//dimensoes do robo
int ROBOT_X = _MAX_DIST*_SCALE-5;
int ROBOT_Y = _MAX_DIST*_SCALE-5;
int ROBOT_SIZE_X = 10;
int ROBOT_SIZE_Y = 10;

//retangulo da frente
int FRONT_SIZE_X = 2.5*_SCALE;
int FRONT_SIZE_Y = 1.5*_SCALE;
int FRONT_X = _MAX_DIST*_SCALE - FRONT_SIZE_X/2;
int FRONT_Y = (_MAX_DIST+0.75)*_SCALE;

//retangulo da lateral esquerda
int LEFT_SIZE_X = 1.0*_SCALE;
int LEFT_SIZE_Y = 1.5*_SCALE;
int LEFT_X = _MAX_DIST*_SCALE+65-LEFT_SIZE_X;
int LEFT_Y = _MAX_DIST*_SCALE-10;


//retangulo da lateral direita
int RIGHT_SIZE_X = 1.0*_SCALE;
int RIGHT_SIZE_Y = 1.5*_SCALE;
int RIGHT_X = _MAX_DIST*_SCALE-65;
int RIGHT_Y = _MAX_DIST*_SCALE-10;


//imagens
Mat img(HEIGHT, LENGHT, CV_8UC1, Scalar(0)); //imagem com o isImportant() aplicado
Mat imgProcessed(HEIGHT, LENGHT, CV_8UC1, Scalar(0)); //imagem depois do processamento do opencv


//vetor x, y
points_t *pointData;

//informacao dos lados
Sides_Info_t *sidesInfo;

//velocidade do robo
pra_vale::RosiMovementArray tractionCommandList;


void getInfo(int SIDE, int frontX, int frontY, int frontSizeX, int frontSizeY){


  int line;
  int column;
  int sumX = 0;
  int sumY = 0;

  uchar* map = imgProcessed.data;


  sidesInfo[SIDE].area = 0;


  for(line = FRONT_Y; line < FRONT_Y+FRONT_SIZE_Y; line++){
    for (column = FRONT_X; column < FRONT_X+FRONT_SIZE_X; column++){
      if(map[line*LENGHT + column] == 255){
        sumX += column;
        sumY += line;
        (sidesInfo[SIDE].area)++;
      } 
    }
  }
  
  if(sidesInfo[SIDE].area >= _MIN_AREA){
    sidesInfo[SIDE].medX = (sidesInfo[SIDE].area == 0)? 0.0 :(float) sumX/(sidesInfo[SIDE].area*_SCALE);
    sidesInfo[SIDE].medY = (sidesInfo[SIDE].area == 0)? 0.0 :(float) sumY/(sidesInfo[SIDE].area*_SCALE);
    sidesInfo[SIDE].distance = (sidesInfo[SIDE].medX == 0 || sidesInfo[SIDE].medY == 0)? 0.0 : 
    (float) sqrt(pow(sidesInfo[SIDE].medX -_MAX_DIST, 2) + pow(sidesInfo[SIDE].medY -_MAX_DIST, 2));
  }else{
    sidesInfo[SIDE].area = 0;
  }

}



//controle do robo a partir do mapa
void processMap(){
  //imgProcessed e sidesInfo são variaveis globais
  
  uchar *map = imgProcessed.data;
  int obstacleDirection = _NO_OBSTACLE;
  
  pra_vale::RosiMovement tractionCommandDir;
  pra_vale::RosiMovement tractionCommandEsq;
  

  //pega as informacoes de cada lado
  getInfo(_FRONT, FRONT_X, FRONT_Y, FRONT_SIZE_X, FRONT_SIZE_Y);
  getInfo(_LEFT, LEFT_X, LEFT_Y, LEFT_SIZE_X, LEFT_SIZE_Y);
  getInfo(_RIGHT, RIGHT_X, RIGHT_Y, RIGHT_SIZE_X, RIGHT_SIZE_Y);
  
  

  if(sidesInfo[_FRONT].distance < _MIN_DIST_FRONT && sidesInfo[_FRONT].area >= _MIN_AREA){
    tractionCommandDir.nodeID = 1;
    tractionCommandDir.joint_var = _V0 +_KP*(1/(sidesInfo[_FRONT].distance));
    tractionCommandEsq.nodeID = 3;
    tractionCommandEsq.joint_var = _V0 -_KP*(1/(sidesInfo[_FRONT].distance));
  }else if( ){

  }else{
    tractionCommandDir.nodeID = 1;
    tractionCommandDir.joint_var = _V0;

    tractionCommandEsq.nodeID = 3;
    tractionCommandEsq.joint_var = _V0;
  }




  tractionCommandList.movement_array.push_back(tractionCommandDir);
  tractionCommandDir.nodeID = 2;
  tractionCommandList.movement_array.push_back(tractionCommandDir);
  tractionCommandList.movement_array.push_back(tractionCommandEsq);
  tractionCommandEsq.nodeID = 4;
  tractionCommandList.movement_array.push_back(tractionCommandEsq);

  cout << "Area: " << sidesInfo[_FRONT].area;
  cout << " | Distancia: " << sidesInfo[_FRONT].distance;
  cout << " | VEsq: " << tractionCommandEsq.joint_var << " | VDir: " << tractionCommandDir.joint_var << endl;

  return;
}

//desenha o mapa com as informacoes
void drawMap(points_t *pointData, int quant/*, float minX, float minY*/){

  int MAX = 255-_ADD_GRAY_SCALE;
  int i=0;
  

  //preenche as imagens de preto
  img = Mat::zeros(img.size(), img.type());
  imgProcessed = Mat::zeros(imgProcessed.size(), imgProcessed.type());

  
  String windowOriginal = "Original"; 
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
  
  
  
  Rect robotRect(ROBOT_X, ROBOT_Y, ROBOT_SIZE_X, ROBOT_SIZE_Y);
  rectangle(imgProcessed,robotRect,100,7);

  Rect frontRect(FRONT_X, FRONT_Y, FRONT_SIZE_X, FRONT_SIZE_Y);
  rectangle(imgProcessed, frontRect, 100, 1);
  
  Rect leftRect(LEFT_X, LEFT_Y, LEFT_SIZE_X, LEFT_SIZE_Y);
  rectangle(imgProcessed, leftRect, 100, 1);

  Rect rightRect(RIGHT_X, RIGHT_Y, RIGHT_SIZE_X, RIGHT_SIZE_Y);
  rectangle(imgProcessed, rightRect, 100, 1);

  //mostra a imagem original
  //imshow(windowProcessed, imgProcessed); //mostra a imagem depois de ser processada
  //waitKey(1);

  return;

}


//remove os pontos nao necessarios
bool isImportant(geometry_msgs::Point32 pointInput){
  return pointInput.z > _MIN_HIGHT && pointInput.x < _MAX_DIST && pointInput.x > _MAX_DIST_NEG 
        && pointInput.y < _MAX_DIST && pointInput.y > -_MAX_DIST;
}



bool dataProcessing(const sensor_msgs::PointCloud out_pointcloud){

  int quant = 0;
  geometry_msgs::Point32 pointInput;  


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
  drawMap(pointData, quant-1);

  processMap();
  

  imshow("Threshold", imgProcessed);
  imshow("Original", img);
  waitKey(1);
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
  sidesInfo = (Sides_Info_t*) malloc(sizeof(Sides_Info_t)*3);
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
