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
#define _QUANT_POINTS 150000 //quantidade de pontos processados pelo velodyne
#define _MIN_HIGHT -1 //altura minima para capturar dados do velodyne
#define _SCALE  40 //aumenta a resolucao dos dados do velodyne
#define _ADD_GRAY_SCALE 10 //deixa mais definido quais tem mais pontos em z
#define _MAX_DIST 5 //distancia maxima que sera processada
#define _MAX_DIST_NEG -2 //distancia maxima negativa que sera processada
#define _MIN_GRAY_S 100 //minimo de luminosiade para ser processado


//controle do robo
#define _V0 2.5
#define _KP 3.5
#define _MIN_DIST_FRONT 2.5
//#define _MAX_DIST_SEGUE_PAREDE 2.5
#define _DIST_SEGUE_PAREDE 1.8


//maquina de estado
#define _NO_OBSTACLE 0
#define _DESVIA_FRENTE 1
#define _SEGUE_ESQUERDA 2
#define _RECUPERA_ESQUERDA 3
#define _SEQUE_DIREITA 4
#define _RECUPERA_DIREITA 5
#define _HORARIO true 
#define _ANTI_HORARIO false




//informacoes da struct
#define _FRONT 0
#define _LEFT 1
#define _RIGHT 2

//processamento da imagem
#define _MIN_AREA 30


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
const int HEIGHT = 2*_MAX_DIST*_SCALE;
const int LENGHT = 2*_MAX_DIST*_SCALE;


//dimensoes do robo
const int ROBOT_X = _MAX_DIST*_SCALE-5;
const int ROBOT_Y = _MAX_DIST*_SCALE-5;
const int ROBOT_SIZE_X = 10;
const int ROBOT_SIZE_Y = 10;


//retangulo da frente
const int FRONT_SIZE_X = 2.0*_SCALE;
const int FRONT_SIZE_Y = 1.5*_SCALE;
const int FRONT_X = _MAX_DIST*_SCALE - FRONT_SIZE_X/2;
const int FRONT_Y = (_MAX_DIST+0.85)*_SCALE;


//retangulo da lateral esquerda
const int LEFT_SIZE_X = 2.7*_SCALE;
const int LEFT_SIZE_Y = 2.2*_SCALE;
const int LEFT_X = (_MAX_DIST+0.6)*_SCALE;
const int LEFT_Y = _MAX_DIST*_SCALE - LEFT_SIZE_Y/3;


//retangulo da lateral direita
const int RIGHT_SIZE_X = 2.7*_SCALE;
const int RIGHT_SIZE_Y = 2.2*_SCALE;
const int RIGHT_X = (_MAX_DIST-0.6)*_SCALE - RIGHT_SIZE_X;
const int RIGHT_Y = _MAX_DIST*_SCALE - RIGHT_SIZE_Y/3;


//salva o estado
int estado = _NO_OBSTACLE;
bool sentido = _HORARIO;

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


  for(line = frontY; line < frontY+frontSizeY; line++){
    for (column = frontX; column < frontX+frontSizeX; column++){
      if(map[line*LENGHT + column] == 255){
        sumX += column;
        sumY += line;
        (sidesInfo[SIDE].area)++;
      } 
    }
  }
  

  sidesInfo[SIDE].medX = (sidesInfo[SIDE].area == 0)? 10 :(float) sumX/(sidesInfo[SIDE].area*_SCALE);
  sidesInfo[SIDE].medY = (sidesInfo[SIDE].area == 0)? 10 :(float) sumY/(sidesInfo[SIDE].area*_SCALE);
  sidesInfo[SIDE].distance = (sidesInfo[SIDE].medX == 10 || sidesInfo[SIDE].medY == 10)? 10 : 
  (float) sqrt(pow(sidesInfo[SIDE].medX -_MAX_DIST, 2) + pow(sidesInfo[SIDE].medY -_MAX_DIST, 2));

}



//controle do robo a partir do mapa
void processMap(){

  //imgProcessed e sidesInfo são variaveis globais
  uchar *map = imgProcessed.data;

  pra_vale::RosiMovement tractionCommandDir;
  pra_vale::RosiMovement tractionCommandEsq;
  

  //pega as informacoes de cada lado
  getInfo(_FRONT, FRONT_X, FRONT_Y, FRONT_SIZE_X, FRONT_SIZE_Y);
  getInfo(_LEFT, LEFT_X, LEFT_Y, LEFT_SIZE_X, LEFT_SIZE_Y);
  getInfo(_RIGHT, RIGHT_X, RIGHT_Y, RIGHT_SIZE_X, RIGHT_SIZE_Y);
  


  //Implementacao da maquina de estado
  if(sidesInfo[_FRONT].area > _MIN_AREA && sidesInfo[_FRONT].distance < _MIN_DIST_FRONT){

    if(sentido ==_HORARIO){
      tractionCommandDir.joint_var = _V0 +_KP*(1/(sidesInfo[_FRONT].distance));
      tractionCommandEsq.joint_var = _V0 -_KP*(1/(sidesInfo[_FRONT].distance));
    }else{
      tractionCommandDir.joint_var = _V0 -_KP*(1/(sidesInfo[_FRONT].distance));
      tractionCommandEsq.joint_var = _V0 +_KP*(1/(sidesInfo[_FRONT].distance));
    }

    estado = _DESVIA_FRENTE;

    cout << "E: DesviaFr";
    cout << " | AF: " << sidesInfo[_FRONT].area;
    cout << " | DF: " << sidesInfo[_FRONT].distance;
    cout << " | DFY: " << sidesInfo[_FRONT].medY;

  

  }else if(sidesInfo[_RIGHT].medY < _MAX_DIST - 0.2 && sidesInfo[_RIGHT].area > _MIN_AREA/2){
    
    tractionCommandDir.joint_var = _V0 - _KP*(1/(sidesInfo[_RIGHT].distance));
    tractionCommandEsq.joint_var = _V0 + _KP*(1/(sidesInfo[_RIGHT].distance));
    estado = _RECUPERA_DIREITA;

    cout << "E: RercuDir";
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DD: " << sidesInfo[_RIGHT].distance;
    cout << " | DDY: " << sidesInfo[_RIGHT].medY;



  }else if(sidesInfo[_RIGHT].area > _MIN_AREA){

    tractionCommandDir.joint_var = _V0 +_KP*(_DIST_SEGUE_PAREDE - sidesInfo[_RIGHT].distance)/3.5;
    tractionCommandEsq.joint_var = _V0 -_KP*(_DIST_SEGUE_PAREDE - sidesInfo[_RIGHT].distance)/3.5;
    estado = _SEQUE_DIREITA;

    cout << "E: SegueDir";
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DD: " << sidesInfo[_RIGHT].distance;
    cout << " | DDY: " << sidesInfo[_RIGHT].medY;



  }else if(sidesInfo[_LEFT].medY < _MAX_DIST - 0.2 && sidesInfo[_LEFT].area > _MIN_AREA/2){
    
    tractionCommandDir.joint_var = _V0 + _KP*(1/(sidesInfo[_LEFT].distance));
    tractionCommandEsq.joint_var = _V0 - _KP*(1/(sidesInfo[_LEFT].distance));
    estado = _RECUPERA_ESQUERDA;

    cout << "E: RercuEsq";
    cout << " | AE: " << sidesInfo[_LEFT].area;
    cout << " | DE: " << sidesInfo[_LEFT].distance;
    cout << " | DEY: " << sidesInfo[_LEFT].medY;



  }else if(sidesInfo[_LEFT].area > _MIN_AREA){

    tractionCommandDir.joint_var = _V0 - _KP*(_DIST_SEGUE_PAREDE - sidesInfo[_LEFT].distance)/3.5;
    tractionCommandEsq.joint_var = _V0 + _KP*(_DIST_SEGUE_PAREDE - sidesInfo[_LEFT].distance)/3.5;
    estado = _SEGUE_ESQUERDA;

    cout << "E: SegueEsq";
    cout << " | AE: " << sidesInfo[_LEFT].area;
    cout << " | DE: " << sidesInfo[_LEFT].distance;
    cout << " | DEY: " << sidesInfo[_LEFT].medY;


  }else{
    
    tractionCommandDir.joint_var = _V0;
    tractionCommandEsq.joint_var = _V0;
    estado = _NO_OBSTACLE;

    cout << "E: NormalEt";
  
  }
  cout << " | VEsq: " << tractionCommandEsq.joint_var << " | VDir: " << tractionCommandDir.joint_var << endl;



  tractionCommandDir.nodeID = 1;
  tractionCommandList.movement_array.push_back(tractionCommandDir);
  tractionCommandDir.nodeID = 2;
  tractionCommandList.movement_array.push_back(tractionCommandDir);
  
  tractionCommandEsq.nodeID = 3;
  tractionCommandList.movement_array.push_back(tractionCommandEsq);
  tractionCommandEsq.nodeID = 4;
  tractionCommandList.movement_array.push_back(tractionCommandEsq);

  return;
}

//desenha o mapa com as informacoes
void drawMap(points_t *pointData, int quant){

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


  for(int i = 0 ; i < out_pointcloud.points.size() && i < _QUANT_POINTS; ++i){
    
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
