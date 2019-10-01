//includes para o ros:
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include "pra_vale/RosiMovementArray.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


//opencv:
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//c++:
#include <iostream>
#include <math.h>

//defines-----------------------------

//Analise do veloyne:
#define _MIN_HIGHT -10 //altura minima para capturar dados do velodyne
#define _SCALE  40 //aumenta a resolucao dos dados do velodyne
#define _ADD_GRAY_SCALE 10 //deixa mais definido quais tem mais pontos em z
#define _MAX_DIST 5 //distancia maxima que sera processada
#define _MAX_DIST_NEG -2 //distancia maxima negativa que sera processada
#define _MIN_GRAY_S 100 //minimo de luminosiade para ser processado


//controle do robo
#define _V0 2.2 //velocidade do robo
#define _KP 4.0 //constante para o PID
#define _MAX_SPEED 4.2 //velocidade maxima do robo em rad/s
#define _MIN_DIST_FRONT 2.5 //distancia maxima do obstaculo para virar
#define _DIST_SEGUE_PAREDE 2.3 //distancia ideal para seguir a parede
#define _MAX_WHEEL_R_SPEED 0.52 // maxima velocidade de rotação das rodas


//maquina de estado
#define _NO_OBSTACLE 0
#define _DESVIA_FRENTE 1
#define _SEGUE_ESQUERDA 2
#define _RECUPERA_ESQUERDA 3
#define _SEQUE_DIREITA 4
#define _RECUPERA_DIREITA 5
#define _LADDER_UP 6
#define _HORARIO true 
#define _ANTI_HORARIO false


//informacoes da struct
#define _FRONT 0
#define _LEFT 1
#define _RIGHT 2


//processamento da imagem
#define _MIN_AREA 25
#define _MIN_AREA_REC 15


//namespaces---------------------------
using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;


typedef struct{
  int area;
  float medX;
  float medY;
  float distance;
}Sides_Info_t;


//variaveis globais------------------
ros::Publisher speedPub;
ros::Publisher wheelPub;


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
const int LEFT_SIZE_X = 4*_SCALE;
const int LEFT_SIZE_Y = 2.2*_SCALE;
const int LEFT_X = (_MAX_DIST+0.6)*_SCALE;
const int LEFT_Y = _MAX_DIST*_SCALE - LEFT_SIZE_Y/3;


//retangulo da lateral direita
const int RIGHT_SIZE_X = 4*_SCALE;
const int RIGHT_SIZE_Y = 2.2*_SCALE;
const int RIGHT_X = (_MAX_DIST-0.6)*_SCALE - RIGHT_SIZE_X;
const int RIGHT_Y = _MAX_DIST*_SCALE - RIGHT_SIZE_Y/3;


int estado = _NO_OBSTACLE; //salva o estado
float saveAngle = 10; 
bool sentido = _HORARIO;
bool _isLadderInFront = true;
bool rodar = false;
double yAngle = 0.0f;

//imagens
Mat img(HEIGHT, LENGHT, CV_8UC1, Scalar(0)); //imagem com o isImportant() aplicado
Mat imgProcessed(HEIGHT, LENGHT, CV_8UC1, Scalar(0)); //imagem depois do processamento do opencv


//informacao dos lados
Sides_Info_t *sidesInfo;

//velocidade do robo
pra_vale::RosiMovementArray tractionCommandList;

//velocidade para a roda do robo
pra_vale::RosiMovementArray wheelsCommandList;


void getInfo(int SIDE, int X, int Y, int SizeX, int SizeY){

  int line;
  int column;
  float sumX = 0;
  float sumY = 0;

  uchar* map = imgProcessed.data;


  sidesInfo[SIDE].area = 0;


  for (line = Y; line < Y + SizeY; line++){
      for (column = X; column < X + SizeX; column++){
          if (map[line * LENGHT + column] == 255){
              sumX += column;
              sumY += line;
              (sidesInfo[SIDE].area)++;
          }
      }
  }

  
  sidesInfo[SIDE].medX = (sidesInfo[SIDE].area < _MIN_AREA_REC) ? 10 : (sumX/(sidesInfo[SIDE].area*_SCALE)  - _MAX_DIST);
  if(sidesInfo[SIDE].medX < 0) sidesInfo[SIDE].medX = -sidesInfo[SIDE].medX;

  sidesInfo[SIDE].medY = (sidesInfo[SIDE].area < _MIN_AREA_REC) ? 10 : sumY/(sidesInfo[SIDE].area*_SCALE)  - _MAX_DIST;
  sidesInfo[SIDE].distance = (sidesInfo[SIDE].medX == 10 || sidesInfo[SIDE].medY == 10) ? 10 : 
  (float) sqrt(pow(sidesInfo[SIDE].medX, 2) + pow(sidesInfo[SIDE].medY, 2));
  
}

void climbLadder(){
  pra_vale::RosiMovement wheelCommand;
  float wheelFrontSpeed;
  float wheelRearSpeed;

  if(yAngle < 0.01f && yAngle > -0.01f){
    
    cout << "FRONT WHEELS IS ON\n";
    wheelRearSpeed = _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = -1.0f * _MAX_WHEEL_R_SPEED;
  
  }else if(yAngle > 0.07f || yAngle < -0.07f){

    cout << "REAR WHEELS IS ON\n";
    wheelRearSpeed = -1.0f *_MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = _MAX_WHEEL_R_SPEED;

  }else if(yAngle < 0.003f && yAngle > -0.003f){

    cout << "IT'S OKAY\n";
    wheelRearSpeed = 0.0f;
    wheelFrontSpeed = 0.0f;

  }else{

    cout << "ESTABILIZING\n";
    wheelRearSpeed = -1.0f * _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = 0.0f;

  }

  for(int i = 0; i < 4; i++){
    wheelCommand.joint_var = (i == 0 || i == 2)? wheelFrontSpeed : wheelRearSpeed;
    wheelCommand.nodeID = i + 1;
    wheelsCommandList.movement_array.push_back(wheelCommand);    
  }

  wheelPub.publish(wheelsCommandList);

  wheelsCommandList.movement_array.pop_back();
  wheelsCommandList.movement_array.pop_back();
  wheelsCommandList.movement_array.pop_back();
  wheelsCommandList.movement_array.pop_back();  
}

//controle do robo a partir do mapa
void processMap(){

  //imgProcessed e sidesInfo são variaveis globais

  uchar *map = imgProcessed.data;
  float erro;

  pra_vale::RosiMovement tractionCommandDir;
  pra_vale::RosiMovement tractionCommandEsq;
  

  //pega as informacoes de cada lado
  getInfo(_FRONT, FRONT_X, FRONT_Y, FRONT_SIZE_X, FRONT_SIZE_Y);
  getInfo(_LEFT, LEFT_X, LEFT_Y, LEFT_SIZE_X, LEFT_SIZE_Y);
  getInfo(_RIGHT, RIGHT_X, RIGHT_Y, RIGHT_SIZE_X, RIGHT_SIZE_Y);
  


  //Implementacao da maquina de estado

  //sobe a escada 
  if(_isLadderInFront){
    climbLadder();
    
    estado = _LADDER_UP;

    cout << "E: SubirEscada\t yAngle: " << yAngle;

  //desvia do obstaculo na frente
  }else if(sidesInfo[_FRONT].medY < _MIN_DIST_FRONT){

    if(sentido ==_HORARIO)
      erro = 1/(sidesInfo[_FRONT].medY);
    else
      erro = -1/(sidesInfo[_FRONT].medY);
    

    estado = _DESVIA_FRENTE;

    cout << "E: DesviaFr";
    cout << " | AF: " << sidesInfo[_FRONT].area;
    cout << " | DF: " << sidesInfo[_FRONT].distance;
    cout << " | DFY: " << sidesInfo[_FRONT].medY;


  //esta entre dois obstaculos
  /*}else if(sidesInfo[_RIGHT].area > _MIN_AREA_REC && sidesInfo[_LEFT].area > _MIN_AREA_REC){
    if(sidesInfo[_RIGHT].medX < sidesInfo[_LEFT].medY){
      erro = sidesInfo[_RIGHT].medX - sidesInfo[_LEFT].medX;
    }
  */


  //Recupera o trajeto da direita
  }else if(sidesInfo[_RIGHT].medY < -0.2 && sentido == _HORARIO){
    
    erro = -1/(sidesInfo[_RIGHT].medX);
    estado = _RECUPERA_DIREITA;

    cout << "E: RercuDir";
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DD: " << sidesInfo[_RIGHT].distance;
    cout << " | DDX: " << sidesInfo[_RIGHT].medX;


  //segue a parede da direita
  }else if(sidesInfo[_RIGHT].area > _MIN_AREA){

    erro = (_DIST_SEGUE_PAREDE - sidesInfo[_RIGHT].medX)/2.5;
    estado = _SEQUE_DIREITA;

    cout << "E: SegueDir";
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DD: " << sidesInfo[_RIGHT].distance;
    cout << " | DDX: " << sidesInfo[_RIGHT].medX;


  //Recupera o trajeto da esquerda
  }else if(sidesInfo[_LEFT].medY < -0.2 && sentido == _ANTI_HORARIO){
    
    erro = 1/(sidesInfo[_LEFT].medX);
    estado = _RECUPERA_ESQUERDA;

    cout << "E: RercuEsq";
    cout << " | AE: " << sidesInfo[_LEFT].area;
    cout << " | DE: " << sidesInfo[_LEFT].distance;
    cout << " | DEX: " << sidesInfo[_LEFT].medX;


  //segue a parede da esquerda
  }else if(sidesInfo[_LEFT].area > _MIN_AREA){

    erro = -(_DIST_SEGUE_PAREDE - sidesInfo[_LEFT].medX)/2.5;
    estado = _SEGUE_ESQUERDA;

    cout << "E: SegueEsq";
    cout << " | AE: " << sidesInfo[_LEFT].area;
    cout << " | DE: " << sidesInfo[_LEFT].distance;
    cout << " | DEX: " << sidesInfo[_LEFT].medX;


  //segue reto caso nao tenha nada
  }else{
    
    erro = 0;
    estado = _NO_OBSTACLE;

    cout << "E: NormalEt";
  
  }

  if(_isLadderInFront){
    tractionCommandDir.joint_var = (float) _MAX_SPEED;
    tractionCommandEsq.joint_var = (float) _MAX_SPEED; 
  }else{
    tractionCommandDir.joint_var = (float) _V0 + _KP*erro;
    tractionCommandEsq.joint_var = (float) _V0 - _KP*erro;
  }

  if(tractionCommandDir.joint_var > _MAX_SPEED)
    tractionCommandDir.joint_var = _MAX_SPEED;
  else if(tractionCommandDir.joint_var < -_MAX_SPEED)
    tractionCommandDir.joint_var = -_MAX_SPEED;


  if(tractionCommandEsq.joint_var > _MAX_SPEED)
    tractionCommandEsq.joint_var = _MAX_SPEED;
  else if(tractionCommandEsq.joint_var < -_MAX_SPEED)
    tractionCommandEsq.joint_var = -_MAX_SPEED;


  cout << " | VEsq: " << tractionCommandEsq.joint_var << " | VDir: " << tractionCommandDir.joint_var << endl;


  //altera o vetor das velocidades das 'joints'

  //Direita:
  tractionCommandDir.nodeID = 1;
  tractionCommandList.movement_array.push_back(tractionCommandDir);
  tractionCommandDir.nodeID = 2;
  tractionCommandList.movement_array.push_back(tractionCommandDir);
  
  //Esquerda:
  tractionCommandEsq.nodeID = 3;
  tractionCommandList.movement_array.push_back(tractionCommandEsq);
  tractionCommandEsq.nodeID = 4;
  tractionCommandList.movement_array.push_back(tractionCommandEsq);

  return;
}


//remove os pontos nao necessarios
bool isImportant(geometry_msgs::Point32 pointInput){
  return pointInput.z > _MIN_HIGHT && pointInput.x < _MAX_DIST && pointInput.x > _MAX_DIST_NEG 
        && pointInput.y < _MAX_DIST && pointInput.y > -_MAX_DIST;
}



void rodarFunction(float zAngle){
  
  float dif;

  pra_vale::RosiMovement tractionCommandDir;
  pra_vale::RosiMovement tractionCommandEsq;

  if(zAngle < 0)
    zAngle += M_PI*2;
    
  if(saveAngle == 10)
    saveAngle = zAngle;

  if(sentido == _HORARIO){
    tractionCommandDir.joint_var = 3;
    tractionCommandEsq.joint_var = -3;

    if(saveAngle >= 0 && saveAngle < M_PI && zAngle > M_PI)
        dif = saveAngle + M_PI*2 - zAngle;

    else
      dif = saveAngle - zAngle;
  
  }else{
    tractionCommandDir.joint_var = -3;
    tractionCommandEsq.joint_var = 3;

    if(zAngle >= 0 && zAngle < M_PI && saveAngle > M_PI)
        dif =  M_PI*2 - saveAngle + zAngle;
      
    else
      dif = saveAngle - zAngle;
  }

  if(dif < 0)
    dif *=-1;

  if(dif > 3){
    rodar = false; //para de rodar
    sentido = !sentido; //troca o sentido
    saveAngle = 10; //da um reset no angulo
  } 
    

  cout << "girando" << " | zAngle: " << zAngle << " | saveAngle: " << saveAngle  << " | dif: " << dif <<" | velE: " << tractionCommandEsq.joint_var << " | velD: " << tractionCommandDir.joint_var << endl;


  //Direita:
  tractionCommandDir.nodeID = 1;
  tractionCommandList.movement_array.push_back(tractionCommandDir);
  tractionCommandDir.nodeID = 2;
  tractionCommandList.movement_array.push_back(tractionCommandDir);
  
  //Esquerda:
  tractionCommandEsq.nodeID = 3;
  tractionCommandList.movement_array.push_back(tractionCommandEsq);
  tractionCommandEsq.nodeID = 4;
  tractionCommandList.movement_array.push_back(tractionCommandEsq);


  speedPub.publish(tractionCommandList);
  
  //limpa o vetor
  tractionCommandList.movement_array.pop_back();
  tractionCommandList.movement_array.pop_back();
  tractionCommandList.movement_array.pop_back();
  tractionCommandList.movement_array.pop_back();  

}


void velodyneCallback(const  sensor_msgs::PointCloud2::ConstPtr msg){
/*
  if(rodar)
    return;

  int MAX = 255-_ADD_GRAY_SCALE;

  //converte o publisher
  sensor_msgs::PointCloud2 input_pointcloud;
  sensor_msgs::PointCloud out_pointcloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud);


  geometry_msgs::Point32 pointInput;


  //preenche as imagens de preto
  img = Mat::zeros(img.size(), img.type());
  imgProcessed = Mat::zeros(imgProcessed.size(), imgProcessed.type());

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
      if(img.at<uchar>((int) pointInput.x +_MAX_DIST_NEG, (int) pointInput.y) < MAX)
        img.at<uchar>((int) pointInput.x +_MAX_DIST_NEG, (int) pointInput.y) += _ADD_GRAY_SCALE;

    }
  }

  //remove os que estao com uma intensidade menor
  inRange(img,_MIN_GRAY_S,Scalar(255),imgProcessed);

  //remove as linhas
  erode(imgProcessed, imgProcessed, getStructuringElement(MORPH_ELLIPSE, Size(1, 3)));  
  
  
  //desenha o retangulo que representa o robo
  Rect robotRect(ROBOT_X, ROBOT_Y, ROBOT_SIZE_X, ROBOT_SIZE_Y);
  rectangle(imgProcessed,robotRect,100,7);


  //desenha os retangulos das areas anlisadas pelo codigo
  Rect frontRect(FRONT_X, FRONT_Y, FRONT_SIZE_X, FRONT_SIZE_Y);
  rectangle(imgProcessed, frontRect, 100, 1);
  
  Rect leftRect(LEFT_X, LEFT_Y, LEFT_SIZE_X, LEFT_SIZE_Y);
  rectangle(imgProcessed, leftRect, 100, 1);

  Rect rightRect(RIGHT_X, RIGHT_Y, RIGHT_SIZE_X, RIGHT_SIZE_Y);
  rectangle(imgProcessed, rightRect, 100, 1);


  //faz o processamento para controlar as velocidades do motor
  processMap();
  

  imshow("Threshold", imgProcessed);
  imshow("Original", img);
  waitKey(1);
*/

  processMap();

  speedPub.publish(tractionCommandList);
  
  //limpa o vetor
  tractionCommandList.movement_array.pop_back();
  tractionCommandList.movement_array.pop_back();
  tractionCommandList.movement_array.pop_back();
  tractionCommandList.movement_array.pop_back();
}

void ImuCallback(const  sensor_msgs::Imu::ConstPtr msg){
  
  float qx = msg->orientation.x;
  float qy = msg->orientation.y;
  float qz = msg->orientation.z;
  float qw = msg->orientation.w;
  float zAngle;

  double t2 = 2.0f * (qw * qy - qx * qx);
  t2 = (t2 > 1.0f)? 1.0f : t2;
  t2 = (t2 < -1.0f)? -1.0f : t2;
  yAngle = asin(t2);

  float t3 = +2.0 * (qw * qz + qx * qy);
  float t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      
  zAngle = atan2(t4, t3) + M_PI/2;  //z angulo de euler

  if(rodar){
    rodarFunction(zAngle);
    return;
  }

}

void kinectCallback(const  sensor_msgs::ImageConstPtr msg){
  
/*

 cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  namedWindow("kinect");
  imshow("kinect", cv_ptr->image);
  waitKey(1);
*/
  //Mat imgKinect = msg;

}

int main(int argc, char **argv){
  

  //inicializa as janelas
/*  String windowOriginal = "Original"; 
  String windowProcessed = "Threshold";
  namedWindow(windowOriginal);
  namedWindow(windowProcessed);
*/  
  sidesInfo = (Sides_Info_t*) malloc(sizeof(Sides_Info_t)*3);

  //inicia o Ros  
  ros::init(argc, argv, "velodyme");

  ros::NodeHandle n;

  //le o publisher do vrep
  ros::Subscriber sub = n.subscribe("/sensor/velodyne", 1, velodyneCallback);

  ros::Rate loop_rate(1);
  ros::Subscriber subVelodyne = n.subscribe("/sensor/velodyne", 1, velodyneCallback);
  ros::Subscriber subImu = n.subscribe("/sensor/imu", 1, ImuCallback);
  ros::Subscriber subKinectRGB = n.subscribe("/sensor/kinect_rgb", 1, kinectCallback);

  speedPub = n.advertise<pra_vale::RosiMovementArray>("/rosi/command_traction_speed",1);
  wheelPub = n.advertise<pra_vale::RosiMovementArray>("/rosi/command_arms_speed",1);
 
  ros::spin();
  

  return 0;
}
