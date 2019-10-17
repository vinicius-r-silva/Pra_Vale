//includes para o ros:
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>


//opencv:
#include <opencv2/opencv.hpp>

//c++:
#include <iostream>
#include <math.h>

//DEFINES

//Processamento do velodyne
#define _MIN_HIGHT        -10  //altura minima para capturar dados do velodyne
#define _MAX_DIST         5    //distancia maxima que sera processada
#define _SCALE            40   //aumenta a resolucao dos dados do velodyne
#define _ADD_GRAY_SCALE   10   //deixa mais definido quais tem mais pontos em z
#define _MAX_DIST_NEG     -2   //distancia maxima negativa que sera processada
#define _MIN_GRAY_S       100  //minimo de luminosiade para ser processado


//Estados
#define NOTHING              0
#define ENABLE_VELODYME      1
#define ARM_CHANGING_POSE    2
#define FOUND_FIRE_FRONT     4
#define FOUND_FIRE_LEFT      5
#define FOUND_FIRE_RIGHT     6
#define FOUND_FIRE_TOUCH     7
#define SETTING_UP_HOKUYO    8
#define INITIAL_SETUP        9
#define ROBOT_ROTATION       10
#define ROBOT_CLOCKWISE      11
#define ROBOT_ANTICLOCKWISE  12
#define HOKUYO_READING       13
#define FOUND_STAIR          14
#define LEAVING_FIRE         15
#define STRAIT_PATH          16
#define END_STAIR        	 17
#define IN_STAIR             18
#define CLIMB_STAIR          19


//controle da velocidade do robo
#define _V0                 3.0  //velocidade do robo
#define _KP                 5.5  //constante para o PID
#define _KP_OBSTACLE        1.2  
#define _MAX_SPEED          4.2  //velocidade maxima do robo em rad/s
#define _KP_REC             0.7 
#define _MAX_WHEEL_R_SPEED  1.0  // maxima velocidade de rotação das rodas



//deteccao de objetos
#define _MIN_AREA       25
#define _MIN_AREA_REC   10 //area minima para utilizar o metodo de recuDir ou recuEsq


//informacoes da struct
#define _FRONT          0
#define _LEFT           1
#define _RIGHT          2
#define _FRONT_LEFT     3
#define _FRONT_RIGHT    4
#define _FRONT_MIDLE    5


//Distancias para controle do robo

//Desviar de objetos
#define _MIN_DIST_FRONT         2.5   //distancia maxima do obstaculo para virar
#define _MIN_SAFE_DIST_SPIN     0.75  //distancia minima da frente para girar
#define NICE_DIST_TRACK         0.80  //Distancia ideal para seguir a esteira

//Alinhar com a escada
#define _MIN_DIST_ESCADA                1.00  //distancia minima para mandar o robo subir
#define _MIN_DIST_ALIGNE_FRONT_STAIR    1.1   //distancia minima para comecar a alinhar com a esteira para subir a escada
#define _MAX_DIST_SIDE_ESCADA           0.85  //distancia maxima do robo para esteira na hora da escada
#define _MIN_DIST_SIDE_ESCADA           0.65  //distancia minima da esteira para subir a escada
#define _MAX_ERRO_ESCADA                0.05  //erro no angulo maximo permitido na escada


//Parametros do sentido
#define _HORARIO        true  
#define _ANTI_HORARIO   false 


//Subir a  escada, angulos para cada fase de subida do robo
#define FRONT_WHEELS    0.25 
#define REAR_WHEELS     0.37
#define PLANE           0.12

