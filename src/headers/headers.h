//includes para o ros:
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
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

//defines
#define _MIN_HIGHT -10 //altura minima para capturar dados do velodyne
#define _MAX_DIST 5 //distancia maxima que sera processada
#define _SCALE 40 //aumenta a resolucao dos dados do velodyne
#define _ADD_GRAY_SCALE 10 //deixa mais definido quais tem mais pontos em z
#define _MAX_DIST_NEG -2 //distancia maxima negativa que sera processada
#define _MIN_GRAY_S 100 //minimo de luminosiade para ser processado

//Analise do velodyne:
#define _NOTHING 0
#define _VELODYNE_ENABLED 1
#define _ARM_CHANGING_POSE 2
#define _FOLLOW_TRACK 3
#define _FOUND_FIRE_FRONT 4
#define _FOUND_FIRE_RIGHT 5
#define _FOUND_FIRE_TOUCH 6
#define _SETTING_UP_HOKUYO 7
#define _INITIAL_SETUP     8

//controle do robo
#define _V0 2.0 //velocidade do robo
#define _KP 5.0 //constante para o PID
#define _KP_OBSTACLE 15.0
#define _MAX_SPEED 4.2 //velocidade maxima do robo em rad/s
#define _MIN_DIST_FRONT 2.5 //distancia maxima do obstaculo para virar
#define _DIST_SEGUE_PAREDE 1.5 //distancia ideal para seguir a parede
#define _MAX_WHEEL_R_SPEED 0.52 // maxima velocidade de rotação das rodas
#define _HORARIO true 
#define _ANTI_HORARIO false

//informacoes da struct
#define _FRONT 0
#define _LEFT 1
#define _RIGHT 2

//processamento da imagem
#define _MIN_AREA 25
#define _MIN_AREA_REC 15