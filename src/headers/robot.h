#ifndef ROBOT_H
#define ROBOT_H

#include "consts.h"
#include "sidesinfo.h"

//Classe que contém os atributos e métodos para o funcionamento do robô pelo Velodyne
class Robot {
    private:
        //Publicadores para enviar a informação dos vetores abaixo
        ros::Publisher speedPub;
        ros::Publisher wheelPub;
        ros::Publisher statePub;

        //Vetores para receberem informações de velocidade dos robos
        std_msgs::Float32MultiArray tractionCommandList;    
        std_msgs::Float32MultiArray wheelsCommandList;

        //Recebe o estado do robô
        std_msgs::Int32 _enable; 

        //Máquina de estado interna da classe para movimentação pela escada
        enum{WALKING, LADDER_UP, IN_LADDER, LADDER_DOWN};
        int _state;

        //Parâmetros utlizados em cálculos
        double _yAngle; //Recebe o ângulo y do robô determinado pelo IMU
        double _zAngle; //Recebe o ângulo z do robô determinado pelo IMU
        double _saveAngle; //Recebe um ângulo para realizar o giro de 180°
        float _distToTrack; //Recebe a distância que o robô deve ficar da esteira conforme a posição dele
        bool _sentido;   //Recebe se o robô segue a esteira no sentido horário ou anti-horário
        bool _isInStairs; //Determina se o robô está na escada
        bool _rodar; //Determina se o robô deve girar
        bool _avoidingObs; // Determina se o robô está desviando de um obstáculo
        bool _straitPath; // Determina se o robô reconheceu um caminho estreito 
        bool _provavelEscada; //Determina se há uma escada por perto para mudar o comportamento do robô

    public:
        Robot(); //Construtor da classe
        void processMap(SidesInfo *sidesInfo); //Algoritmo para processar o mapa e setar as velocidades do robô
        void climbStairs(); //Algoritmo para subir as escadas
        void rodarFunction(SidesInfo *sidesInfo);   //Algoritmo para girar o robô em 180º
        void setAngles(double yAngle, double zAngle);
        void setPublishers(std_msgs::Int32 enable, ros::Publisher speedPub, ros::Publisher wheelPub, ros::Publisher statePub);
        void aligneEscada(SidesInfo *sidesInfo);
        bool getAvoidingObs();
        bool getProvavelEscada();
        bool getIsInStairs();
        bool getRodar();
};
#endif