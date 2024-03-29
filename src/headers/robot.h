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


        //Recebe o estado do robô
        std_msgs::Int32 _enable;
        std_msgs::Int32 _states;

        //Máquina de estado interna da classe para movimentação pela escada
        enum{WALKING, LADDER_UP, IN_LADDER, LADDER_DOWN};
        int _state;

        //Parâmetros utlizados em cálculos
        double _yAngle;         //Recebe o ângulo y do robô determinado pelo IMU
        double _zAngle;         //Recebe o ângulo z do robô determinado pelo IMU
        float _distToTrack;     //Recebe a distância que o robô deve ficar da esteira conforme a posição dele
        int _avoidSide;         //Define como o robo esta em cada lado
        bool _sentido;          //Recebe se o robô segue a esteira no sentido horário ou anti-horário
        bool _isInStairs;       //Determina se o robô está na escada
        bool _rodar;            //Determina se o robô deve girar
        bool _avoidingObs;      //Determina se o robô está desviando de um obstáculo
        bool _narrowPath;       //Determina se o robô reconheceu um caminho estreito 
        bool _provavelEscada;   //Determina se há uma escada por perto para mudar o comportamento do robô
        bool _climbing;         //Determina se o robo esta subindo a escada ou nao
        bool _nothing;          //Confere se o velodyne nao detectou nada no alcance normal
        bool _begin;            //Determina se precisa analisar o caminho para descobrir o sentido ou nao
        bool _isInNarPath;      //Determina quando o robo está no caminho estreito
        bool _wheelsStable;     //Endireita as esteiras do robo
        
        void climbStairs();                         //Algoritmo para subir as escadas
        void rodarFunction(SidesInfo *sidesInfo);   //Algoritmo para girar o robô em 180º
        void aligneEscada(SidesInfo *sidesInfo);    //alinha o robo com a esteira para subir a escada
        void downStairs();                          //Algoritmo para descer das escadas
        void setSpeed(float tractionDirFront, float tractionDirBack, float tractionLeftFront, float tractionLeftBack);  //Recebe as velocidades e as publica
        void stableWheelTrack();                    //Algoritmo para arrumar as esteiras das rodas

    public:
        Robot();//Construtor da classe
        
        void processMap(SidesInfo *sidesInfo);                                                          //Algoritmo para processar o mapa e setar as velocidades do robô
        void setAngles(double yAngle, double zAngle);                                                   //recebe os angulos do robo a partir do IMU
        void setPublishers(ros::Publisher speedPub, ros::Publisher wheelPub, ros::Publisher statePub);  //seta os ponteiros para publicar
        void setEnable(std_msgs::Int32 enable);                                                         //atualiza o estado do robo
        bool getAvoidingObs();                                                                          //retorna se tem um obstaculo ou nao
        bool getNothing();                                                                              //retorna se tem algum objeto perto
        bool getisInStair();                                                                            //retorna se o robo esta na escada

};
#endif