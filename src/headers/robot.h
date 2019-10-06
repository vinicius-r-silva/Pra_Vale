#ifndef ROBOT_H
#define ROBOT_H

#include "consts.h"
#include "sidesinfo.h"

class Robot {
    private:
        //Publicadores para enviar a informação dos vetores abaixo
        ros::Publisher speedPub;
        ros::Publisher wheelPub;
        ros::Publisher statePub;

        //Vetores para receberem informações de velocidade dos robos
        std_msgs::Float32MultiArray tractionCommandList;    
        std_msgs::Float32MultiArray wheelsCommandList;

        //Máquina de estado
        enum{WALKING, LADDER_UP, IN_LADDER, LADDER_DOWN, FRONT_OBS, FOLLOW_LEFT,
            FOLLOW_RIGHT, REC_LEFT, REC_RIGHT, CLS_LEFT, CLS_RIGHT};
        int _state;

        //Parâmetros utlizados em cálculos
        std_msgs::Int32 _enable;
        double yAngle;
        double zAngle;
        double saveAngleZ;
        double saveAngle;
        float distToTrack;
        bool sentido;
        bool _isInStairs;
        bool rodar;
        bool avoidingObs;
        bool inObs;
        bool straitPath;
        bool _provavelEscada;

    public:
        Robot();
        void processMap(SidesInfo *sidesInfo);
        void climbStairs();
        void rodarFunction(SidesInfo *sidesInfo);
        void setAngles(double yAngle, double zAngle);
        bool getRodar();
        void setPublishers(std_msgs::Int32 enable, ros::Publisher speedPub, ros::Publisher wheelPub, ros::Publisher statePub);
        bool getAvoidingObs();
        void aligneEscada(SidesInfo *sidesInfo);
        bool getProvavelEscada();
        bool getIsInStairs();
        bool getSentido();
        bool getStraitPath();
        void setStatePub(std_msgs::Int32 enable);
};
#endif