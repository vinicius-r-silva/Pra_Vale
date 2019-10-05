#ifndef ROBOT_H
#define ROBOT_H

#include "consts.h"
#include "sidesinfo.h"

class Robot {
    private:
        //Publicadores para enviar a informação dos vetores abaixo
        ros::Publisher speedPub;
        ros::Publisher wheelPub;

        //Vetores para receberem informações de velocidade dos robos
        pra_vale::RosiMovementArray tractionCommandList;    
        pra_vale::RosiMovementArray wheelsCommandList;

        //Máquina de estado
        enum{WALKING, LADDER_UP, IN_LADDER, LADDER_DOWN, FRONT_OBS, FOLLOW_LEFT,
            FOLLOW_RIGHT, REC_LEFT, REC_RIGHT, CLS_LEFT, CLS_RIGHT};
        int _state;

        //Parâmetros utlizados em cálculos
        double yAngle;
        double zAngle;
        bool sentido;
        bool _isInStairs;
        bool rodar;
        bool avoidingObs;
        bool inObs;

    public:
        Robot();
        void processMap(SidesInfo *sidesInfo);
        void climbStairs();
        void rodarFunction(SidesInfo *sidesInfo);
        void setAngles(double yAngle, double zAngle);
        bool getRodar();
        void setPublishers(ros::Publisher speedPub, ros::Publisher wheelPub);
        bool getAvoidingObs();
};
#endif