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
        enum{WALKING, LADDER_UP, IN_LADDER, LADDER_DOWN};
        int _state;

        //Parâmetros utlizados em cálculos
        double yAngle;
        double zAngle;
        double saveAngle;
        bool sentido;
        bool _isInStairs;
        bool _climbing;
        bool rodar;

    public:
        Robot();
        void processMap(SidesInfo *sidesInfo);
        void climbStairs();
        void rodarFunction();
        void setAngles(double yAngle, double zAngle);
        bool getRodar();
        void setPublishers(ros::Publisher speedPub, ros::Publisher wheelPub);
};
#endif