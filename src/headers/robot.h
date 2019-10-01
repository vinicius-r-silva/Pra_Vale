#ifndef ROBOT_H
#define ROBOT_H

#include "headers.h"
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
        enum{NO_OBSTACLE, DESVIA_FRENTE, SEGUE_ESQUERDA, RECUPERA_ESQUERDA,
            SEGUE_DIREITA, RECUPERA_DIREITA, LADDER_UP, IN_LADDER};
        int _state;

        //Parâmetros utlizados em cálculos
        double yAngle;
        double zAngle;
        double saveAngle;
        bool sentido;
        bool _isStairsInFront;
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