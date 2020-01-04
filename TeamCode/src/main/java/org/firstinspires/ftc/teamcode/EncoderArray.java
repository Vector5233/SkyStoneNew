package org.firstinspires.ftc.teamcode;

public class EncoderArray {

    Encoder left, right,center;
    double r1=1;
    double r2=1;
    double X = 0;
    double Y = 0;
    double angle = 0;


    public EncoderArray(Encoder myLeft, Encoder myRight, Encoder myCenter, double myR1, double myR2){
        left = myLeft;
        right = myRight;
        center = myCenter;
        r1 = myR1;
        r2 = myR2;
    }

    double getLeft(){
        return left.getDisplacement();
    }
    double getRight(){
        return right.getDisplacement();
    }
    double getCenter(){
        return center.getDisplacement();
    }
    void resetAll () {
        left.reset();
        right.reset();
        center.reset();
    }

    double getX() {
        return X;
    }
    double getY() {
        return Y;
    }
    double getTurnAngle(){
        return angle;

    }

    void updateAll(){
        X=X+getDx()*Math.cos(getDAngle())+getDy()*Math.sin(getDAngle());
        Y=Y+getDy()*Math.cos(getDAngle())-getDx()*Math.sin(getDAngle());
        angle=angle+getDAngle();
        left.update();
        right.update();
        center.update();
    }

    double getDx(){
        return getCenter()+r2/r1*getD2();
    }
    double getDy(){
        return (getRight()+getLeft())/2;
    }
    double getDAngle(){
        return getD2()/r1;
    }
    double getD2 () {
        return (getRight()-getLeft())/2;
    }
}
