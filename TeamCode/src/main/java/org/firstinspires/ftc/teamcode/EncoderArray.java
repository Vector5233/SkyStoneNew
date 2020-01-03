package org.firstinspires.ftc.teamcode;

public class EncoderArray {

    Encoder left, right,center;
    double r1=1;
    double r2=1;

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

    double getDisplacemetX() {
        return 1;
    }
    double getDisplacemetY() {
        return 1;
    }
    double getTurnAngle(){
        return  1;

    }

    void updateAll(){}

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

