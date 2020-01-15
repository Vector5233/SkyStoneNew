package org.firstinspires.ftc.teamcode;

import java.lang.reflect.Array;

public class EncoderArray {

    Encoder left, right,center;
    double r1;
    double r2;
    double r3;
    double r1plusr2;
    double r2minusr1;
    double X = 0;
    double Y = 0;
    double angle = 0;



    public EncoderArray(Encoder myLeft, Encoder myRight, Encoder myCenter, double myR1, double myR2, double myR3){
        left = myLeft;
        right = myRight;
        center = myCenter;
        r1 = myR1;
        r2 = myR2;
        r3 = myR3;

    }

    double getLeftPosition(){
        return left.getPosition();
    }
    double getRightPosition(){
        return right.getPosition();
    }
    double getCenterPosition(){
        return center.getPosition();
    }

    double getLeftDisplacement(){
        return left.getDisplacement();
    }
    double getRightDisplacement(){
        return right.getDisplacement();
    }
    double getCenterDisplacement(){
        return center.getDisplacement();
    }

    public double[] calibrate(double degrees) {
        double theta = degrees * Math.PI / 180.0;
        r1plusr2 = (left.getPosition()+right.getPosition())/theta;
        r2minusr1 = r1plusr2*(-left.getPosition()+right.getPosition())/(left.getPosition()+right.getPosition());
        r1 = (r1plusr2-r2minusr1)/2;
        r2 = (r1plusr2+r2minusr1)/2;
        r3 = -center.getPosition() * r1plusr2 / (left.getPosition()+right.getPosition());
        double[] radii={r1, r2, r3};
        return radii;
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
    double getDegrees(){
        return angle;

    }

    void updateAll(){
        X=X+getDx()*Math.cos(getDTheta())+getDy()*Math.sin(getDTheta());
        Y=Y+getDy()*Math.cos(getDTheta())-getDx()*Math.sin(getDTheta());
        angle=angle+getDTheta();
        left.update();
        right.update();
        center.update();
    }

    double getDx(){
        return getCenterDisplacement()-r2*(getLeftDisplacement()+getRightDisplacement())/r1;
    }
    double getDy(){
        return (getRightDisplacement()-getLeftDisplacement())/2;
    }
    double getDTheta(){
        return getLeftDisplacement()+getRightDisplacement()/r1;
    }

}
