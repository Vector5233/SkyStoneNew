package org.firstinspires.ftc.teamcode;

public class EncoderArray {

    Encoder left, right,center;
    double r1;
    double r2;
    double r3;
    double r1plusr2;
    double r2minusr1;
    double X = 0;
    double Y = 0;
    double relativeX = 0;
    double relativeY = 0;
    double accumulatedX = 0;
    double accumulatedY = 0;
    double accumulatedTheta = 0;
    double theta = 0;

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
        return left.getDiff();
    }
    double getRightDisplacement(){
        return right.getDiff();
    }
    double getCenterDisplacement(){
        return center.getDiff();
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
//        X += getDeltaX()*Math.cos(theta)+getDeltaY()*Math.sin(theta);
//        Y += getDeltaY()*Math.cos(theta)-getDeltaX()*Math.sin(theta);
        theta += getDeltaTheta();
        Y += getDeltaY()*Math.cos(theta)+getDeltaX()*Math.sin(theta);
        X += -getDeltaY()*Math.sin(theta)+getDeltaX()*Math.cos(theta);
        left.reset();
        right.reset();
        center.reset();
    }

    double getDeltaX(){
        return -getCenterPosition()-r3*(getLeftPosition()+getRightPosition())/(r1+r2);
    }

    double getDeltaY(){
        return .5*(getRightPosition()-getLeftPosition()-(r2-r1)*(getLeftPosition()+getRightPosition())/(r1+r2));
    }

    double getDeltaTheta(){
        return (getLeftPosition()+getRightPosition())/(r1+r2);
    }

    double getDx(){
        return -getCenterDisplacement()+r3*(getLeftDisplacement()+getRightDisplacement())/(r1+r2);
    }

    double getDy(){
        return .5*(getRightDisplacement()-getLeftDisplacement()-(r2-r1)*(getLeftDisplacement()+getRightDisplacement())/(r1+r2));
    }

    double getDTheta(){
        return (getLeftDisplacement()+getRightDisplacement())/(r1+r2);
    }

    void updateAll(){
        accumulatedX += getDx()*Math.cos(getDTheta())+getDy()*Math.sin(getDTheta());
        accumulatedY += getDy()*Math.cos(getDTheta())-getDx()*Math.sin(getDTheta());
        accumulatedTheta += getDTheta();
        relativeY += getDy();
        relativeX += getDx();
        left.update();
        right.update();
        center.update();
    }
}
