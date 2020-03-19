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

    double getLeftPosition;
    double getRightPosition;
    double getCenterPosition;
    /*double getLeftDisplacement;
    double getRightDisplacement;
    double getCenterDisplacement;*/

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

    void resetAll() {
//        X += getDeltaX()*Math.cos(theta)+getDeltaY()*Math.sin(theta);
//        Y += getDeltaY()*Math.cos(theta)-getDeltaX()*Math.sin(theta);
        theta += getDeltaTheta();
        Y += getDeltaY()*Math.cos(theta*Math.PI/180)+getDeltaX()*Math.sin(theta*Math.PI/180);
        X += -getDeltaY()*Math.sin(theta*Math.PI/180)+getDeltaX()*Math.cos(theta*Math.PI/180);
        left.reset();
        right.reset();
        center.reset();
    }

    double getDeltaX(){
        return -getCenterPosition-r3*(getLeftPosition+getRightPosition)/(r1+r2);
    }

    double getDeltaY(){
        return .5*(getRightPosition-getLeftPosition-(r2-r1)*(getLeftPosition+getRightPosition)/(r1+r2));
    }

    double getDeltaTheta(){
        return (180/Math.PI)*(getLeftPosition+getRightPosition)/(r1+r2);
    }

    void trashStrafeAngleBeforeReset() {
        theta-=getDeltaTheta();
    }

    double getDeltaThetaRad(){
        return (getLeftPosition+getRightPosition)/(r1+r2);
    }

    void readEncoderValue(){
        getLeftPosition = left.getPosition();
        getRightPosition = right.getPosition();
        getCenterPosition = center.getPosition();
        /*getLeftDisplacement = left.getDiff();
        getRightDisplacement = right.getDiff();
        getCenterDisplacement = center.getDiff();*/
    }
}

    /*double getDx(){
        return -getCenterDisplacement+r3*(getLeftDisplacement+getRightDisplacement)/(r1+r2);
    }

    double getDy(){
        return .5*(getRightDisplacement-getLeftDisplacement-(r2-r1)*(getLeftDisplacement+getRightDisplacement)/(r1+r2));
    }

    double getDTheta(){
        return (getLeftDisplacement+getRightDisplacement)/(r1+r2);
    }
*/
    /*void updateAll(){
        accumulatedX += getDx()*Math.cos(getDTheta())+getDy()*Math.sin(getDTheta());
        accumulatedY += getDy()*Math.cos(getDTheta())-getDx()*Math.sin(getDTheta());
        accumulatedTheta += getDTheta();
        relativeY += getDy();
        relativeX += getDx();
        left.update();
        right.update();
        center.update();
    }*/

