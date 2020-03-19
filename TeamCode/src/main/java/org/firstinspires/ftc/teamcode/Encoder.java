package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Encoder {
    DcMotor odometer;

    final double TICKS_PER_INCH = (8192) / (2 * 3.14159265358979323846264*1.1811);

    final boolean FORWARD = true;

    final boolean REVERSE = false;

    double oldPosition = 0; // position since last update

    double totalDistance = 0;  // total displacement since creation of encoder

    double accumulatedDisplacement = 0;


    public Encoder(DcMotor myMotor) {
        odometer = myMotor;
        odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void setEncoderDirection(boolean direction) {
        if (direction == FORWARD) {
            odometer.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction == REVERSE) {
            odometer.setDirection(DcMotor.Direction.REVERSE);
        }
    }




    double ticksToInches(int ticks) {
        return ticks / TICKS_PER_INCH;
    }

    double getPosition() {
        /** return position since last reset
         *
         */
        return ticksToInches(odometer.getCurrentPosition());
    }

    void reset() {
        totalDistance += getPosition();
        accumulatedDisplacement = 0;
        odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*void update() {
        accumulatedDisplacement += getDiff();
        oldPosition = getPosition();
    }*/

    double getDiff (){
        /** return change in position since last update
         *
         */
        return getPosition()-oldPosition;
    }
}




