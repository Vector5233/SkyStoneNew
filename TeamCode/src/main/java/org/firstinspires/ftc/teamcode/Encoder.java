package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;


public class Encoder {
    DcMotor odometer;

    final double TICKS_PER_INCH = (8192) / (2 * 3.14159265358979323846264*1.1811);
    // need to be tested.

    double oldPosition = 0;

    double totalDisplacement = 0;

    public Encoder(DcMotor myMotor) {
        odometer = myMotor;
        odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    double ticksToInches(int ticks) {
        return ticks / TICKS_PER_INCH;
    }

    double getPosition() {
        return ticksToInches(odometer.getCurrentPosition());
    }

    void reset() {
        totalDisplacement += getPosition();
        odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void update() {
        oldPosition = getPosition();
    }

    double getDisplacement (){
        return getPosition()-oldPosition;
    }




}




