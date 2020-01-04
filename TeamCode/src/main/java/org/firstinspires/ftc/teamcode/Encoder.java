package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;


public class Encoder {
    DcMotor odometer;

    final double TICKS_PER_INCH = (383.6 * 2) / (2.4 * 3.14159265358979323846264);
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

    double getDisplacement() {
        return ticksToInches(odometer.getCurrentPosition());
    }

    void reset() {
        totalDisplacement += getDisplacement();
        odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void update() {
        oldPosition = getDisplacement();
    }

    double getDx (){
        return getDisplacement()-oldPosition;
    }




}




