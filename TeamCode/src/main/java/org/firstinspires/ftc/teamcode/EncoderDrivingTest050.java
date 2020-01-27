package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="EncoderDrivingTest050", group = "Blue")

public class EncoderDrivingTest050 extends LinearOpMode {
    SSDriveObject drive;
    
    public void initialize() {
        drive = new SSDriveObject(this);

    }

    public void runOpMode() {
        initialize();
/*
        drive.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
*/
        waitForStart();

        /*telemetry.addLine("start driving");
        drive.turnDegree(.67,90);
        telemetry.addLine("final delta read");
        drive.encoderArray.readEncoderValue();
        Log.i("FINAL DISPLACEMENT",String.format("DeltaTheta: %f", drive.encoderArray.getDeltaTheta()));

         */

        drive.driveDistance(0.5,20);
        drive.encoderArray.readEncoderValue();
        telemetry.addData("final distance reached: ", drive.encoderArray.getDeltaY());
        telemetry.update();
        sleep(4000);
        Log.i("FINAL DISPLACEMENT",String.format("DeltaY: %f", drive.encoderArray.getDeltaY()));
/*
        telemetry.addLine("initial delta");
        drive.telemetryEncoderArray();
        sleep(8000);
        drive.encoderArray.updateAll();
        drive.encoderArray.resetAll();
        drive.encoderArray.readEncoderValue();
        telemetry.addLine("final delta");
*/
        drive.telemetryEncoderArray();
        sleep(4000);
        stop();
    }

    public void getPositionTelemetry() {
        telemetry.addData("X", drive.encoderArray.X);
        telemetry.addData("Y", drive.encoderArray.Y);
        telemetry.addData("theta", drive.encoderArray.theta*180/Math.PI);
        telemetry.update();
    }
}
