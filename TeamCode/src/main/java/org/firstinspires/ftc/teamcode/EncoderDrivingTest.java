package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="EncoderDrivingTest", group = "Blue")

public class EncoderDrivingTest extends LinearOpMode {
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

        drive.driveDistance(1,30);
        telemetry.addLine("final deltaY read");
        drive.encoderArray.readEncoderValue();
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
