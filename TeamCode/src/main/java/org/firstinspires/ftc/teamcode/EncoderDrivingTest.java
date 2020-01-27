package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="EncoderDrivingTest", group = "Blue")

public class EncoderDrivingTest extends LinearOpMode {
    SSDriveObject drive;

    public void initialize() {
        drive = new SSDriveObject(this);
    }

    public void runOpMode() {
        initialize();
        waitForStart();

        drive.driveDistance(.3,30);
        telemetry.addLine("final deltaY read");
        telemetry.update();
        sleep(1000);
        drive.encoderArray.updateAll();
        drive.encoderArray.resetAll();
        getPositionTelemetry();
        sleep(2000);
        stop();
    }
    
    public void getPositionTelemetry() {
        telemetry.addData("X", drive.encoderArray.X);
        telemetry.addData("Y", drive.encoderArray.Y);
        telemetry.addData("theta", drive.encoderArray.theta*180/Math.PI);
        telemetry.update();

    }
    
}