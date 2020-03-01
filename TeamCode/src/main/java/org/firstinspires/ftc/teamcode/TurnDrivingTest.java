package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="TurnDrivingTest", group = "Blue")

public class TurnDrivingTest extends LinearOpMode {
    SSDriveObject drive;

    public void initialize() {
        drive = new SSDriveObject(this);
        drive.initialize();
        telemetry.addLine("initialized");
        telemetry.update();
    }

    public void runOpMode() {
        initialize();

        waitForStart();

        telemetry.addLine("start driving");
        telemetry.update();

//        drive.turnToDegree(.67, -90);
//        idle();
        drive.encoderArray.theta = 35;
        drive.turnToDegree(.67, -180);
    }

    public void getPositionTelemetry() {
        telemetry.addData("X", drive.encoderArray.X);
        telemetry.addData("Y", drive.encoderArray.Y);
        telemetry.addData("theta", drive.encoderArray.theta);
        telemetry.update();
    }
}