package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="TurnDrivingTest", group = "Blue")
@Disabled

public class TurnDrivingTest extends LinearOpMode {
    SSDriveObject game;
    BaseDriveObject drive;

    public void initialize() {
        game = new SSDriveObject(this, drive);
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