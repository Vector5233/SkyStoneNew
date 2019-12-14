package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BluePark", group="Blue")

public class BluePark extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.setBlockSweeper(false);
        drive.setCameraServo(1);
        drive.setHookVrt(1);
        drive.setHookHrz(0);

        telemetry.addLine("initialized");
        telemetry.update();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.setFoundationLeft(true);
    }
}

