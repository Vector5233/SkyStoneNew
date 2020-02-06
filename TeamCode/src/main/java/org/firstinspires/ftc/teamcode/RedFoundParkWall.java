package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RedFoundParkWall", group="Red")

public class RedFoundParkWall extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }

    public void runOpMode(){
        initialize();
        waitForStart();

        drive.strafeDistance(.8, -8);
        drive.sleepBetweenMotion();
        drive.driveDistance(.5, -28);
        drive.moveFoundation(drive.RED);
        telemetry.addLine("FoundationMoved");
        telemetry.update();
        drive.sleepBetweenMotion();
        drive.park(drive.RED, drive.WALL);
    }
}

