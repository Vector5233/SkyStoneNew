package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueFoundParkWall", group="Blue")

public class BlueFoundParkWall extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.strafeDistance(.8, 6.5);
        drive.sleepBetweenMotion();
        drive.driveDistance(.5, -28);
        drive.moveFoundation(drive.BLUE);
        telemetry.addLine("FoundationMoved");
        telemetry.update();
        drive.sleepBetweenMotion();
        drive.park(drive.BLUE, drive.WALL);
    }
}

