package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RedFoundPark", group="Red")

public class RedFoundPark extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.driveDistance(1, -24.5);
        drive.moveFoundation(drive.RED);
        drive.park(drive.RED, drive.FOUNDATION);
    }
}

