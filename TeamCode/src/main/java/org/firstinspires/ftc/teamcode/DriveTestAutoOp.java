package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="DriveTestAutoOp", group="Test")

public class DriveTestAutoOp extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.strafeDistance(1, 20);
        sleep(2000);
        drive.strafeDistanceNoAccel(1,-20);
    }
}

