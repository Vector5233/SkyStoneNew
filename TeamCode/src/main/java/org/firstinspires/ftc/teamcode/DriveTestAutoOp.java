package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="DriveTestAutoOp", group="Test")

public class DriveTestAutoOp extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.capServo.setPosition(0.8);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.strafeDistanceNoAccel(1,20);
        sleep(5500);
        drive.driveDistance(1, 20);
    }
}

