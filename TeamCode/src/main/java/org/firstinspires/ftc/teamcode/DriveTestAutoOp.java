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

        drive.turnDegree(.67,90);
        sleep(1000);
        drive.turnDegree(.67, -90);
    }
}

