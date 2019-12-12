package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="DriveTestNewAutoOp", group="Test")

public class DriveTestNewAutoOp extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.capServo.setPosition(0.8);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        //drive.driveDistance(1, 20);
        telemetry.addLine("new drive finished");
        telemetry.update();
        sleep(2000);
        drive.strafeDistance(1, 20);
        telemetry.addLine("new strafe finished");
        telemetry.update();


    }
}

