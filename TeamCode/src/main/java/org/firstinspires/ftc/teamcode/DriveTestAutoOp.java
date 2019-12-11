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

        drive.driveDistanceOld(1, 20);
        telemetry.addLine("drive finished");
        telemetry.update();
        sleep(2000);
        drive.strafeDistanceOld(1, 20);
        telemetry.addLine("strafe finished");
        telemetry.update();
    }
}

