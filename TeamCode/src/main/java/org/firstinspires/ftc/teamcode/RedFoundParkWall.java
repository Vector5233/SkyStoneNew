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

        drive.driveDistance(.5, -29);
        idle();

        drive.strafeDistance(.5, -8);
        idle();

        drive.moveFoundation(drive.RED);
        telemetry.addLine("FoundationMoved");
        telemetry.update();
        idle();
        drive.turnToDegree(.3,-90);
        idle();
        drive.park(drive.RED, drive.WALL);
    }
}

