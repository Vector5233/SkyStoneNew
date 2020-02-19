package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RedFound_NoPark", group="Red")

public class RedFound_NoPark extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }

    public void runOpMode(){
        initialize();
        waitForStart();

        drive.strafeDistance(.8, -8);
        idle();
        drive.driveDistance(.5, -28);
        drive.moveFoundation(drive.RED);
    }
}