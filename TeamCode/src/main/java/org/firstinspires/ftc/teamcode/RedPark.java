package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RedPark", group="Red")

public class RedPark extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.park(drive.RED, drive.NORMAL);
    }
}

