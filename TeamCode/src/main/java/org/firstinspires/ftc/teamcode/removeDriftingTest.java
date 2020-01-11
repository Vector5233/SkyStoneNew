package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class removeDriftingTest extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode() {
        initialize();
        waitForStart();
        sleep(5000);
        drive.strafeDistance(1,20);
        stop();
    }
}


