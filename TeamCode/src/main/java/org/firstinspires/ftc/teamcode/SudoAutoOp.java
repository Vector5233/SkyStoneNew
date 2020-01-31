package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="SudoAutoOp", group="Red")

public class SudoAutoOp extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode() {
        initialize();
        waitForStart();

        //drive.;
    }
}

