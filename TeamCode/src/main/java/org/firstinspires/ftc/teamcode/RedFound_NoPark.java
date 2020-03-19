package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RedFound_NoPark", group="Red")

public class RedFound_NoPark extends LinearOpMode {
    SSDriveObject game;
    BaseDriveObject drive;

    public void initialize(){
        game = new SSDriveObject(this, drive);

        drive.initialize();
    }

    public void runOpMode(){
        initialize();
        waitForStart();

        drive.driveDistance(.5, -29);
        game.moveFoundation(game.RED);
    }
}