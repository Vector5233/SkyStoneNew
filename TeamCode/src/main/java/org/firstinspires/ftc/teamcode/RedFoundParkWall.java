package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RedFoundParkWall", group="Red")

public class RedFoundParkWall extends LinearOpMode {
    SSDriveObject game;
    BaseDriveObject drive;

    public void initialize(){
        game = new SSDriveObject(this, drive);

        drive.initialize();
    }

    public void runOpMode(){
        initialize();
        waitForStart();

        drive.driveDistance(.5, -30);
        idle();

        drive.strafeDistance(.5, -8);
        idle();

        game.moveFoundation(game.RED);
        telemetry.addLine("FoundationMoved");
        telemetry.update();
        idle();
        drive.turnToDegree(.3,-90);
        idle();
        game.park(game.RED, game.WALL);
    }
}

