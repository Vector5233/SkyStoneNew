package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueFoundParkWall", group="Blue")

public class BlueFoundParkWall extends LinearOpMode {
    SSDriveObject game;
    BaseDriveObject drive;

    public void initialize(){
        game = new SSDriveObject(this, drive);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.strafeDistance(.5, 8);
        idle();
        drive.driveDistance(.5, -35);
        idle();
        game.moveFoundation(game.BLUE);
        telemetry.addLine("FoundationMoved");
        telemetry.update();
        idle();
        drive.turnToDegree(.3,90);
        idle();
        game.park(game.BLUE, game.WALL);
    }
}

