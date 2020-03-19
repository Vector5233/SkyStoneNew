package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueStonesNoFound", group="Blue")

public class BlueStonesNoFound extends LinearOpMode {
    SSDriveObject game;
    BaseDriveObject drive;

    public void initialize(){
        game = new SSDriveObject(this, drive);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();
        drive.opModeTime.reset();
        game.collectSkyStone(game.BLUE,game.skystone);
        /*game.moveToFoundation(game.RED);
        idle();
        game.moveFoundation(game.RED);*/
        game.moveToSecondSS(game.BLUE, game.BRIDGE);
        game.collectSecondSkyStone(game.BLUE);
        game.park(game.BLUE, game.BRIDGE);
    }
}