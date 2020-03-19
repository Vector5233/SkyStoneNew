package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RedStonesNoFound", group="Red")

public class RedStonesNoFound extends LinearOpMode {
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
        game.collectSkyStone(game.RED,game.skystone);
        /*game.moveToFoundation(game.RED);
        idle();
        game.moveFoundation(game.RED);*/
        game.moveToSecondSS(game.RED, game.BRIDGE);
        game.collectSecondSkyStone(game.RED);
        game.park(game.RED, game.BRIDGE);
    }
}
