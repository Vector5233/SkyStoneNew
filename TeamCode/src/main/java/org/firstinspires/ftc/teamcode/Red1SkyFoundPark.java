package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red1SkyFoundPark", group="Red")

public class Red1SkyFoundPark extends LinearOpMode {
    SSDriveObject game;
    BaseDriveObject drive;

    public void initialize(){
        game = new SSDriveObject(this, drive);

        game.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();
        game.opModeTime.reset();
        game.collectSkyStone(game.RED,game.skystone);
        game.moveToFoundation(game.RED);
        idle();
        game.moveFoundation(game.RED);
        game.moveToSecondSS(game.RED, game.BRIDGE);
        game.collectSecondSkyStone(game.RED);
        game.park(game.RED, game.BRIDGE);
    }
}