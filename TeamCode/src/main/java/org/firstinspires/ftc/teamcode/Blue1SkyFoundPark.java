package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue1SkyFoundPark", group="Blue")

public class Blue1SkyFoundPark extends LinearOpMode {
    SSDriveObject game;
    BaseDriveObject drive;

    public void initialize(){//lll
        game = new SSDriveObject(this, drive);

        game.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();
        game.collectSkyStone(game.BLUE, game.skystone);
        game.moveToFoundation(game.BLUE);

        game.moveFoundation(game.BLUE);
        idle();
        game.moveToSecondSS(game.BLUE,game.BRIDGE);
        game.collectSecondSkyStone(game.BLUE);

        game.park(game.BLUE, game.BRIDGE);
        Log.i("FINALTHETA",String.format("FinalTheta: \t%f\n", (drive.encoderArray.theta)));

    }
}