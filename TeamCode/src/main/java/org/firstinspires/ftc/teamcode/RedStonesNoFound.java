package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RedStonesNoFound", group="Red")

public class RedStonesNoFound extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();
        drive.opModeTime.reset();
        drive.collectSkyStone(drive.RED,drive.skystone);
        /*drive.moveToFoundation(drive.RED);
        idle();
        drive.moveFoundation(drive.RED);*/
        drive.moveToSecondSS(drive.RED, drive.BRIDGE);
        drive.collectSecondSkyStone(drive.RED);
        drive.park(drive.RED, drive.BRIDGE);
    }
}
