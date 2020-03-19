package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueStonesNoFound", group="Blue")

public class BlueStonesNoFound extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();
        drive.opModeTime.reset();
        drive.collectSkyStone(drive.BLUE,drive.skystone);
        /*drive.moveToFoundation(drive.RED);
        idle();
        drive.moveFoundation(drive.RED);*/
        drive.moveToSecondSS(drive.BLUE, drive.BRIDGE);
        drive.collectSecondSkyStone(drive.BLUE);
        drive.park(drive.BLUE, drive.BRIDGE);
    }
}