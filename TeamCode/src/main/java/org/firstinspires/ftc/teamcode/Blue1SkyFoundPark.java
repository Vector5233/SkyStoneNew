package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue1SkyFoundPark", group="Blue")

public class Blue1SkyFoundPark extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){//lll
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();
        drive.collectSkyStone(drive.BLUE, drive.skystone);
        drive.moveToFoundation(drive.BLUE);

        drive.moveFoundation(drive.BLUE);
        idle();
        drive.moveToSecondSS(drive.BLUE,drive.BRIDGE);
        drive.collectSecondSkyStone(drive.BLUE);

        drive.park(drive.BLUE, drive.BRIDGE);
        Log.i("FINALTHETA",String.format("FinalTheta: \t%f\n", (drive.encoderArray.theta)));

    }
}