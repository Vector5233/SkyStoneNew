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

        drive.goToDetectPosition();
        Log.i("FINAL POSITION",drive.getFinalPosition());
        sleep(1700);
        int skystone = drive.detectStonesStatic(drive.BLUE);
        telemetry.addLine(drive.skystoneString(skystone));
        telemetry.update();
        //Log.i("STATIC DETECTION", String.format("Number of Stones Detected: %f",drive.numberOfStones));
        //Log.i("STATIC DETECTION","SkyStone Pos: " + drive.skystoneString(skystone));

        drive.collectSkyStone(drive.BLUE, skystone);
        drive.moveToFoundation(drive.BLUE);
        drive.deliverSkystone(drive.BLUE);
        idle();
        drive.moveFoundation(drive.BLUE);
        idle();
        drive.park(drive.BLUE, drive.BRIDGE);

    }
}

