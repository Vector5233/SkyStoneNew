package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red1SkyFoundPark", group="Red")

public class Red1SkyFoundPark extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();
        drive.opModeTime.reset();
        //Log.i("OPMODETIME", String.format("OpModeStart: \t%f\n",drive.opModeTime.milliseconds()));


        drive.goToDetectPosition();
        int skystone = drive.detectStonesStatic(drive.RED);
        telemetry.addLine(drive.skystoneString(skystone));
        telemetry.update();
        //Log.i("STATIC DETECTION", String.format("Number of Stones Detected: %f",drive.numberOfStones));
        //Log.i("STATIC DETECTION","SkyStone Pos: " + drive.skystoneString(skystone));
        drive.collectSkyStone(drive.RED,skystone);
        drive.moveToFoundation(drive.RED);

        drive.deliverSkystone(drive.RED);
        idle();
        drive.moveFoundation(drive.RED);
        sleep(1000);
        drive.encoderArray.readEncoderValue();
        drive.encoderArray.resetAll();
        Log.i("FINALTHETA",String.format("Theta: %f", drive.encoderArray.theta));
//        drive.park(drive.RED, drive.BRIDGE);
    }
}