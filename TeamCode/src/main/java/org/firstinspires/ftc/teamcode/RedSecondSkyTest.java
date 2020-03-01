package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RedSecondSkyTest", group="Red")

public class RedSecondSkyTest extends LinearOpMode {
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

        drive.collectSecondSkyStone(drive.RED);
    }
}