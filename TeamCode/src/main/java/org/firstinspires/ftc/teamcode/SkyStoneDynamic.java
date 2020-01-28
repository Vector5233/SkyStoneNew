package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="SkyStoneDynamic", group="Test")

public class SkyStoneDynamic extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

//        int skystone = drive.detectStonesStatic(drive.BLUE);
//        Log.i("STATIC DETECTION","SkyStone Pos: " + skystoneString(skystone));

        drive.detectStonesDynamic();
        telemetry.addData("  SS left", "%.03f", drive.SS_leftPixel);
        telemetry.addData("  SS right", "%.03f", drive.SS_rightPixel);
        telemetry.update();
        sleep(1000);

        drive.getDisplacement();
        telemetry.addData("displacement", "%.03f", drive.displacement);
        telemetry.update();
        sleep(1000);

//        drive.collectSkyStone();

        //drive.moveToFoundation(drive.RED);
    }
    public String skystoneString(int skystone){
        switch(skystone) {
            case 0:
                return "left";
            case 1:
                return "center";
            case 2:
                return "right";
            default:
                return "oops";
        }
    }
}
