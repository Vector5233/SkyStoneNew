package org.firstinspires.ftc.teamcode;

import android.util.Log;
import android.widget.BaseExpandableListAdapter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="SkyStoneDynamic", group="Test")
@Disabled

public class SkyStoneDynamic extends LinearOpMode {
    SSDriveObject game;
    BaseDriveObject drive;

    public void initialize(){
        game = new SSDriveObject(this, drive);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

//        int skystone = game.detectStonesStatic(game.BLUE);
//        Log.i("STATIC DETECTION","SkyStone Pos: " + skystoneString(skystone));

        //game.detectStonesDynamic();
        telemetry.addData("  SS left", "%.03f", game.SS_leftPixel);
        telemetry.addData("  SS right", "%.03f", game.SS_rightPixel);
        telemetry.update();
        Log.i("DYNAMIC DETECTION", String.format("LeftPixel: %f\tRightPixel: %f",game.SS_leftPixel,game.SS_rightPixel));
        sleep(500);

        game.getDisplacement();
        telemetry.addData("displacement", "%.03f", game.displacement);
        telemetry.update();
        Log.i("DYNAMIC DETECTION", String.format("Displacement: %f",game.displacement));
        sleep(500);

//        game.collectSkyStone();

        //game.moveToFoundation(game.RED);
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
