package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="SSTestAutoOp", group="Test")

public class SSTestAutoOp extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.detectReady();

        drive.detectStones();
        telemetry.addData("  SS left", "%.03f", drive.SS_leftPixel);
        telemetry.addData("  SS right", "%.03f", drive.SS_rightPixel);
        telemetry.update();
        sleep(1000);

        drive.getDisplacement();
        telemetry.addData("displacement", "%.03f", drive.displacement);
        telemetry.update();
        sleep(1000);

        drive.collectSkyStone();

        drive.moveToFoundation();
    }
}
