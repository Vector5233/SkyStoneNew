package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="EncoderDrivingTest", group = "Blue")

public class EncoderDrivingTest extends LinearOpMode {
    SSDriveObject drive;

    public void initialize() {
        drive = new SSDriveObject(this);
//        drive.initialize();
        drive.setDeliveryExtender(drive.IN);
        drive.setDeliveryRotation(drive.IN);
        drive.setDeliveryGrabber(drive.IN);
        telemetry.addLine("initialized");
        telemetry.update();
    }

    public void runOpMode() {
        initialize();

        waitForStart();

        telemetry.addLine("start driving");
        telemetry.update();
        drive.setDeliveryExtender(drive.OUT);
        sleep(750);
        drive.deliverSkystone(drive.RED);







        //        drive.encoderArray.readEncoderValue();


        /*Log.i("DELTATHETA",String.format("DeltaTheta: %f", (drive.encoderArray.getDeltaTheta())));

        drive.encoderArray.resetAll();
        Log.i("FINALTHETA",String.format("FinalTheta: %f", (drive.encoderArray.theta)));

        Log.i("FINAL POSITION",drive.getFinalPosition());*/



        telemetry.addLine("final delta read");

        drive.telemetryEncoderArray();
        sleep(1000);
        stop();
    }

    public void getPositionTelemetry() {
        telemetry.addData("X", drive.encoderArray.X);
        telemetry.addData("Y", drive.encoderArray.Y);
        telemetry.addData("theta", drive.encoderArray.theta);
        telemetry.update();
    }
}