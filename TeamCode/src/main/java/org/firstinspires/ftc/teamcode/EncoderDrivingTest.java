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
        telemetry.addLine("initialized");
        telemetry.update();
    }

    public void runOpMode() {
        initialize();
/*
        drive.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
*/
        waitForStart();

        telemetry.addLine("start driving");
        telemetry.update();
        sleep(500);

        drive.turnDegree(1,115);
        sleep(500);
        drive.encoderArray.readEncoderValue();
        Log.i("DELTATHETA",String.format("DeltaTheta: %f", (drive.encoderArray.getDeltaTheta())));

        drive.turnToDegree(.67,90);
        sleep(500);
        drive.encoderArray.readEncoderValue();
        Log.i("DELTATHETA",String.format("DeltaTheta: %f", (drive.encoderArray.getDeltaTheta())));

        drive.encoderArray.resetAll();
        Log.i("FINALTHETA",String.format("FinalTheta: %f", (drive.encoderArray.theta)));

//        drive.encoderArray.resetAll();
        Log.i("FINAL POSITION",drive.getFinalPosition());

//        drive.turnDegree(.67,90-drive.encoderArray.theta);


//        drive.strafeDistance(.8,-30);
//        sleep(500);
//        drive.encoderArray.readEncoderValue();
//        drive.encoderArray.resetAll();
//        Log.i("FINAL POSITION",drive.getFinalPosition());

        telemetry.addLine("final delta read");

/*
        telemetry.addLine("initial delta");
        drive.telemetryEncoderArray();
        sleep(8000);
        drive.encoderArray.updateAll();
        drive.encoderArray.resetAll();
        drive.encoderArray.readEncoderValue();
        telemetry.addLine("final delta");
*/
        drive.telemetryEncoderArray();
        sleep(4000);
        stop();
    }

    public void getPositionTelemetry() {
        telemetry.addData("X", drive.encoderArray.X);
        telemetry.addData("Y", drive.encoderArray.Y);
        telemetry.addData("theta", drive.encoderArray.theta);
        telemetry.update();
    }
}