package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="EncoderDrivingTest2", group = "Blue")
//@Disabled

public class EncoderDrivingTest2 extends LinearOpMode {
    SSDriveObject drive;

    public void initialize() {
        drive = new SSDriveObject(this);
        drive.initialize();
    }

    public void runOpMode() {
        initialize();
        /*drive.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/
        waitForStart();
        driveDistanceConstantComp(.8,70);

        stop();
    }
    
    public void driveDistanceConstantComp(double powerLimit, double distance) {
        double deltaY = 0;
        double powerMin = drive.POWER_MIN;
        double deltaTheta = 0;
        final double COMP = 0;



        drive.encoderArray.readEncoderValue();
        drive.encoderArray.updateAll();
        drive.encoderArray.resetAll();
        if (distance > 0) {
            if (powerLimit == .8) {
                distance -= 2.24;
            } else if (powerLimit ==.5) {
                distance -= .71;
            } else {
                distance -= 3.39;
            }
        } else if (distance < 0) {
            if (powerLimit == .8) {
                distance += 2.24;
            } else if (powerLimit ==.5) {
                distance += .71;
            } else {
                distance += 3.39;
            }
        }


        if (distance > 0) {
//            direction = FORWARD;
            while(opModeIsActive()) {
                drive.encoderArray.readEncoderValue();
                deltaY = drive.encoderArray.getDeltaY();
                deltaTheta = drive.encoderArray.getDeltaTheta();
                if (deltaY >= distance)
                    break;

                double drivePower = Math.max(.22,drive.calculatePowerStraight(powerLimit, distance, deltaY));
                drive.setSelectPowerAll(drivePower +COMP, drivePower - COMP, drivePower + COMP, drivePower - COMP);
                Log.i("POWER",String.format("Delta Y: %f\tPower: %f\t Compensation: %f\n", deltaY, drivePower,COMP ));


//                telemetryEncoderArray();
                telemetry.addData("deltaY: ", deltaY);
                telemetry.update();
//                telemetryWheelPower();
//                debugString.concat(String.format("Y: %f\tpower: %f\n", encoderArray.getDeltaY(), calculatePowerStraight(powerLimit,distance)));
            }
        } else if (distance < 0) {

            while(opModeIsActive()) {
                drive.encoderArray.readEncoderValue();
                deltaY = drive.encoderArray.getDeltaY();
                deltaTheta = drive.encoderArray.getDeltaTheta();
                if (deltaY <= distance)
                    break;

                double drivePower = -Math.max(.22,drive.calculatePowerStraight(powerLimit, distance, deltaY));
                drive.setSelectPowerAll(drivePower + COMP, drivePower - COMP, drivePower +COMP, drivePower - COMP);
                telemetry.addData("deltaY", drive.encoderArray.getDeltaY());
                Log.i("POWER",String.format("Delta Y: %f\tPower: %f\t Compensation: %f\n", deltaY, -drivePower,COMP ));
//                opmode.telemetry.update();
            }
        }
        drive.stopDriving();
        telemetry.addLine("motors stopped");
        telemetry.update();
        Log.i("OPMODETIME", String.format("driveDistance: \t%f\n",drive.opModeTime.milliseconds()));

//        System.out.println(debugString);
//        debugString="";
//        opmode.sleep(500);
    }

    public void getPositionTelemetry() {
        telemetry.addData("X", drive.encoderArray.X);
        telemetry.addData("Y", drive.encoderArray.Y);
        telemetry.addData("theta", drive.encoderArray.theta*180/Math.PI);
        telemetry.update();
    }
}
