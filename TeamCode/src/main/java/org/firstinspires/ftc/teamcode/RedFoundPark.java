package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="RedFoundPark", group="TeamCode")

public class RedFoundPark extends LinearOpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft, rightRoller, leftRoller;
    Servo hookHrz, hookVrt, deliveryGrabber, deliveryRotation, camera, leftFoundation, blockSweeper;
    CRServo deliveryExtender;
    LinearOpMode opmode;
    ModernRoboticsI2cGyro gyro;
    SSDriveObject drive;

    public void initialize(){
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        rightRoller = hardwareMap.dcMotor.get("rightRoller");
        leftRoller = hardwareMap.dcMotor.get("leftRoller");

        hookHrz = hardwareMap.servo.get("hookHrz");
        hookVrt = hardwareMap.servo.get("hookVrt");

        deliveryGrabber = hardwareMap.servo.get("deliveryGrabber");
        deliveryRotation = hardwareMap.servo.get("deliveryRotation");

        leftFoundation = hardwareMap.servo.get("leftFoundation");

        deliveryExtender = hardwareMap.crservo.get("deliveryExtender");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");



        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        //change frontLeft into reverse
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        rightRoller.setDirection(DcMotor.Direction.REVERSE);
        leftRoller.setDirection(DcMotor.Direction.FORWARD);

        deliveryExtender.setDirection(CRServo.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new SSDriveObject(frontLeft, frontRight, backLeft, backRight, hookHrz, hookVrt, deliveryGrabber, leftFoundation, deliveryExtender, rightRoller, leftRoller, camera, leftFoundation, blockSweeper, gyro, this);

        gyro.calibrate();
        while (gyro.isCalibrating()) {
            ;
        }

    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.moveFoundation(drive.RED);
        drive.park(drive.RED, drive.FOUNDATION);
    }
}

