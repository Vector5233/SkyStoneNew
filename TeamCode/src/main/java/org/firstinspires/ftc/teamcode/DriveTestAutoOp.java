package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="DriveTestAutoOp", group="TeamCode")

public class DriveTestAutoOp extends LinearOpMode {
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


        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        //change frontLeft into reverse
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        rightRoller.setDirection(DcMotor.Direction.FORWARD);
        leftRoller.setDirection(DcMotor.Direction.REVERSE);

        deliveryExtender.setDirection(CRServo.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new SSDriveObject(frontLeft, frontRight, backLeft, backRight, hookHrz, hookVrt, deliveryGrabber, leftFoundation, deliveryExtender, rightRoller, leftRoller, camera, leftFoundation, blockSweeper, gyro, this);
        telemetry.addLine("gyro calibrate starts");
        telemetry.update();
        sleep(500);

        gyro.calibrate();


        telemetry.addLine("gyro calibrated");
        telemetry.update();
        sleep(500);

        while ((gyro.getIntegratedZValue() != 0) && opmode.opModeIsActive()) {
            continue;
        }
        telemetry.addLine("Init done");
        telemetry.update();
    }

    public void runOpMode() {
        initialize();
        waitForStart();

        drive.strafeDistance(1, 20);
        sleep(1000);
        drive.driveDistance(1, 20);

    }
}