package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeoutException;


public class BaseDriveObject extends Object{
    Servo hookHrz, hookVrt, deliveryGrabber, deliveryRotation, cameraServo, leftFoundation, rightFoundation, blockSweeper, capServo, deliveryExtender;
    DcMotor frontRight, frontLeft, backRight, backLeft, rightRoller, leftRoller;
    LinearOpMode opmode;
    Encoder myLeft, myRight, myCenter;
    EncoderArray encoderArray;
    ElapsedTime elapsedTime;
    OpenCvCamera webcam;

    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";

    final double ROBOT_RADIUS = 13.5;
    final double TICKS_PER_INCH_STRAIGHT = (383.6*2) / (4 * 3.14159265358979323846264);
    final double TICKS_PER_INCH_TURN = TICKS_PER_INCH_STRAIGHT;
    final double TICKS_PER_INCH_STRAFE = (TICKS_PER_INCH_STRAIGHT)*1.15*(20.0/17.0);
    final double TICKS_PER_DEGREE = (3.14159 / 180) *  ROBOT_RADIUS * TICKS_PER_INCH_TURN;
    final double TOLERANCE = 3;  // in degrees


    final boolean IN = true;
    final boolean OUT = false;

    final boolean LEFT = true;
    final boolean RIGHT = false;

    final boolean FDOWN = true;
    final boolean FUP = false;

    final double r1 = 5.37;
    final double r2 = 5.85;
    final double r3 = 3.22;

    final double POWER_MIN = 0.3;
    final double POWER_MAX = .8;

    final double CAP_INIT_POS = .725;
    final double CAM_INIT_POS = .4;


    double FRpower = 1;
    double FLpower = 1;
    double BRpower = 1;
    double BLpower = 1;

    final double DRIVE_COMP_CONST_LEFT = 0.02;
    final double DRIVE_COMP_CONST_RIGHT = 0.02;

    final double FOUND_LEFT_INIT = 0;
    final double FOUND_RIGHT_INIT = 0.5;

    final double FOUND_LEFT_LAUNCH = 0.5;
    final double FOUND_RIGHT_LAUNCH = 0;

    final double ROLLER_POWER = .7;

    double numberOfStones;

    double SS_leftPixel;
    double SS_rightPixel;

    double displacement;
    double secondDisplacement = displacement - 24;

    boolean isVirtual = false;

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    ElapsedTime opModeTime = new ElapsedTime();

    public BaseDriveObject(LinearOpMode parent){
        opmode = parent;

        /*frontRight = opmode.hardwareMap.dcMotor.get("frontRight");
        frontLeft = opmode.hardwareMap.dcMotor.get("frontLeft");
        backRight = opmode.hardwareMap.dcMotor.get("backRight");
        backLeft = opmode.hardwareMap.dcMotor.get("backLeft");

        rightRoller = opmode.hardwareMap.dcMotor.get("rightRoller");
        leftRoller = opmode.hardwareMap.dcMotor.get("leftRoller");

        hookHrz = opmode.hardwareMap.servo.get("hookHrz");
        hookVrt = opmode.hardwareMap.servo.get("hookVrt");

        deliveryGrabber = opmode.hardwareMap.servo.get("deliveryGrabber");
        deliveryRotation = opmode.hardwareMap.servo.get("deliveryRotation");

        leftFoundation = opmode.hardwareMap.servo.get("leftFoundation");
        rightFoundation = opmode.hardwareMap.servo.get("rightFoundation");

        blockSweeper = opmode.hardwareMap.servo.get("blockSweeper");
        capServo = opmode.hardwareMap.servo.get("capServo");
        cameraServo = opmode.hardwareMap.servo.get("cameraServo");

        deliveryExtender = opmode.hardwareMap.servo.get("deliveryExtender");*/

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        /*rightRoller.setDirection(DcMotor.Direction.FORWARD);
        leftRoller.setDirection(DcMotor.Direction.REVERSE);

        deliveryExtender.setDirection(Servo.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*myLeft = new Encoder(frontLeft);
        myRight = new Encoder(frontRight);
        myCenter = new Encoder(backRight);
        encoderArray = new EncoderArray(myLeft, myRight, myCenter, r1, r2, r3);*/




    }

    public void initialize(){
        capServo.setPosition(CAP_INIT_POS);
        setFoundation(FUP);
        setDeliveryExtender(IN);
        setBlockSweeper(OUT);
        setDeliveryGrabber(OUT);
        setCameraServo(CAM_INIT_POS);
        setDeliveryRotation(IN);
        setBlockSweeper(IN);

        encoderArray.resetAll();

        opmode.telemetry.addLine("initialized");
        opmode.telemetry.update();
    }


    public void driveDistanceWithTime(double powerLimit, double distance, double time){
        opModeTime.reset();
        double deltaY = 0;
        double powerMin = POWER_MIN;
        double deltaTheta = 0;

        encoderArray.readEncoderValue();

        encoderArray.resetAll();
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
            while(opmode.opModeIsActive() && opModeTime.milliseconds()<=time) {

                encoderArray.readEncoderValue();
                deltaY = encoderArray.getDeltaY();
                deltaTheta = encoderArray.getDeltaTheta();
                if (deltaY >= distance)
                    break;

                double drivePower = Math.max(.22,calculatePowerStraight(powerLimit, distance, deltaY));
                setSelectPowerAll(drivePower - DRIVE_COMP_CONST_LEFT*deltaTheta, drivePower + DRIVE_COMP_CONST_LEFT*deltaTheta, drivePower -DRIVE_COMP_CONST_LEFT*deltaTheta, drivePower + DRIVE_COMP_CONST_LEFT*deltaTheta);
//                setDrivePowerAll(drivePower);
                Log.i("POWER",String.format("Delta Theta: %f\tDelta Y: %f\tPower: %f\t Compensation: %f\n", deltaTheta,deltaY, drivePower,DRIVE_COMP_CONST_LEFT*deltaTheta ));
                Log.i("INDIVIDUAL ENCODERS",String.format("Left Encoder: %f\tRight Encoder: %f\tCenter Encoder: %f\n", encoderArray.getLeftPosition,encoderArray.getRightPosition, encoderArray.getCenterPosition));


//                telemetryEncoderArray();
                opmode.telemetry.addData("deltaY: ", deltaY);
                opmode.telemetry.update();

//                telemetryWheelPower();
//                debugString.concat(String.format("Y: %f\tpower: %f\n", encoderArray.getDeltaY(), calculatePowerStraight(powerLimit,distance)));
            }
        } else if (distance < 0) {

            while(opmode.opModeIsActive() && opModeTime.milliseconds()<=time) {
                encoderArray.readEncoderValue();
                deltaY = encoderArray.getDeltaY();
                deltaTheta = encoderArray.getDeltaTheta();
                if (deltaY <= distance)
                    break;

                double drivePower = -Math.max(.22,calculatePowerStraight(powerLimit, distance, deltaY));
                setSelectPowerAll(drivePower - DRIVE_COMP_CONST_LEFT*deltaTheta, drivePower + DRIVE_COMP_CONST_LEFT*deltaTheta, drivePower -DRIVE_COMP_CONST_LEFT*deltaTheta, drivePower + DRIVE_COMP_CONST_LEFT*deltaTheta);
//                opmode.telemetry.addData("deltaY", encoderArray.getDeltaY());
//                setDrivePowerAll(drivePower);
                Log.i("POWER",String.format("Delta Theta: %f\tDelta Y: %f\tPower: %f\t Compensation: %f\n", deltaTheta,deltaY, drivePower,DRIVE_COMP_CONST_LEFT*deltaTheta ));
                Log.i("INDIVIDUAL ENCODERS",String.format("Left Encoder: %f\tRight Encoder: %f\tCenter Encoder: %f\n", encoderArray.getLeftPosition,encoderArray.getRightPosition, encoderArray.getCenterPosition));

//                opmode.telemetry.update();
            }
        }
        stopDriving();
        opmode.telemetry.addLine("motors stopped");
        opmode.telemetry.update();
        //Log.i("OPMODETIME", String.format("driveDistance: \t%f\n",opModeTime.milliseconds()));

//        System.out.println(debugString);
//        debugString="";
//        opmode.sleep(500);
        encoderArray.readEncoderValue();

        encoderArray.resetAll();
    }

    //detection



    //drive chassis motor

    public double calculatePowerStraight(double powerLimit, double distance, double deltaY) {
        distance = Math.abs(distance);
        deltaY = Math.abs(deltaY);
        if (distance != 0) {
//            return ((0.44 - 4 * powerLimit) / (distance * distance)) * deltaY * deltaY + ((4 * powerLimit - 0.66) / distance) * deltaY + POWER_MIN;
            return ((-16 * (powerLimit-POWER_MIN)) / (distance * distance * distance * distance)) * (deltaY - distance / 2) * (deltaY - distance / 2) * (deltaY - distance / 2) * (deltaY - distance / 2) + powerLimit;
        } else {
            return 0;
        }
    }

    public double calculatePowerStrafe(double powerLimit, double distance, double deltaX){
        distance = Math.abs(distance);
        deltaX = Math.abs(deltaX);

        if(distance != 0){
            return ((-16 * (powerLimit-POWER_MIN)) / (distance * distance * distance * distance)) * (deltaX - distance / 2) * (deltaX - distance / 2) * (deltaX - distance / 2) * (deltaX - distance / 2) + powerLimit;
//            return ((0.44 - 4 * powerLimit) / (distance * distance)) * deltaX * deltaX + ((4 * powerLimit - 0.66) / distance) * deltaX + POWER_MIN;
        } else {
            return 0;
        }
    }

    public double calculatePowerTurn(double powerLimit, double degrees, double deltaTheta) {
        degrees = Math.abs(degrees);
        deltaTheta = Math.abs(deltaTheta);
        if (degrees != 0) {
            return ((0.44 - 4 * powerLimit) / (degrees * degrees)) * deltaTheta * deltaTheta + ((4 * powerLimit - 0.66) / degrees) * deltaTheta + POWER_MIN;
        } else {
            return 0;
        }
    }

    public void driveDistance(double powerLimit, double distance) {
        double deltaY = 0;
        double powerMin = POWER_MIN;
        double deltaTheta = 0;

        encoderArray.readEncoderValue();

        encoderArray.resetAll();
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
            while(opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaY = encoderArray.getDeltaY();
                deltaTheta = encoderArray.getDeltaTheta();
                if (deltaY >= distance)
                    break;

                double drivePower = Math.max(.22,calculatePowerStraight(powerLimit, distance, deltaY));
                setSelectPowerAll(drivePower - DRIVE_COMP_CONST_LEFT*deltaTheta, drivePower + DRIVE_COMP_CONST_LEFT*deltaTheta, drivePower -DRIVE_COMP_CONST_LEFT*deltaTheta, drivePower + DRIVE_COMP_CONST_LEFT*deltaTheta);
//                setDrivePowerAll(drivePower);
                Log.i("POWER",String.format("Delta Theta: %f\tDelta Y: %f\tPower: %f\t Compensation: %f\n", deltaTheta,deltaY, drivePower,DRIVE_COMP_CONST_LEFT*deltaTheta ));
                Log.i("INDIVIDUAL ENCODERS",String.format("Left Encoder: %f\tRight Encoder: %f\tCenter Encoder: %f\n", encoderArray.getLeftPosition,encoderArray.getRightPosition, encoderArray.getCenterPosition));


//                telemetryEncoderArray();
                opmode.telemetry.addData("deltaY: ", deltaY);
                opmode.telemetry.update();
//                telemetryWheelPower();
//                debugString.concat(String.format("Y: %f\tpower: %f\n", encoderArray.getDeltaY(), calculatePowerStraight(powerLimit,distance)));
            }
        } else if (distance < 0) {

            while(opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaY = encoderArray.getDeltaY();
                deltaTheta = encoderArray.getDeltaTheta();
                if (deltaY <= distance)
                    break;

                double drivePower = -Math.max(.22,calculatePowerStraight(powerLimit, distance, deltaY));
                setSelectPowerAll(drivePower - DRIVE_COMP_CONST_LEFT*deltaTheta, drivePower + DRIVE_COMP_CONST_LEFT*deltaTheta, drivePower -DRIVE_COMP_CONST_LEFT*deltaTheta, drivePower + DRIVE_COMP_CONST_LEFT*deltaTheta);
//                opmode.telemetry.addData("deltaY", encoderArray.getDeltaY());
//                setDrivePowerAll(drivePower);
                Log.i("POWER",String.format("Delta Theta: %f\tDelta Y: %f\tPower: %f\t Compensation: %f\n", deltaTheta,deltaY, drivePower,DRIVE_COMP_CONST_LEFT*deltaTheta ));
                Log.i("INDIVIDUAL ENCODERS",String.format("Left Encoder: %f\tRight Encoder: %f\tCenter Encoder: %f\n", encoderArray.getLeftPosition,encoderArray.getRightPosition, encoderArray.getCenterPosition));

//                opmode.telemetry.update();
            }
        }
        stopDriving();
        opmode.telemetry.addLine("motors stopped");
        opmode.telemetry.update();
        //Log.i("OPMODETIME", String.format("driveDistance: \t%f\n",opModeTime.milliseconds()));

//        System.out.println(debugString);
//        debugString="";
//        opmode.sleep(500);
        encoderArray.readEncoderValue();

        encoderArray.resetAll();
    }

    final double STRAFE_RIGHT_COMP_CONST = 0.03;
    final double STRAFE_LEFT_COMP_CONST = 0;
    public void strafeDistance(double powerLimit, double distance) {
        double deltaX = 0;
        double deltaTheta = 0;
        double strafePower;

        if (distance > 0) {
            // specific distance compensations for various power levels.  Should be separate method!
            if (powerLimit == POWER_MAX) {
                distance -= .23;
            } else if (powerLimit ==.5) {
                distance -= .24;
            } else {
                distance -= .68;
            }
        } else if (distance < 0) {
            if (powerLimit == POWER_MAX) {
                distance += .23;
            } else if (powerLimit ==.5) {
                distance += .24;
            } else {
                distance += .68;
            }
        }

        encoderArray.readEncoderValue();

        encoderArray.resetAll();

        if (distance > 0) {   //strafe right
            while((deltaX <= distance) && opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaX = encoderArray.getDeltaX();
                deltaTheta = encoderArray.getDeltaTheta();
                strafePower = Math.max(.22,calculatePowerStrafe(powerLimit, distance, deltaX));
//                setStrafePowerAll(strafePower);
                // manual compensation to account for weight imbalance.  Better if done based on theta
                setSelectPowerAll(-strafePower +STRAFE_RIGHT_COMP_CONST, strafePower - STRAFE_RIGHT_COMP_CONST, strafePower,-strafePower);
                Log.i("INDIVIDUAL ENCODERS",String.format("Left Encoder: %f\tRight Encoder: %f\tCenter Encoder: %f\n", encoderArray.getLeftPosition,encoderArray.getRightPosition, encoderArray.getCenterPosition));

                Log.i("POWER",String.format("Delta Theta: %f\tDelta X: %f\tPower: %f\t Compensation: %f\n", deltaTheta, deltaX, strafePower,.22*deltaTheta ));
//                telemetryEncoderArray();
//                opmode.telemetry.addData("deltaX: ", encoderArray.getDeltaX());
//                telemetryWheelPower();
            }
        } else if (distance < 0) {  //strafe left

            while((deltaX >= distance) && opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaX = encoderArray.getDeltaX();
                deltaTheta = encoderArray.getDeltaTheta();
                strafePower = Math.max(.22,calculatePowerStrafe(powerLimit, distance, deltaX));
//                setStrafePowerAll(-strafePower);
                setSelectPowerAll(strafePower + STRAFE_RIGHT_COMP_CONST, -strafePower - STRAFE_LEFT_COMP_CONST, -strafePower, strafePower);
                Log.i("INDIVIDUAL ENCODERS",String.format("Left Encoder: %f\tRight Encoder: %f\tCenter Encoder: %f\n", encoderArray.getLeftPosition,encoderArray.getRightPosition, encoderArray.getCenterPosition));
                Log.i("POWER",String.format("Delta Theta: %f\tDelta X: %f\tPower: %f\t Compensation: %f\n", deltaTheta, deltaX, strafePower,.22*deltaTheta ));//                opmode.telemetry.addData("deltaY", deltaX);
//                opmode.telemetry.update();
            }
        }
        stopDriving();
        encoderArray.trashStrafeAngleBeforeReset();

        opmode.telemetry.addLine("motors stopped");
        opmode.telemetry.update();
        encoderArray.readEncoderValue();
        Log.i("INDIVIDUAL ENCODERS",String.format("Left Encoder: %f\tRight Encoder: %f\tCenter Encoder: %f\n", encoderArray.getLeftPosition,encoderArray.getRightPosition, encoderArray.getCenterPosition));
        Log.i("STRAFE",String.format("Delta Theta: %f\tDelta X: %f\tDelta Y: %f\n", encoderArray.getDeltaTheta(),encoderArray.getDeltaX(),encoderArray.getDeltaY()));
        //Log.i("OPMODETIME", String.format("strafeDistance: \t%f\n",opModeTime.milliseconds()));
//        opmode.sleep(500);

        encoderArray.readEncoderValue();

        encoderArray.resetAll();
    }

    public double degreesToRadians(double degrees){
        return degrees*=Math.PI/180;
    }

    public void turnArc(boolean direction, double power, double degrees) {

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        encoderArray.readEncoderValue();

        encoderArray.resetAll();

        double radians = degreesToRadians(degrees);
        double deltaTheta = 0;
/*
       There is no need to enter a negative power. All functionality is based as if the robot was moving around a circle.
       Starting at east when going LEFT, west when going RIGHT.
*/
        power = Math.abs(power);


        if (direction == LEFT) {
            if (radians > 0) {
                while ((Math.abs(deltaTheta) <= Math.abs(radians)) && opmode.opModeIsActive()) {
                    encoderArray.readEncoderValue();
                    deltaTheta = encoderArray.getDeltaThetaRad();
                    setArcPowerAll(direction, power);
                    Log.i("POWER",String.format("Delta Theta: %f\tPower: %f\n", deltaTheta, power));
                }
            } else if (radians < 0) {
                while ((Math.abs(deltaTheta) <= Math.abs(radians)) && opmode.opModeIsActive()) {
                    encoderArray.readEncoderValue();
                    deltaTheta = encoderArray.getDeltaThetaRad();
                    setArcPowerAll(direction, -power);
                    Log.i("POWER",String.format("Delta Theta: %f\tPower: %f\n", deltaTheta, -power));
                }
            }
        } else if (direction == RIGHT) {
            if (radians > 0) {
                while ((Math.abs(deltaTheta) <= Math.abs(radians)) && opmode.opModeIsActive()) {
                    encoderArray.readEncoderValue();
                    deltaTheta = encoderArray.getDeltaThetaRad();
                    setArcPowerAll(direction, -power);
                    Log.i("POWER",String.format("Delta Theta: %f\tPower: %f\n", deltaTheta*180/Math.PI, -power));
                }
            } else if (radians < 0) {
                while ((Math.abs(deltaTheta) <= Math.abs(radians)) && opmode.opModeIsActive()) {
                    encoderArray.readEncoderValue();
                    deltaTheta = encoderArray.getDeltaThetaRad();
                    setArcPowerAll(direction, power);
                    Log.i("POWER",String.format("Delta Theta: %f\tPower: %f\n", deltaTheta*180/Math.PI, power));
                }
            }
        }
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopDriving();

        //Log.i("OPMODETIME", String.format("turnArc: \t%f\n",opModeTime.milliseconds()));
        encoderArray.readEncoderValue();

        encoderArray.resetAll();
    }



    public void turnToDegree(double powerLimit, double targetDegrees) {
        //This needs work
        encoderArray.readEncoderValue();

        encoderArray.resetAll();

        //Log.i("TOTALTHETA", String.format("theta: \t%f\n",encoderArray.theta));

        double initialTheta = encoderArray.theta;
        double deltaTheta = targetDegrees - initialTheta;
        //Log.i("RELATIVETHETA",String.format("theta: \t%f\n",deltaTheta));

        if (Math.abs(deltaTheta) < Math.abs(55)){
            opmode.telemetry.addLine("Power reduced on turnToDegree");
            opmode.telemetry.update();
            Log.i("TURNTODEGREE","Power reduced.");
            powerLimit = .4;
        } else if (Math.abs(deltaTheta) < Math.abs(45)) {
            opmode.telemetry.addLine("Power reduced on turnToDegree");
            opmode.telemetry.update();
            Log.i("TURNTODEGREE","Power reduced.");
            powerLimit = .31;
        }

        turnDegree(powerLimit, deltaTheta);
        stopDriving();

        //Log.i("OPMODETIME", String.format("turnDegree: \t%f\n",opModeTime.milliseconds()));
//        getAngleTelemetry("TURN END");
    }

    public void turnDegree(double powerLimit, double degrees) {
        // distance in inches
        //conjecture instead of moving 12", wheels will go 12"*cos(45)= 8.5"
        encoderArray.readEncoderValue();

        encoderArray.resetAll();

        double deltaTheta = 0;

        if (degrees > 0) {
            // positive (CCW) turn => left motor goes in (-) direction
            while ((deltaTheta <= degrees) && opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaTheta = encoderArray.getDeltaTheta();
                double turnPower = Math.max(.17,calculatePowerTurn(powerLimit, degrees, deltaTheta));
                setTurnPowerAll(turnPower);
                Log.i("POWER",String.format("Delta Theta: %f\tPower: %f\n", deltaTheta, turnPower));

            }
        } else if (degrees < 0) {
            while ((deltaTheta >= degrees) && opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaTheta = encoderArray.getDeltaTheta();
                double turnPower = Math.max(.17,calculatePowerTurn(powerLimit, degrees, deltaTheta));
                setTurnPowerAll(-turnPower);
                Log.i("POWER",String.format("Delta Theta: %f\tPower: %f\n", deltaTheta, -turnPower));
//                opmode.telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
//                opmode.telemetry.update();
            }
        }

        stopDriving();
        //Log.i("OPMODETIME", String.format("turnDegree: \t%f\n",opModeTime.milliseconds()));
//        getAngleTelemetry("TURN END");
        encoderArray.readEncoderValue();

        encoderArray.resetAll();
    }


    //set chassis motors

    public void setModeAll(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void stopDriving() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    public void setDrivePowerAll(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    public void setStrafePowerAll(double power) {
        frontLeft.setPower(FLpower*power);
        frontRight.setPower(-FRpower*power);
        backLeft.setPower(-BLpower*power);
        backRight.setPower(BRpower*power);
    }

    public void setTurnPowerAll(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    public void setArcPowerAll(boolean direction, double power) {
        if (direction == LEFT) {
            frontLeft.setPower(0);
            frontRight.setPower(power);
            backLeft.setPower(0);
            backRight.setPower(power);
        } else if (direction == RIGHT) {
            frontLeft.setPower(power);
            frontRight.setPower(0);
            backLeft.setPower(power);
            backRight.setPower(0);
        }
    }

    public void setSelectPowerAll(double FRpower, double FLpower, double BRpower, double BLpower){
        frontRight.setPower(FRpower);
        frontLeft.setPower(FLpower);
        backRight.setPower(BRpower);
        backLeft.setPower(BLpower);
    }

    //set other motors
    public void stopRollerMotors () {
        leftRoller.setPower(0.0);
        rightRoller.setPower(0.0);
        //Log.i("OPMODETIME", String.format("setRollerMotors: \t%f\n",opModeTime.milliseconds()));
    }

    public void setRollerMotors (boolean direction, double power) {
        //direction true = outward
        //direction false = inward
       /*rollerTimeout = new ElapsedTime();
       final int ROLLER_OPMODETIMEOUT = time;*/

        if(direction == OUT){
            leftRoller.setDirection(DcMotorSimple.Direction.FORWARD);
            rightRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if(direction == IN){
            leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        //Log.i("OPMODETIME", String.format("setRollerMotors: \t%f\n",opModeTime.milliseconds()));

        leftRoller.setPower(ROLLER_POWER);
        rightRoller.setPower(ROLLER_POWER);



       /*while ((rightRoller.isBusy() || leftRoller.isBusy()) && opmode.opModeIsActive()){
           if (rollerTimeout.milliseconds() > ROLLER_OPMODETIMEOUT)
               break;
       }*/
    }

    public void setCameraServo (double position) {
        cameraServo.setPosition(position);
    }

    public void setFoundation (boolean position) {
        //launch true = grabber down
        //launch false = grabber up
        if (position == FDOWN) {
            leftFoundation.setPosition(FOUND_LEFT_LAUNCH);
            rightFoundation.setPosition(FOUND_RIGHT_LAUNCH);
        } else if (position == FUP) {
            leftFoundation.setPosition(FOUND_LEFT_INIT);
            rightFoundation.setPosition(FOUND_RIGHT_INIT);
        }
    }



    public void setBlockSweeper (boolean state) {
        //kick true = blockSweeper up
        //kick false = blockSweeper down

        if (state == OUT) {
            blockSweeper.setPosition(0.8);

        } else if (state == IN){
            blockSweeper.setPosition(.2);

        }
        //Log.i("OPMODETIME", String.format("setBlockSweeper: \t%f\n",opModeTime.milliseconds()));


    }

    public void setDeliveryGrabber (boolean grab) {
        if(grab == IN){
            deliveryGrabber.setPosition(.5);
        } else if (grab == OUT){
            deliveryGrabber.setPosition(0);
        }
    }

    public void setDeliveryExtender (boolean direction) {
        //true = in
        //false = out
        if (direction == IN)
            deliveryExtender.setPosition(1);
        else if (direction == OUT)
            deliveryExtender.setPosition(0);
    }

    public void setDeliveryRotation (boolean rotate) {


        if (rotate == OUT) {
            deliveryRotation.setPosition(0.5);
        } else if (rotate == IN) {
            deliveryRotation.setPosition(0);
        }
    }

    //telemetry

    public void telemetryEncoderArray (){
        opmode.telemetry.addData("deltaX", encoderArray.getDeltaX());
        opmode.telemetry.addData("deltaY", encoderArray.getDeltaY());
        opmode.telemetry.addData("deltaTheta(degrees)", encoderArray.getDeltaTheta());
        opmode.telemetry.update();
    }

    public void telemetryRawEncoders() {
        opmode.telemetry.addData("left", encoderArray.getLeftPosition);
        opmode.telemetry.addData("right",encoderArray.getRightPosition);
        opmode.telemetry.addData("center",encoderArray.getCenterPosition);
        opmode.telemetry.update();
    }

    public void telemetryChassisPower(){
        opmode.telemetry.addData("FR", frontRight.getPower());
        opmode.telemetry.addData("FL", frontLeft.getPower());
        opmode.telemetry.addData("BR", backRight.getPower());
        opmode.telemetry.addData("BL", backLeft.getPower());
        opmode.telemetry.update();
    }

    public String getFinalPosition() {
        String finalPos = String.format("X: %f\tY: %f\tTheta: %f",encoderArray.X,encoderArray.Y,encoderArray.theta);
        return finalPos;
    }



    public void initTfod(){
        int tfodMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}