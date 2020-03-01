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

/* TODO
  Possibly revert Code to 1/22
*/

public class SSDriveObject extends Object{
    Servo hookHrz, hookVrt, deliveryGrabber, deliveryRotation, cameraServo, leftFoundation, rightFoundation, blockSweeper, capServo, deliveryExtender;
    DcMotor frontRight, frontLeft, backRight, backLeft, rightRoller, leftRoller;
    LinearOpMode opmode;
    Encoder myLeft, myRight, myCenter;
    EncoderArray encoderArray;
    ElapsedTime elapsedTime;
    OpenCvCamera webcam;
    private SkyStoneDetector detector = new SkyStoneDetector();
    private String position;

    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";

    final double ROBOT_RADIUS = 13.5;
    final double TICKS_PER_INCH_STRAIGHT = (383.6*2) / (4 * 3.14159265358979323846264);
    final double TICKS_PER_INCH_TURN = TICKS_PER_INCH_STRAIGHT;
    final double TICKS_PER_INCH_STRAFE = (TICKS_PER_INCH_STRAIGHT)*1.15*(20.0/17.0);
    final double TICKS_PER_DEGREE = (3.14159 / 180) *  ROBOT_RADIUS * TICKS_PER_INCH_TURN;
    final double TOLERANCE = 3;  // in degrees

    final boolean BLUE = true;
    final boolean RED = false;
    final boolean BRIDGE = false;
    final boolean WALL = true;

    final boolean IN = true;
    final boolean OUT = false;

    final boolean LEFT = true;
    final boolean RIGHT = false;

    final boolean FDOWN = true;
    final boolean FUP = false;

    final double WEBCAM_TO_BLOCKS = 9.5;
    final double CENTER_PIXELS = 400.0;
    final double BLOCK_LENGTH = 8.0;
    final double ARM_TO_WEBCAM = 5.875;
    final double FRONT_CENTER_TO_WEBCAM = 3.5;

    final double r1 = 5.37;
    final double r2 = 5.85;
    final double r3 = 3.22;

    final double POWER_MIN = 0.3;
    final double POWER_MAX = .8;

    int skystone;

    final int LEFT_SS = 0;
    final int CENTER_SS = 1;
    final int RIGHT_SS = 2;

    final int BS_OUT = 1;
    final int BS_IN = 2;

    //final double TOLERANCE = ??;
    //final double ROOT2 = 1.414;
    final int DETECT_STRAFE = -21;

    final double CAP_INIT_POS = .725;
    final double CAM_INIT_POS = .4;

    double inchPerPixel;

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

    public SSDriveObject(LinearOpMode parent){
        opmode = parent;

        frontRight = opmode.hardwareMap.dcMotor.get("frontRight");
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

        deliveryExtender = opmode.hardwareMap.servo.get("deliveryExtender");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        rightRoller.setDirection(DcMotor.Direction.FORWARD);
        leftRoller.setDirection(DcMotor.Direction.REVERSE);

        deliveryExtender.setDirection(Servo.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myLeft = new Encoder(frontLeft);
        myRight = new Encoder(frontRight);
        myCenter = new Encoder(backRight);
        encoderArray = new EncoderArray(myLeft, myRight, myCenter, r1, r2, r3);

        //vuforia tfod init

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AbLVQDn/////AAABma+zV9cCqU+7pLBUXgQ3J6II1u1B8Vg4mrnGfVawPjc1l7C6GWoddOaL6Wqj5kXPBVUh3U3WND38234Tm0h3+LKmmTzzaVPRwOk3J+zBwKlOvv93+u7chctULk8ZYEyf0NuuEfsGwpgJx7xL9hIFBoaB2G1SpbJIt+n94wz6EvfRYSusBEiST/lUqgDISIlaeOLPWEipHh46axomcrGVRRl09pg6pCt2h7rU6us+guN5nKhupTXvM+BTUYW3kCO9YsUjz16jLr7GyFh8wVQbRS3dikSX7kzVsdkLjZnJdyinYaB5oDXfmmXtaC6ZXeD6vKs62vpaydAq9VGAlCtnSyq2J4NLI+LOIOvdtsCwarfS";
        parameters.cameraName = opmode.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        initTfod();
        if (tfod != null) {
            tfod.activate();
        }
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

        skystone = detectStonesStatic(RED);
        opmode.telemetry.addLine(skystoneString(skystone));

        encoderArray.resetAll();

        opmode.telemetry.addLine("initialized");
        opmode.telemetry.update();
    }

    public int detectStonesStatic(boolean side){
        int cameraMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(opmode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(detector);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        opmode.telemetry.addData("is started", opmode.isStarted());

        while (!opmode.isStarted()) {
            position = detector.position;
            opmode.telemetry.addData("position", position);
            opmode.telemetry.update();
            Log.i("OPENCV", String.format("driveDistance: \t%s\n",position));
        }

            if (position.equals("LEFT")) {
                return LEFT_SS;
            } else if (position.equals("RIGHT")) {
                return RIGHT_SS;
            } else {
                return CENTER_SS;
            }

    }

    public void collectionBlueLeft() {
        turnDegree(.67, 65);


        setRollerMotors(IN, .6);
        driveDistance(.8, 15);


        driveDistance(.8, -15);

        setBlockSweeper(IN);
        stopRollerMotors();
        turnToDegree(.67, 0);
        setDeliveryGrabber(IN);
        setBlockSweeper(OUT);
        Log.i("FINALTHETA",String.format("FinalTheta after turn back: %f", (encoderArray.theta)));
    }

    public void collectSkyStone(boolean side, int skystone){
        setBlockSweeper(OUT);
        if(side == BLUE) {
            strafeDistance(POWER_MAX,DETECT_STRAFE);
            switch (skystone) {
                case LEFT_SS:
                    driveDistance(.8, BLOCK_LENGTH-1);
                    opmode.idle();
                    collectionBlueLeft();
                    opmode.idle();
                    turnDegree(.3,.01);
                    encoderArray.theta=0;
                    driveDistance(1, - (92));
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case CENTER_SS:
                    driveDistance(.8, -(7 + BLOCK_LENGTH));
                    opmode.idle();
                    collection(side);

                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    turnDegree(.3, -.1);
                    encoderArray.theta = 0;
                    driveDistance(1, - (60 + BLOCK_LENGTH));
                    break;
                case RIGHT_SS:
                    driveDistance(.8,-7);
                    opmode.idle();
                    collection(side);
                    turnDegree(.3, -1);
                    driveDistance(1,-(60 + BLOCK_LENGTH*2));
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));

                    break;
            }
        } else if (side == RED) {
            strafeDistance(POWER_MAX,DETECT_STRAFE);
            Log.i("FINALTHETA",String.format("FinalTheta after strafe: %f", (encoderArray.theta)));
            switch (skystone) {
                case LEFT_SS:
                    driveDistance(1,-22.5);
                    opmode.idle();
                    collection(side);
                    driveDistance(1,90 + BLOCK_LENGTH*2);
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case CENTER_SS:
                    driveDistance(1,-13);//.5 -> 1
                    Log.i("FINALTHETA",String.format("FinalTheta after very first drive to collect: %f", (encoderArray.theta)));
                    opmode.idle();
                    collection(side);
                    driveDistance(1,90 + BLOCK_LENGTH);
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case RIGHT_SS:
                    driveDistance(1,-7);
                    opmode.idle();
                    collectionRightRed();
                    driveDistance(1,90);
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
            }
        }
    }

    public void moveToFoundation(boolean side){
        if (side == BLUE) {
            turnDegree(.67, -88);
        } else if (side == RED){
            turnDegree(.67,-92);
        }

//        turnToDegree(.67, -90);
        Log.i("THETA", String.format("delta theta: \t%f\n", encoderArray.getDeltaTheta()));
        Log.i("THETA", String.format("theta: \t%f\n", encoderArray.theta));

        driveDistance(.5, -7.5);
        //Log.i("ACTIONTIME", String.format("moveToFoundation: \t%f\n",opModeTime.milliseconds()));
    }

    public void moveFoundation (boolean side) {
//        turnDegree(.3,-0.1);
        setFoundation(FDOWN);
        opmode.sleep(400);//500 -> 300
        driveDistance(1, 22.5);
        setRollerMotors(OUT,1);
        arcWhileExtending(.67, side);
        stopRollerMotors();
        opmode.idle();
        driveDistance(1, -12);
        opmode.sleep(200);
        setDeliveryGrabber(OUT);
        //Log.i("ACTIONTIME", String.format("moveFoundation: \t%f\n",opModeTime.milliseconds()));
    }

    public void moveToSecondSS(boolean side, boolean state){
        setFoundation(FUP);
        setBlockSweeper(OUT);

        setDeliveryRotation(IN);
        if(state == WALL) {
            if (side == BLUE) {
                strafeDistance(.8, 19);
            } else if (side == RED) {
                strafeDistance(.8, -26);
            }
        } else if (state == BRIDGE) {
            if (side == BLUE) {
//                strafeDistance(.8, -2);
            } else if (side == RED) {
                strafeDistance(.8, -2);
            }
        }
    }

    public void collectSecondSkyStone(boolean side){
        setDeliveryExtender(IN);
        if(side == BLUE) {
            switch (skystone) {
                case LEFT_SS:
                    driveDistance(1,62);
                    opmode.idle();
                    collectionBlueLeft();
                    driveDistance(1,-(62));
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case CENTER_SS:
                    driveDistance(1,80 + BLOCK_LENGTH);
                    opmode.idle();
                    collection2(side);
                    driveDistance(1,-(80 + BLOCK_LENGTH));
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case RIGHT_SS:
                    driveDistance(1,80 + BLOCK_LENGTH*2);
                    opmode.idle();
                    collection2(side);
                    driveDistance(1,-(80+ BLOCK_LENGTH*2));
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
            }
        } else if (side == RED) {
            switch (skystone) {
                case LEFT_SS:
                    driveDistance(1,79 + BLOCK_LENGTH*2);
                    opmode.idle();
                    collection2(side);
                    driveDistance(POWER_MAX,52 + BLOCK_LENGTH*2);
                    setRollerMotors(OUT, 1);
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case CENTER_SS:
                    driveDistance(1,80 + BLOCK_LENGTH);
                    Log.i("FINALTHETA",String.format("FinalTheta after very first drive to collect: %f", (encoderArray.theta)));
                    opmode.idle();
                    collection2(side);
                    //driveDistance(1,-88);
                    driveDistance(1,52 + BLOCK_LENGTH);
                    setRollerMotors(OUT, 1);
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case RIGHT_SS:
                    driveDistance(1,78);
                    opmode.idle();
                    collection2(side);
                    driveDistance(1,52);
                    setRollerMotors(OUT, 1);
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
            }
        }
    }

    public void deliverSkystone (boolean side) {
        setDeliveryRotation(OUT);
        opmode.sleep(300);
        setDeliveryGrabber(OUT);
    }

    public void park (boolean side, boolean state) {
        if (side == RED) {
        setRollerMotors(OUT,0);
        setBlockSweeper(IN);
        driveDistance(.8,-15);
        } else if (side == BLUE) {
            setDeliveryExtender(OUT);
            opmode.sleep(750);
            deliverSkystone(BLUE);
            strafeDistance(.5,-.1);
            driveDistance(1,35);

        }
        setDeliveryRotation(IN);
        setDeliveryExtender(IN);
        opmode.sleep(1000);


    }

    //detection

    public void initTfod(){
        int tfodMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void createVirtualStone(double S1_rightPixel, double S1_leftPixel, double S2_rightPixel, double S2_leftPixel){
        double S1_size = S1_rightPixel - S1_leftPixel;
        double S2_size = S2_rightPixel - S2_leftPixel;

        SS_rightPixel = Math.min(S1_leftPixel, S2_leftPixel);
        SS_leftPixel = SS_rightPixel - (S1_size + S2_size)/2;

        opmode.telemetry.addLine("Skystone virtually created");
        opmode.telemetry.update();
    }

    public void getDisplacement(){
        double SS_size = SS_rightPixel - SS_leftPixel;
        inchPerPixel = Math.abs(BLOCK_LENGTH/(SS_size));
        opmode.telemetry.addData("inch / pixel", "%.03f", inchPerPixel);

        //displacement = ((5/8)*(SS_rightPixel - SS_leftPixel) + SS_leftPixel - CENTER_PIXELS)*inchPerPixel - ARM_TO_WEBCAM;
        if(250 <= SS_rightPixel && SS_rightPixel <= 500) {
            opmode.telemetry.addLine("center");
            displacement = inchPerPixel * (SS_rightPixel - CENTER_PIXELS) - FRONT_CENTER_TO_WEBCAM/*+ ARM_TO_WEBCAM - 3*/;
        }
        else if(600 <= SS_rightPixel && SS_rightPixel <= 800) {
            opmode.telemetry.addLine("right");
            displacement = inchPerPixel * (SS_leftPixel - CENTER_PIXELS) - FRONT_CENTER_TO_WEBCAM/*+ ARM_TO_WEBCAM + 5*/;
        }
        else if(isVirtual) {
            opmode.telemetry.addLine("left");
            displacement = inchPerPixel * (SS_rightPixel + SS_size - CENTER_PIXELS) - FRONT_CENTER_TO_WEBCAM/*- 8 + ARM_TO_WEBCAM*/;
        }
    }

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
        encoderArray.updateAll();
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
        encoderArray.updateAll();
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
        encoderArray.updateAll();
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
        encoderArray.updateAll();
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
        encoderArray.updateAll();
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
        encoderArray.updateAll();
        encoderArray.resetAll();
    }



    public void turnToDegree(double powerLimit, double targetDegrees) {
        //This needs work
        encoderArray.readEncoderValue();
        encoderArray.updateAll();
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
        encoderArray.updateAll();
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
        encoderArray.updateAll();
        encoderArray.resetAll();
    }

    public void arcWhileExtending (double powerLimit, boolean side) {
       //true = in
       //false = out

       double deltaY = 0;
       double deltaTheta = 0;

       encoderArray.readEncoderValue();
       encoderArray.updateAll();
       encoderArray.resetAll();

       setDeliveryExtender(OUT);

       if (side == RED) {
           while(opmode.opModeIsActive()) {
               encoderArray.readEncoderValue();
               deltaTheta = encoderArray.getDeltaTheta();

               if (deltaTheta <= -90)
                   break;

               setSelectPowerAll(-1, 1, 0, 0);
               //Log.i("POWER",String.format("Delta Y: %f\tPower: %f\n", deltaY, calculatePowerStraight(powerLimit,distance,deltaY)));
//                telemetryEncoderArray();
           }
       } else {
           while(opmode.opModeIsActive()) {
               encoderArray.readEncoderValue();
               deltaY = encoderArray.getDeltaY();
               deltaTheta = encoderArray.getDeltaTheta();
               if (deltaTheta >= 90)
                   break;

               setSelectPowerAll(1, -1, 0, 0);
           }
       }
       setDeliveryRotation(OUT);
       stopDriving();
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

    public void collectionRightRed() {
        turnToDegree(.67, 45);
        Log.i("FINALTHETA",String.format("FinalTheta after turn in: %f", (encoderArray.theta)));

        setRollerMotors(IN, .6);
        driveDistance(1, 20);
        Log.i("FINALTHETA",String.format("FinalTheta after drive forward: %f", (encoderArray.theta)));

        driveDistance(1, -20);
        Log.i("FINALTHETA",String.format("FinalTheta after drive back: %f", (encoderArray.theta)));
        setBlockSweeper(IN);

        stopRollerMotors();


        turnToDegree(.67, 0);
        setBlockSweeper(OUT);
        Log.i("FINALTHETA",String.format("FinalTheta after turn back: %f", (encoderArray.theta)));
        setDeliveryGrabber(IN);
    }

    public void collection(boolean side){
        if(side == BLUE) {
            /*turnDegree(.67, 45);
            setRollerMotors(IN, .6);
            driveDistance(1, 15);
            driveDistance(1, -15);
            setBlockSweeper(IN);
            stopRollerMotors();
            turnToDegree(.67, 0);
            setBlockSweeper(OUT);
            setDeliveryGrabber(IN);*/
            strafeDistance(.8, -12.5);
            setRollerMotors(IN, .6);
            driveDistance(.5, 5);
            driveDistance(.5, -5);
            opmode.idle();
            setBlockSweeper(IN);
            strafeDistance(.8, 13);
            setDeliveryGrabber(IN);
            setBlockSweeper(OUT);

        } else if(side == RED) {
//            strafeDistance(.3, -5);

            turnDegree(.67, 45);
            Log.i("FINALTHETA",String.format("FinalTheta after turn in: %f", (encoderArray.theta)));

            setRollerMotors(IN, .6);
            driveDistance(1, 15);
            Log.i("FINALTHETA",String.format("FinalTheta after drive forward: %f", (encoderArray.theta)));

            driveDistance(1, -15);
            Log.i("FINALTHETA",String.format("FinalTheta after drive back: %f", (encoderArray.theta)));
            setBlockSweeper(IN);
            stopRollerMotors();
            turnToDegree(.67, 0);
            setDeliveryGrabber(IN);
            setBlockSweeper(OUT);
            Log.i("FINALTHETA",String.format("FinalTheta after turn back: %f", (encoderArray.theta)));

        }
    }

    public void collection2 (boolean side){
        if(side == BLUE) {
            strafeDistance(.8, -11);
            setRollerMotors(IN, .6);
            driveDistance(.5, 5);
            driveDistance(.5, -5);
            opmode.idle();
            setBlockSweeper(IN);
            strafeDistance(.8, 11);
            setDeliveryGrabber(IN);
            setBlockSweeper(OUT);
        } else if(side == RED) {
//            strafeDistance(.3, -5);
            turnDegree(.67, -45);
            Log.i("FINALTHETA",String.format("FinalTheta after turn in: %f", (encoderArray.theta)));

            setRollerMotors(IN, .6);
            driveDistance(1, 15);
            Log.i("FINALTHETA",String.format("FinalTheta after drive forward: %f", (encoderArray.theta)));

            driveDistance(1, -15);
            Log.i("FINALTHETA",String.format("FinalTheta after drive back: %f", (encoderArray.theta)));

            //setBlockSweeper(IN);
            stopRollerMotors();
            turnDegree(.67, -135);
            Log.i("FINALTHETA",String.format("FinalTheta after turn back: %f", (encoderArray.theta)));
            //setDeliveryGrabber(true);
            //setDeliveryExtender(OUT);
        }
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