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

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeoutException;

/* TODO
   Possibly revert Code to 1/22
 */

public class SSDriveObject extends Object{
    Servo hookHrz, hookVrt, deliveryGrabber, deliveryRotation, cameraServo, leftFoundation, rightFoundation, blockSweeper, capServo;
    CRServo deliveryExtender;
    DcMotor frontRight, frontLeft, backRight, backLeft, rightRoller, leftRoller;
    LinearOpMode opmode;
    Encoder myLeft, myRight, myCenter;
    EncoderArray encoderArray;
    ElapsedTime elapsedTime;

    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";

    final double ROBOT_RADIUS = 13.5;
    final double TICKS_PER_INCH_STRAIGHT = (383.6*2) / (4 * 3.14159265358979323846264);
    final double TICKS_PER_INCH_TURN = TICKS_PER_INCH_STRAIGHT;
    final double TICKS_PER_INCH_STRAFE = (TICKS_PER_INCH_STRAIGHT)*1.15*(20.0/17.0);
    final double TICKS_PER_DEGREE = (3.14159 / 180) *  ROBOT_RADIUS * TICKS_PER_INCH_TURN;
    final double TOLERANCE = 2;  // in degrees
    final double MAXSPEED = 0.65;

    final boolean BLUE = true;
    final boolean RED = false;

    final boolean BRIDGE = false;
    final boolean WALL = true;

    final boolean FOUNDATION = false;
    final boolean NORMAL = true;

    final double WEBCAM_TO_BLOCKS = 9.5;

    final double CENTER_PIXELS = 400.0;
    final double BLOCK_LENGTH = 8.0;
    final double ARM_TO_WEBCAM = 5.875;
    final double FRONT_CENTER_TO_WEBCAM = 3.5;
    final double ROBOT_CENTER_TO_COLLECTOR = 10;

    final double r1 = 5.37;
    final double r2 = 5.85;
    final double r3 = 3.22;

    final double ACCEL_DIST = 10;
    final double DECEL_DIST = 15;

    final boolean FORWARD = true;
    final boolean BACKWARD = false;

    final double PERCENT = .45;
    final double POWER_MIN = 0.22;

    final int TFOD_TIMEOUT = 500;
    final int LEFT = 0;
    final int CENTER = 1;
    final int RIGHT = 2;

    //final double TOLERANCE = ??;
    //final double ROOT2 = 1.414;
    //final int CAMERA_MIDPOINT = ??;
    //final int SAMPLING_FORWARD = ?;

    double inchPerPixel;

    double FRpower = 1;
    double FLpower = 1;
    double BRpower = 1;
    double BLpower = 1;

    double numberOfStones;

    double SS_leftPixel;
    double SS_rightPixel;

    double displacement;
    double secondDisplacement = displacement - 24;

    boolean isVirtual = false;

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    ElapsedTime tfodTimeout;
    ElapsedTime extenderTimeout = new ElapsedTime();

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

        deliveryExtender = opmode.hardwareMap.crservo.get("deliveryExtender");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);

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
        capServo.setPosition(0.8);
        setFoundation(false);
        setBlockSweeper(false);
        setDeliveryGrabber(false);
        setCameraServo(.4);




        opmode.telemetry.addLine("initialized");
        opmode.telemetry.update();
    }

    public void goToDetectPosition(boolean side) {

        if (side == BLUE) {
            //might need to change driveDistance, add a strafe
            strafeDistance(.8,-21);


        } else {
            strafeDistance(.8,-21);
        }

    }


    public String getFinalPosition() {
        String finalPos = String.format("X: %f\tY: %f\tTheta: %f",encoderArray.X,encoderArray.Y,encoderArray.theta*180/Math.PI);
        return finalPos;
    }

    public int detectStonesStatic(boolean side){

        opmode.sleep(1600);
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (!updatedRecognitions.isEmpty()) {
                opmode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (side == RED) {
                    switch (updatedRecognitions.size()) {
                        case 1:
                            numberOfStones = 1;
                            return CENTER;
                        case 2:
                            for (Recognition recognition : updatedRecognitions) {
                                opmode.telemetry.addData("  left", "%.03f", recognition.getLeft());
                                Log.i("STATIC DETECTION","SkyStone Left px: " + recognition.getLeft());
                                opmode.telemetry.addData("  right", "%.03f", recognition.getRight());
                                Log.i("STATIC DETECTION","SkyStone Right px: " + recognition.getRight());
                                opmode.telemetry.update();
                                opmode.sleep(500);
                                numberOfStones = 2;
                                if (recognition.getLabel().equals("Skystone")) {
                                    if (recognition.getRight() > 485) {
                                        return RIGHT;
                                    } else if (recognition.getLeft() < 25){
                                        return CENTER;
                                    }
                                }
                            }
                            return LEFT;
                        default:
                            numberOfStones = 100;
                            return CENTER;

                    }
                } else {
                    switch (updatedRecognitions.size()) {
                        case 1:
                            numberOfStones = 1;
                            return CENTER;
                        case 2:
                            numberOfStones = 2;
                            for (Recognition recognition : updatedRecognitions) {
                                opmode.telemetry.addData("  left", "%.03f", recognition.getLeft());
                                Log.i("STATIC DETECTION","SkyStone Left px: " + recognition.getLeft());
                                opmode.telemetry.addData("  right", "%.03f", recognition.getRight());
                                Log.i("STATIC DETECTION","SkyStone Right px: " + recognition.getRight());
                                opmode.telemetry.update();
                                opmode.sleep(500);
                                if (recognition.getLabel().equals("Skystone")) {
                                    if (recognition.getLeft() < 200) {
                                        return LEFT;
                                    } else if(recognition.getRight() > 750){
                                        return CENTER;
                                    }
                                }
                            }
                            return RIGHT;
                        default:
                            numberOfStones = 100;
                            return CENTER;

                    }
                }
            } else {
                opmode.telemetry.addLine("0 objects detected");
                return CENTER;
            }
        } else {
            opmode.telemetry.addLine("tfod fail");
            return CENTER;
        }


    }

    public void testCollectionAndDelivery(){
        setRollerMotors(false, 1);
        //how to check if the block is collected or not (next round)
        driveDistance(.3,6);
        opmode.sleep(700);
//        setBlockSweeper(true);
//        opmode.sleep(1000);
//        setBlockSweeper(false);
//        opmode.idle();
        setRollerMotors(true,0.0);
        opmode.idle();
        driveDistance(1,20);
        opmode.idle();
        setRollerMotors(true,1);
        opmode.idle();
        driveDistance(1,-10);
//        setDeliveryGrabber(true);
//        opmode.sleep(500);

    }

    public void collectSkyStone(boolean side, int skystone){
        if(side == RED) {
            switch (skystone) {
                case LEFT:
                    driveDistance(.6,-24);
                    opmode.sleep(400);
                    strafeDistance(.3,-5);
                    opmode.sleep(400);
                    turnDegree(.67,45);
                    setRollerMotors(false, 1);
                    opmode.sleep(200);
                    driveDistance(.6,25);
                    opmode.sleep(700);
                    setRollerMotors(false,0.0);
                    driveDistance(.6,-25);
                    opmode.sleep(400);
                    turnDegree(.67,-45);
                    opmode.sleep(400);
                    driveDistance(1,82);
                    setRollerMotors(true,1);
                    opmode.idle();
                    driveDistance(1,-15);
                    opmode.sleep(700);
                    setRollerMotors(true,0.0);

                    break;
                case CENTER:
                    driveDistance(.6,-16);
                    opmode.sleep(500);
                    strafeDistance(.3,-5);
                    opmode.sleep(400);
                    turnDegree(.67,45);
                    setRollerMotors(false, .8);
                    opmode.sleep(200);
                    driveDistance(.6,25);
                    opmode.sleep(700);
                    setRollerMotors(false,0.0);
                    driveDistance(.6,-25);
                    opmode.sleep(400);
                    turnDegree(.67,-45);
                    opmode.sleep(400);
                    driveDistance(1,74);
                    setRollerMotors(true,.8);
                    opmode.idle();
                    driveDistance(1,-15);
                    opmode.sleep(700);
                    setRollerMotors(true,0.0);
                    break;
                case RIGHT:
                    driveDistance(.6,-7.5);
                    opmode.sleep(500);
                    strafeDistance(.3,-5);
                    opmode.sleep(400);
                    turnDegree(.67,45);
                    setRollerMotors(false, .8);
                    opmode.sleep(200);
                    driveDistance(.6,25);
                    opmode.sleep(700);
                    setRollerMotors(false,0.0);
                    driveDistance(.6,-25);
                    opmode.sleep(400);
                    turnDegree(.67,-45);
                    opmode.sleep(400);
                    driveDistance(1,66);
                    setRollerMotors(true,.8);
                    opmode.idle();
                    driveDistance(1,-15);
                    opmode.sleep(700);
                    setRollerMotors(true,0.0);
                    break;
            }
//            setRollerMotors(false, 1);
//            how to check if the block is collected or not (next round)
//            driveDistance(.3,6);
//            opmode.sleep(700);
            
        } else {
            switch (skystone) {
                case LEFT:
                    driveDistance(.6,-20);
                    opmode.sleep(500);
                    turnDegree(.67,45);
                    setRollerMotors(false, 1);
                    opmode.idle();
                    driveDistance(.6,25);
                    opmode.sleep(700);
                    setRollerMotors(false,0.0);
                    driveDistance(.6,-25);
                    opmode.sleep(400);
                    turnDegree(.67,155);
                    opmode.sleep(400);
//                    driveDistanceNoAccel(1,82);
//                    setRollerMotors(true,.8);
//                    opmode.idle();
//                    driveDistance(1,-15);
//                    opmode.sleep(700);
//                    setRollerMotors(true,0.0);

                    break;
                case CENTER:
                    driveDistance(.6,-18);
                    opmode.sleep(500);
                    turnDegree(.67,45);
                    setRollerMotors(false, 1);
                    opmode.sleep(200);
                    driveDistance(.6,25);
                    opmode.sleep(700);
                    setRollerMotors(false,0.0);
                    driveDistance(.6,-25);
                    opmode.sleep(400);
                    turnDegree(.67,-50);
                    opmode.sleep(400);
                    driveDistance(1,74);
                    setRollerMotors(true,.8);
                    opmode.idle();
                    driveDistance(1,-15);
                    opmode.sleep(700);
                    setRollerMotors(true,0.0);
                    break;
                case RIGHT:
                    driveDistance(.6,-7.5);
                    opmode.sleep(500);
                    turnDegree(.67,45);
                    setRollerMotors(false, 1);
                    opmode.sleep(200);
                    driveDistance(.6,25);
                    setRollerMotors(false,0.0);
                    driveDistance(.6,-25);
                    opmode.sleep(400);
                    turnDegree(.67,-50);
                    opmode.sleep(400);
                    driveDistance(1,66);
                    setRollerMotors(true,.8);
                    opmode.idle();
                    driveDistance(1,-15);
                    opmode.sleep(700);
                    setRollerMotors(true,0.0);
                    break;
            }
        }
//        setRollerMotors(true, 1);
//        driveDistance(1,6);
//        opmode.sleep(400);
//        setBlockSweeper(true);
//        opmode.sleep(500);
//        setBlockSweeper(false);
//        opmode.idle();
//        setRollerMotors(true,0.0);
    }

    public void moveToFoundation(boolean side){
        if (side) {
            turnDegree(1, -90);
            opmode.sleep(50);
            driveDistance(1,61.5);
            opmode.sleep(50);
            turnDegree(1,-90);
            opmode.sleep(50);
        } else{

        }
        driveDistance(1,-12.5);
    }

    public void moveFoundation (boolean side)  {
        /*setFoundation(true);
        opmode.sleep(400);
        driveDistanceNoAccel(.4,1);
        opmode.sleep(400);
        strafeDistanceNoAccel(.3,20 );
        *//*opmode.sleep(400);
        strafeDistanceNoAccel(.5,-10);
        opmode.sleep(400);
        turnDegree(.3,-80);
        opmode.sleep(400);
        turnDegree(.4,7.5);
        strafeDistanceNoAccel(.8,12);
        opmode.sleep(400);
        driveDistance(1,-30);*/

        if (side == BLUE) {
            /*driveDistance(.6, -27);
            opmode.sleep(400);
            strafeDistance(.6,4);
            opmode.sleep(400);
            setFoundation(true);
            opmode.sleep(400);
            driveDistance(.6, 18);
            opmode.sleep(400);
            turnDegree(.3,145);
            opmode.sleep(400);
            setFoundation(false);
            opmode.sleep(400);
            driveDistance(.7,-11);
            opmode.sleep(400);
            turnDegree(.67,2);
            opmode.sleep(400);*/

        } else if (side == RED) {
            /*opmode.telemetry.addLine("red Foundation moving");
            driveDistance(.6, -27);
            opmode.sleep(400);
            strafeDistance(.6,-2);
            opmode.sleep(400);
            setFoundation(true);
            opmode.sleep(400);
            driveDistance(.6, 7);
            opmode.sleep(400);
            turnDegree(.3,-155);
            opmode.sleep(400);
            setFoundation(false);
            opmode.sleep(400);
            driveDistance(.7,-11);
            opmode.sleep(400);*/
        }
    }

    public void park (boolean side, boolean state) {
        //Side
        //true = blue
        //false = red
        //State
        //true = wall
        //false = bridge

        if(state) {
            if (side) {
                opmode.telemetry.addLine("Blue wall");
                opmode.telemetry.update();
                driveDistance(.6,10);
                opmode.sleep(400);
                strafeDistance(.6,16);
                opmode.sleep(400);
                driveDistance(.6,28);
                opmode.sleep(400);
            } else {
                opmode.telemetry.addLine("Red wall");
                opmode.telemetry.update();
                driveDistance(.6,10);
                opmode.sleep(400);
                strafeDistance(.6,-25);
                opmode.sleep(400);
                driveDistance(.6,28);
                opmode.sleep(400);
            }
        } else {
            if (side) {
                opmode.telemetry.addLine("Blue bridge");
                opmode.telemetry.update();
                driveDistance(.6,10);
                opmode.sleep(400);
                strafeDistance(.6,-10);
                opmode.sleep(400);
                driveDistance(.6,28);
                opmode.sleep(400);
            } else {
                opmode.telemetry.addLine("Red bridge");
                opmode.telemetry.update();
                /*turnDegree(.4,-1.5);
                opmode.sleep(250);
                driveDistance(1,-1);
                opmode.sleep(250);
                driveDistance(1,35);*/
                /*opmode.sleep(400);
                driveDistance(1,12);*/

                driveDistance(.6,38);
                opmode.sleep(400);
            }


        }
    }

    public void skystonePark(boolean side) {
        if (side) {
            driveDistance(1,20);
        } else {

        }
    }

    public void deliverSkystone (boolean side) {
        if (side) {
            strafeDistance(1,25);
            driveDistance(1,-25);
            setBlockSweeper(true);
            opmode.sleep(500);
            setDeliveryGrabber(true);
            opmode.sleep(500);
            setDeliveryExtender(1,2500);
            opmode.sleep(500);
            setDeliveryGrabber(false);
            opmode.sleep(250);
            setDeliveryExtender(-1,2500);
        } else {

        }
    }

    //detection

    public void initTfod(){
        int tfodMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void detectStonesDynamic(){
        tfodTimeout = new ElapsedTime();

        if (opmode.opModeIsActive()) {
            while (opmode.opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        opmode.telemetry.addData("# Object Detected", updatedRecognitions.size());

                        int i = 0;
                        boolean skyFlag = false;

                        for (Recognition recognition : updatedRecognitions) {
                            opmode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            opmode.telemetry.addData("  left", "%.03f", recognition.getLeft());
                            opmode.telemetry.addData("  right", "%.03f", recognition.getRight());

                            if(recognition.getLabel().equals("Skystone")){
                                SS_leftPixel = recognition.getLeft();
                                SS_rightPixel = recognition.getRight();
                                skyFlag = true;
                                break;
                            }
                        }
                        opmode.telemetry.update();
                        opmode.sleep(1000);

                        if(!skyFlag) {
                            isVirtual = true;
                            createVirtualStone(updatedRecognitions.get(0).getRight(), updatedRecognitions.get(0).getLeft(), updatedRecognitions.get(1).getRight(), updatedRecognitions.get(0).getLeft());
                        }
                    }
                }
                if (tfodTimeout.milliseconds() >= TFOD_TIMEOUT) {
                    tfod.shutdown();
                    opmode.telemetry.addLine("Tfod Terminated");
                    opmode.telemetry.update();
                    break;
                }
            }
        }
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

    public void setDrivePowerLeft(double powerLeft) {
        frontLeft.setPower(powerLeft);
        backLeft.setPower(powerLeft);
    }

    public void setDrivePowerRight(double powerRight) {
        frontRight.setPower(powerRight);
        backRight.setPower(powerRight);
    }

    public void driveDistanceCompensate(double powerLeft, double powerRight, double distance) {
        final double PERCENT = .1;
        int ticks = (int) (distance * TICKS_PER_INCH_STRAIGHT);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.telemetry.addLine("Encoders reset");
        opmode.telemetry.update();
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (ticks > 0) {
            while ((frontLeft.getCurrentPosition() <= ticks) && opmode.opModeIsActive()) {
                setDrivePowerLeft(powerLeft);
                setDrivePowerRight(powerRight);
                opmode.telemetry.addLine("cruising");
                telemetryDcMotor();
            }
        } else if (ticks < 0) {
            powerLeft = -powerLeft;
            powerRight = -powerRight;
//            powerMin = -powerMin;
            while ((frontLeft.getCurrentPosition() >= ticks) && opmode.opModeIsActive()) {
                setDrivePowerLeft(powerLeft);
                setDrivePowerRight(powerRight);
                opmode.telemetry.addLine("cruising");
                telemetryDcMotor();
                opmode.telemetry.update();
            }
        }
        stopDriving();
        opmode.telemetry.addLine("done driving");
    }

    /*
    public void driveDistanceNoAccel(double power, double distance) {
        encoderArray.updateAll();
        encoderArray.resetAll();

        if (distance > 0) {
            while((encoderArray.getDeltaY() <= distance) && opmode.opModeIsActive()) {
                setDrivePowerAll(power);
                telemetryEncoderArray();
            }
        } else if (distance < 0) {
            power = -power;
            while((encoderArray.getDeltaY() >= distance) && opmode.opModeIsActive()) {
                setDrivePowerAll(power);
                telemetryEncoderArray();
            }
        }
        opmode.telemetry.addData("target reached", encoderArray.getDeltaY());
        opmode.telemetry.update();
        stopDriving();
        opmode.telemetry.addLine("motors stopped");
        opmode.telemetry.update();
    }*/

    public void testEncoderRead(double time){
        double TIMEOUT = time;
        elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < TIMEOUT) {
            encoderArray.readEncoderValue();
            telemetryEncoderArray();
        }
    }

    public double calculatePowerStraight(double powerLimit, double distance, double deltaY) {
        distance = Math.abs(distance);
        deltaY = Math.abs(deltaY);
        if (distance != 0) {
            return ((0.44 - 4 * powerLimit) / (distance * distance)) * deltaY * deltaY + ((4 * powerLimit - 0.66) / distance) * deltaY + POWER_MIN;
        } else {
            return 0;
        }
    }

    public double calculatePowerStrafe(double powerLimit, double distance, double deltaX){
        distance = Math.abs(distance);
        deltaX = Math.abs(deltaX);

        if(distance != 0){

            return ((0.44 - 4 * powerLimit) / (distance * distance)) * deltaX * deltaX + ((4 * powerLimit - 0.66) / distance) * deltaX + POWER_MIN;

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
                if (deltaY >= distance)
                    break;

                setDrivePowerAll(Math.max(.22,calculatePowerStraight(powerLimit, distance, deltaY)));
                Log.i("POWER",String.format("Delta Y: %f\tPower: %f\n", deltaY, calculatePowerStraight(powerLimit,distance,deltaY)));


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
                if (deltaY <= distance)
                    break;

                setDrivePowerAll(-Math.max(.22,calculatePowerStraight(powerLimit, distance, deltaY)));
//                opmode.telemetry.addData("deltaY", encoderArray.getDeltaY());
//                opmode.telemetry.update();
            }
        }
        stopDriving();
        opmode.telemetry.addLine("motors stopped");
        opmode.telemetry.update();

//        System.out.println(debugString);
//        debugString="";
//        opmode.sleep(500);
    }

    /*public void strafeDistanceNoAccel(double power, double distance) {
        encoderArray.updateAll();
        encoderArray.resetAll();

        if (distance > 0) {
            while((encoderArray.getDeltaX() <= distance) && opmode.opModeIsActive()) {
                setStrafePowerAll(power);
                telemetryEncoderArray();
            }
        } else if (distance < 0) {
            power = -power;
            while((encoderArray.getDeltaX() >= distance) && opmode.opModeIsActive()) {
                setStrafePowerAll(power);
                telemetryEncoderArray();
            }
        }
        opmode.telemetry.addData("target reached", encoderArray.getDeltaY());
        opmode.telemetry.update();
        stopDriving();
        opmode.telemetry.addLine("motors stopped");
        opmode.telemetry.update();

    }*/

    public void strafeDistance(double powerLimit, double distance) {
        double deltaX = 0;
        double deltaTheta = 0;
        double strafePower;

        if (distance > 0) {
            if (powerLimit == .8) {
                distance -= .23;
            } else if (powerLimit ==.5) {
                distance -= .24;
            } else {
                distance -= .68;
            }
        } else if (distance < 0) {
            if (powerLimit == .8) {
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

        if (distance > 0) {
            while((deltaX <= distance) && opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaX = encoderArray.getDeltaX();
                deltaTheta = encoderArray.getDeltaTheta();
                strafePower = Math.max(.22,calculatePowerStrafe(powerLimit, distance, deltaX));
                setSelectPowerAll(-strafePower - .02*deltaTheta, strafePower + .02*deltaTheta, strafePower + .02*deltaTheta,-strafePower - .02*deltaTheta);
                Log.i("POWER",String.format("Delta X: %f\tPower: %f\n", deltaX, Math.max(.22,calculatePowerTurn(powerLimit, distance, deltaX))));
//                telemetryEncoderArray();
//                opmode.telemetry.addData("deltaX: ", encoderArray.getDeltaX());
//                telemetryWheelPower();
            }
        } else if (distance < 0) {

            while((deltaX >= distance) && opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaX = encoderArray.getDeltaX();
                strafePower = Math.max(.22,calculatePowerStrafe(powerLimit, distance, deltaX));
                setSelectPowerAll(strafePower,-strafePower,-strafePower,strafePower);
                Log.i("POWER",String.format("Delta X: %f\tPower: %f\n", deltaX, -Math.max(.22,calculatePowerTurn(powerLimit, distance, deltaX))));
//                opmode.telemetry.addData("deltaY", deltaX);
//                opmode.telemetry.update();
            }
        }
        stopDriving();
        opmode.telemetry.addLine("motors stopped");
        opmode.telemetry.update();
//        opmode.sleep(500);

    }

    public void turnDegree(double powerLimit, double degrees) {
        // distance in inches
        //conjecture instead of moving 12", wheels will go 12"*cos(45)= 8.5"

        double deltaTheta = 0;
        encoderArray.readEncoderValue();
        encoderArray.updateAll();
        encoderArray.resetAll();

        if (degrees > 0) {
            // positive (CCW) turn => left motor goes in (-) direction
            while ((deltaTheta <= degrees) && opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaTheta = encoderArray.getDeltaTheta()*180/Math.PI;
                setTurnPowerAll(Math.max(.15,calculatePowerTurn(powerLimit, degrees, deltaTheta)));
                Log.i("POWER",String.format("Delta Theta: %f\tPower: %f\n", deltaTheta, calculatePowerTurn(powerLimit,degrees,deltaTheta)));

            }
        } else if (degrees < 0) {
            while ((deltaTheta >= degrees) && opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaTheta = encoderArray.getDeltaTheta()*180/Math.PI;
                setTurnPowerAll(-(Math.max(.15,calculatePowerTurn(powerLimit, degrees, deltaTheta))));
                Log.i("POWER",String.format("Delta Theta: %f\tPower: %f\n", deltaTheta, -calculatePowerTurn(powerLimit, degrees, deltaTheta)));
//                opmode.telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
//                opmode.telemetry.update();
            }
        }

        stopDriving();
//        getAngleTelemetry("TURN END");

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

    public void setSelectPowerAll(double FRpower, double FLpower, double BRpower, double BLpower){
        frontRight.setPower(FRpower);
        frontLeft.setPower(FLpower);
        backRight.setPower(BRpower);
        backLeft.setPower(BLpower);
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

    //set other motors

    public void setRollerMotors (boolean direction, double power) {
        //direction true = inward
        //direction false = outward
        /*rollerTimeout = new ElapsedTime();
        final int ROLLER_TIMEOUT = time;*/

        if(direction){
            leftRoller.setDirection(DcMotorSimple.Direction.FORWARD);
            rightRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if(!direction){
            leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        leftRoller.setPower(power);
        rightRoller.setPower(power);



        /*while ((rightRoller.isBusy() || leftRoller.isBusy()) && opmode.opModeIsActive()){
            if (rollerTimeout.milliseconds() > ROLLER_TIMEOUT)
                break;
        }*/
    }

    public void setCameraServo (double position) {
        cameraServo.setPosition(position);
    }

    public void setFoundation (boolean launch) {
        //launch true = grabber down
        //launch false = grabber up
        if (launch) {
            leftFoundation.setPosition(.5);
            rightFoundation.setPosition(.25);
        } else {
            leftFoundation.setPosition(0);
            rightFoundation.setPosition(.8);
        }
    }

    public void setBlockSweeper (boolean kick) {
        //kick true = blockSweeper up
        //kick false = blockSweeper down

        if (kick) {
            blockSweeper.setPosition(0.725);

        } else {
            blockSweeper.setPosition(1);

        }
    }

    public void setDeliveryGrabber (boolean grab) {
        if(grab){
            deliveryGrabber.setPosition(.5);
        } else{
            deliveryGrabber.setPosition(0);
        }
    }

    public void setDeliveryExtender (double power, int time) {
        extenderTimeout.reset();
        deliveryExtender.setPower(power);
    }

    public void setDeliveryRotation (boolean rotate) {
        //rotate true = rotate Middle
        //rotate false = rotate in

        if (rotate) {
            deliveryRotation.setPosition(0.5);
        }
        else {
            deliveryRotation.setPosition(0);
        }
    }

    //telemetry

    public void telemetryEncoderArray (){
        opmode.telemetry.addData("deltaX", encoderArray.getDeltaX());
        opmode.telemetry.addData("deltaY", encoderArray.getDeltaY());
        opmode.telemetry.addData("deltaTheta(degrees)", encoderArray.getDeltaTheta()*180/Math.PI);
        opmode.telemetry.update();
    }

    public void telemetryDcMotor(){
        opmode.telemetry.addData("FR", frontRight.getCurrentPosition());
        opmode.telemetry.addData("FL", frontLeft.getCurrentPosition());
        opmode.telemetry.addData("BR", backRight.getCurrentPosition());
        opmode.telemetry.addData("BL", backLeft.getCurrentPosition());
        opmode.telemetry.update();
    }

    public void telemetryWheelPower(){
        opmode.telemetry.addData("FR", frontRight.getPower());
        opmode.telemetry.addData("FL", frontLeft.getPower());
        opmode.telemetry.addData("BR", backRight.getPower());
        opmode.telemetry.addData("BL", backLeft.getPower());
        opmode.telemetry.update();
    }

    public void getAngleTelemetry (String status){
        opmode.telemetry.addLine(status);
        opmode.telemetry.addData("   encoder: ", frontLeft.getCurrentPosition()/TICKS_PER_DEGREE);
        opmode.telemetry.update();
    }
}