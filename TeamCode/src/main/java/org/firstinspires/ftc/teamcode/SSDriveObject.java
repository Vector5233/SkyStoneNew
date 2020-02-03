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
    final double TOLERANCE = 3;  // in degrees

    final boolean BLUE = true;
    final boolean RED = false;
    final boolean BRIDGE = false;
    final boolean WALL = true;

    final double WEBCAM_TO_BLOCKS = 9.5;
    final double CENTER_PIXELS = 400.0;
    final double BLOCK_LENGTH = 8.0;
    final double ARM_TO_WEBCAM = 5.875;
    final double FRONT_CENTER_TO_WEBCAM = 3.5;

    final double r1 = 5.37;
    final double r2 = 5.85;
    final double r3 = 3.22;

    final double POWER_MIN = 0.22;
    final double POWER_MAX = .8;
    
    final int LEFT = 0;
    final int CENTER = 1;
    final int RIGHT = 2;

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

    final double ROLLER_POWER = 1;

    double numberOfStones;

    double SS_leftPixel;
    double SS_rightPixel;

    double displacement;
    double secondDisplacement = displacement - 24;

    boolean isVirtual = false;

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;
    
    ElapsedTime extenderTimeout = new ElapsedTime();
    ElapsedTime opModeTime = new ElapsedTime();

    final int EXTEND_OPMODETIMEOUT = 1875;

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
        capServo.setPosition(CAP_INIT_POS);
        setFoundation(false);
        setBlockSweeper(false);
        setDeliveryGrabber(false);
        setCameraServo(CAM_INIT_POS);
        
        opmode.telemetry.addLine("initialized");
        opmode.telemetry.update();
    }

    public void goToDetectPosition() {
        strafeDistance(POWER_MAX,DETECT_STRAFE);
        Log.i("OPMODETIME", String.format("goToDetectPosition: \t%f\n",opModeTime.milliseconds()));
    }

    public int detectStonesStatic(boolean side){
        opmode.sleep(1000);
        
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (!updatedRecognitions.isEmpty()) {
                opmode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (side == RED) {
                    switch (updatedRecognitions.size()) {
                        case 1:
                            numberOfStones = 1;
                            Log.i("OPMODETIME", String.format("detectStonesStatic: \t%f\n",opModeTime.milliseconds()));
                            return CENTER;
                        case 2:
                            for (Recognition recognition : updatedRecognitions) {
//                                opmode.telemetry.addData("  left", "%.03f", recognition.getLeft());
                                Log.i("STATIC DETECTION","Stone Left px: " + recognition.getLeft());
//                                opmode.telemetry.addData("  right", "%.03f", recognition.getRight());
                                Log.i("STATIC DETECTION","Stone Right px: " + recognition.getRight());
//                                opmode.telemetry.update();
                                numberOfStones = 2;
                                if (recognition.getLabel().equals("Skystone")) {
                                    if (recognition.getRight() > 485) {
                                        Log.i("OPMODETIME", String.format("detectStonesStatic: \t%f\n",opModeTime.milliseconds()));
                                        return RIGHT;
                                    } else if (recognition.getLeft() < 25){
                                        Log.i("OPMODETIME", String.format("detectStonesStatic: \t%f\n",opModeTime.milliseconds()));
                                        return CENTER;
                                    }
                                }
                            }
                            Log.i("OPMODETIME", String.format("detectStonesStatic: \t%f\n",opModeTime.milliseconds()));
                            return LEFT;
                        default:
                            numberOfStones = 100;
                            Log.i("OPMODETIME", String.format("detectStonesStatic: \t%f\n",opModeTime.milliseconds()));
                            return CENTER;
                    }
                } else {
                    switch (updatedRecognitions.size()) {
                        case 1:
                            numberOfStones = 1;
                            Log.i("OPMODETIME", String.format("detectStonesStatic: \t%f\n",opModeTime.milliseconds()));
                            return CENTER;
                        case 2:
                            numberOfStones = 2;
                            for (Recognition recognition : updatedRecognitions) {
//                                opmode.telemetry.addData("  left", "%.03f", recognition.getLeft());
                                Log.i("STATIC DETECTION","Stone Left px: " + recognition.getLeft());
//                                opmode.telemetry.addData("  right", "%.03f", recognition.getRight());
                                Log.i("STATIC DETECTION","Stone Right px: " + recognition.getRight());

                                if (recognition.getLabel().equals("Skystone")) {
                                    if (recognition.getLeft() < 200) {
                                        Log.i("OPMODETIME", String.format("detectStonesStatic: \t%f\n",opModeTime.milliseconds()));
                                        return LEFT;
                                    } else if(recognition.getRight() > 750){
                                        Log.i("OPMODETIME", String.format("detectStonesStatic: \t%f\n",opModeTime.milliseconds()));
                                        return CENTER;
                                    }
                                }
                            }
                            Log.i("OPMODETIME", String.format("detectStonesStatic: \t%f\n",opModeTime.milliseconds()));
                            return RIGHT;
                        default:
                            numberOfStones = 100;
                            Log.i("OPMODETIME", String.format("detectStonesStatic: \t%f\n",opModeTime.milliseconds()));
                            return CENTER;
                    }
                }
            } else {
                opmode.telemetry.addLine("0 objects detected");
                Log.i("OPMODETIME", String.format("detectStonesStatic: \t%f\n",opModeTime.milliseconds()));
                return CENTER;
            }
        } else {
            opmode.telemetry.addLine("tfod fail");
            Log.i("OPMODETIME", String.format("detectStonesStatic: \t%f\n",opModeTime.milliseconds()));
            return CENTER;
        }
    }
    
    public void collectSkyStone(boolean side, int skystone){
        if(side) {
            switch (skystone) {
                case LEFT:
                    driveDistance(.8,-20);
                    sleepBetweenMotion();
                    collection(side);
                    turnDegree(.67,155);
                    setDeliveryGrabber(true);
                    sleepBetweenMotion();
                    Log.i("OPMODETIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case CENTER:
                    driveDistance(.8,-18);
                    sleepBetweenMotion();
                    collection(side);
                    turnDegree(.67, -50);
                    setDeliveryGrabber(true);
                    sleepBetweenMotion();
                    driveDistance(1,74);
                    sleepBetweenMotion();
                    Log.i("OPMODETIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case RIGHT:
                    driveDistance(.8,-7.5);
                    sleepBetweenMotion();
                    collection(side);
                    turnDegree(.67, -50);
                    setDeliveryGrabber(true);
                    sleepBetweenMotion();
                    driveDistance(1,66);
                    sleepBetweenMotion();
                    Log.i("OPMODETIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
            }
        } else {
            switch (skystone) {
                case LEFT:
                    driveDistance(.8,-22.5);
                    sleepBetweenMotion();
                    collection(side);
                    driveDistance(POWER_MAX,105);
                    sleepBetweenMotion();
                    Log.i("OPMODETIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case CENTER:
                    driveDistance(.8,-14.5);
                    sleepBetweenMotion();
                    collection(side);
                    driveDistance(1,97);
                    sleepBetweenMotion();
                    Log.i("OPMODETIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case RIGHT:
                    driveDistance(.8,-6);
                    sleepBetweenMotion();
                    collection(side);
                    driveDistance(1,89);
                    sleepBetweenMotion();
                    Log.i("OPMODETIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
            }
        }
    }

    public void moveToFoundation(boolean side){
        if (side) {
            turnToDegree(.67, 90);
        } else{
            turnToDegree(.67,-90);
        }
        sleepBetweenMotion();
        driveDistance(POWER_MAX, -7);
        sleepBetweenMotion();
        Log.i("OPMODETIME", String.format("moveToFoundation: \t%f\n",opModeTime.milliseconds()));
    }

    public void deliverSkystone (boolean side) {
        setDeliveryExtender(true);
        setRollerMotors(true, 1);
        opmode.sleep(100);
        stopRollerMotors();
        setDeliveryRotation(true);
        opmode.sleep(1000);
        setDeliveryGrabber(false);
        opmode.sleep(500);
        setDeliveryRotation(false);
        opmode.sleep(100);
        setDeliveryExtender(false);
        sleepBetweenMotion();
        Log.i("OPMODETIME", String.format("deliverSkystone: \t%f\n",opModeTime.milliseconds()));
    }
    
    public void moveFoundation (boolean side) {
        setFoundation(true);
        if (side) {

        } else {
            opmode.telemetry.addLine("red Foundation moving");
            setSelectPowerAll(0, .5, 0, .5);
            opmode.sleep(2500);
            driveDistance(POWER_MAX, -15);
            sleepBetweenMotion();
        }
    }

    public void park (boolean side, boolean state) {
        //Side
        //true = blue
        //false = red
        //State
        //true = wall
        //false = bridge

        if(state == WALL) {
            if (side == BLUE) {
                opmode.telemetry.addLine("Blue wall");
                opmode.telemetry.update();
                driveDistance(.8,10);
                sleepBetweenMotion();
                strafeDistance(.8,16);
                sleepBetweenMotion();
                driveDistance(.8,28);
                sleepBetweenMotion();
            } else if (side == RED) {
                opmode.telemetry.addLine("Red wall");
                opmode.telemetry.update();
                driveDistance(.8,10);
                sleepBetweenMotion();
                strafeDistance(.8,-25);
                sleepBetweenMotion();
                driveDistance(.8,28);
                sleepBetweenMotion();
            }
        } else if (state == BRIDGE) {
            if (side == BLUE) {
                opmode.telemetry.addLine("Blue bridge");
                opmode.telemetry.update();
                driveDistance(.8,10);
                sleepBetweenMotion();
                strafeDistance(.8,-10);
                sleepBetweenMotion();
                driveDistance(.8,28);
                sleepBetweenMotion();
            } else if (side == RED) {
                opmode.telemetry.addLine("Red bridge");
                opmode.telemetry.update();
                /*turnDegree(.4,-1.5);
                opmode.sleep(250);
                driveDistance(1,-1);
                opmode.sleep(250);
                driveDistance(1,35);*/
                /*sleepBetweenMotion();
                driveDistance(1,12);*/

                driveDistance(.8,40);
                opmode.idle();
            }
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
                setSelectPowerAll(drivePower -.22*deltaTheta, drivePower + .22*deltaTheta, drivePower -.22*deltaTheta, drivePower + .22*deltaTheta);
                Log.i("POWER",String.format("Delta Y: %f\tPower: %f\t Compensation: %f\n", deltaY, drivePower,.22*deltaTheta ));


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

                double drivePower = Math.max(.22,calculatePowerStraight(powerLimit, distance, deltaY));
                setSelectPowerAll(-(drivePower -.22*deltaTheta), -(drivePower + .22*deltaTheta), -(drivePower -.22*deltaTheta), -(drivePower + .22*deltaTheta));
                opmode.telemetry.addData("deltaY", encoderArray.getDeltaY());
                Log.i("POWER",String.format("Delta Y: %f\tPower: %f\t Compensation: %f\n", deltaY, -drivePower,.22*deltaTheta ));
//                opmode.telemetry.update();
            }
        }
        stopDriving();
        opmode.telemetry.addLine("motors stopped");
        opmode.telemetry.update();
        Log.i("OPMODETIME", String.format("driveDistance: \t%f\n",opModeTime.milliseconds()));

//        System.out.println(debugString);
//        debugString="";
//        opmode.sleep(500);
    }
    
    public void strafeDistance(double powerLimit, double distance) {
        double deltaX = 0;
        double deltaTheta = 0;
        double strafePower;

        if (distance > 0) {
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

        if (distance > 0) {
            while((deltaX <= distance) && opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaX = encoderArray.getDeltaX();
                deltaTheta = encoderArray.getDeltaTheta();
                strafePower = Math.max(.22,calculatePowerStrafe(powerLimit, distance, deltaX));
                setSelectPowerAll(-strafePower - .02*deltaTheta, strafePower + .02*deltaTheta, strafePower + .02*deltaTheta,-strafePower - .02*deltaTheta);
                Log.i("POWER",String.format("Delta X: %f\tPower: %f\t Compensation: %f\n", deltaX, strafePower,.22*deltaTheta ));
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
                Log.i("POWER",String.format("Delta X: %f\tPower: %f\t Compensation: %f\n", deltaX, -strafePower,.22*deltaTheta ));//                opmode.telemetry.addData("deltaY", deltaX);
//                opmode.telemetry.update();
            }
        }
        stopDriving();
        opmode.telemetry.addLine("motors stopped");
        opmode.telemetry.update();
        Log.i("OPMODETIME", String.format("strafeDistance: \t%f\n",opModeTime.milliseconds()));
//        opmode.sleep(500);

    }

    public void turnToDegree(double powerLimit, double targetDegrees) {
        encoderArray.readEncoderValue();
        encoderArray.updateAll();
        encoderArray.resetAll();

        Log.i("TOTALTHETA", String.format("theta: \t%f\n",encoderArray.theta));

        double initialTheta = encoderArray.theta;
        double deltaTheta = targetDegrees - initialTheta;
        Log.i("RELATIVETHETA",String.format("theta: \t%f\n",deltaTheta));

        if (deltaTheta < 55){
            powerLimit = .4;
        }

        turnDegree(powerLimit, deltaTheta);
        stopDriving();

        Log.i("OPMODETIME", String.format("turnDegree: \t%f\n",opModeTime.milliseconds()));
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
                double turnPower = Math.max(.15,calculatePowerTurn(powerLimit, degrees, deltaTheta));
                setTurnPowerAll(turnPower);
                Log.i("POWER",String.format("Delta Theta: %f\tPower: %f\n", deltaTheta, turnPower));

            }
        } else if (degrees < 0) {
            while ((deltaTheta >= degrees) && opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaTheta = encoderArray.getDeltaTheta();
                double turnPower = Math.max(.15,calculatePowerTurn(powerLimit, degrees, deltaTheta));
                setTurnPowerAll(-turnPower);
                Log.i("POWER",String.format("Delta Theta: %f\tPower: %f\n", deltaTheta, -turnPower));
//                opmode.telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
//                opmode.telemetry.update();
            }
        }

        stopDriving();
        Log.i("OPMODETIME", String.format("turnDegree: \t%f\n",opModeTime.milliseconds()));
//        getAngleTelemetry("TURN END");

    }

    public void runWhileExtending (double powerLimit, double distance, boolean direction) {
        //true = in
        //false = out

        double deltaY = 0;
        double powerMin = POWER_MIN;
        double deltaTheta = 0;

        encoderArray.readEncoderValue();
        encoderArray.updateAll();

        encoderArray.resetAll();
        if (distance > 0) {
            if (powerLimit == POWER_MAX) {
                distance -= 2.24;
            } else if (powerLimit ==.5) {
                distance -= .71;
            } else {
                distance -= 3.39;
            }
        } else if (distance < 0) {
            if (powerLimit == POWER_MAX) {
                distance += 2.24;
            } else if (powerLimit ==.5) {
                distance += .71;
            } else {
                distance += 3.39;
            }
        }


        if (distance > 0) {
//            direction = FORWARD;
            extenderTimeout.reset();

            if (direction)
                deliveryExtender.setPower(1);
            else if (!direction)
                deliveryExtender.setPower(-1);

            while(opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaY = encoderArray.getDeltaY();
                deltaTheta = encoderArray.getDeltaTheta();
                if(extenderTimeout.milliseconds() < EXTEND_OPMODETIMEOUT)
                    deliveryExtender.setPower(0);
                if (deltaY >= distance)
                    break;

                double drivePower = Math.max(.22,calculatePowerStraight(powerLimit, distance, deltaY));
                setSelectPowerAll(drivePower +.22*deltaTheta, drivePower - .22*deltaTheta, drivePower +.22*deltaTheta, drivePower - .22*deltaTheta);
                Log.i("POWER",String.format("Delta Y: %f\tPower: %f\n", deltaY, calculatePowerStraight(powerLimit,distance,deltaY)));


//                telemetryEncoderArray();
                opmode.telemetry.addData("deltaY: ", deltaY);
                opmode.telemetry.update();
//                telemetryWheelPower();
//                debugString.concat(String.format("Y: %f\tpower: %f\n", encoderArray.getDeltaY(), calculatePowerStraight(powerLimit,distance)));
            }
        } else if (distance < 0) {
            extenderTimeout.reset();

            if (direction)
                deliveryExtender.setPower(1);
            else if (!direction)
                deliveryExtender.setPower(-1);

            while(opmode.opModeIsActive()) {
                encoderArray.readEncoderValue();
                deltaY = encoderArray.getDeltaY();
                deltaTheta = encoderArray.getDeltaTheta();
                if(extenderTimeout.milliseconds() < EXTEND_OPMODETIMEOUT)
                    deliveryExtender.setPower(0);
                if (deltaY <= distance)
                    break;

                setDrivePowerAll(-Math.max(.22,calculatePowerStraight(powerLimit, distance, deltaY)));
//                opmode.telemetry.addData("deltaY", encoderArray.getDeltaY());
//                opmode.telemetry.update();
            }
        }
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
        Log.i("OPMODETIME", String.format("setRollerMotors: \t%f\n",opModeTime.milliseconds()));
    }

    public void setRollerMotors (boolean direction, double power) {
        //direction true = outward
        //direction false = inward
        /*rollerTimeout = new ElapsedTime();
        final int ROLLER_OPMODETIMEOUT = time;*/

        if(direction){
            leftRoller.setDirection(DcMotorSimple.Direction.FORWARD);
            rightRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if(!direction){
            leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        Log.i("OPMODETIME", String.format("setRollerMotors: \t%f\n",opModeTime.milliseconds()));

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
        Log.i("OPMODETIME", String.format("setBlockSweeper: \t%f\n",opModeTime.milliseconds()));
        

    }

    public void setDeliveryGrabber (boolean grab) {
        if(grab){
            deliveryGrabber.setPosition(.5);
        } else{
            deliveryGrabber.setPosition(0);
        }
    }

    public void setDeliveryExtender (boolean direction) {
        //true = in
        //false = out
        extenderTimeout.reset();

        while(extenderTimeout.milliseconds() < EXTEND_OPMODETIMEOUT) {
            if (direction)
                deliveryExtender.setPower(-1);
            else if (!direction)
                deliveryExtender.setPower(1);
        }
        deliveryExtender.setPower(0);
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
        opmode.telemetry.addData("deltaTheta(degrees)", encoderArray.getDeltaTheta());
        opmode.telemetry.update();
    }
    
    public void telemetryChasisPower(){
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

    public void sleepBetweenMotion(){
        opmode.sleep(50);
    }

    public void collection(boolean side){
        if(side) {
            turnToDegree(.67, 45);
            setRollerMotors(false, .6);
            opmode.idle();
            driveDistance(.6, 15);
            opmode.idle();
            driveDistance(.6, -15);
            opmode.sleep(300);
            stopRollerMotors();
            setBlockSweeper(true);
            sleepBetweenMotion();
        } else {
//            strafeDistance(.3, -5);
            sleepBetweenMotion();
            turnToDegree(.67, 45);
            sleepBetweenMotion();
            setRollerMotors(false, .6);
            driveDistance(.6, 15);
            opmode.idle();
            driveDistance(.6, -15);
            setBlockSweeper(true);
            sleepBetweenMotion();
            stopRollerMotors();
            turnToDegree(.67, -45);
            setDeliveryGrabber(true);
            sleepBetweenMotion();
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