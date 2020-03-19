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


public class SSDriveObject extends Object{
    /*Servo hookHrz, hookVrt, deliveryGrabber, deliveryRotation, cameraServo, leftFoundation, rightFoundation, blockSweeper, capServo, deliveryExtender;
    DcMotor frontRight, frontLeft, backRight, backLeft, rightRoller, leftRoller;
    
    Encoder myLeft, myRight, myCenter;
    EncoderArray drive.encoderArray;
    ElapsedTime elapsedTime;
    OpenCvCamera webcam;*/
    LinearOpMode opmode;
    BaseDriveObject drive;
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

    public SSDriveObject(LinearOpMode parent, BaseDriveObject d){
        drive = d;
        opmode = parent;

        

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

        drive.capServo.setPosition(CAP_INIT_POS);
        drive.setFoundation(FUP);
        drive.setDeliveryExtender(IN);
        drive.setBlockSweeper(OUT);
        drive.setDeliveryGrabber(OUT);
        drive.setCameraServo(CAM_INIT_POS);
        drive.setDeliveryRotation(IN);
        drive.setBlockSweeper(IN);

        skystone = detectStonesStatic(RED);
        opmode.telemetry.addLine(skystoneString(skystone));

        drive.encoderArray.resetAll();

        opmode.telemetry.addLine("initialized");
        opmode.telemetry.update();
    }

    public int detectStonesStatic(boolean side){
        int cameraMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
        drive.webcam = OpenCvCameraFactory.getInstance().createWebcam(opmode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        drive.webcam.openCameraDevice();
        drive.webcam.setPipeline(detector);
        drive.webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        opmode.telemetry.addData("is started", opmode.isStarted());

        while (!opmode.isStarted()) {
            position = detector.position;
            opmode.telemetry.addData("position", position);
            opmode.telemetry.update();
            Log.i("OPENCV", String.format("drive.driveDistance: \t%s\n",position));
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
        drive.turnDegree(.67, 65);


        drive.setRollerMotors(IN, .6);
        drive.driveDistance(.8, 15);


        drive.driveDistance(.8, -15);

        drive.setBlockSweeper(IN);
        drive.stopRollerMotors();
        drive.turnToDegree(.67, 0);
        drive.setDeliveryGrabber(IN);
        drive.setBlockSweeper(OUT);
        Log.i("FINALTHETA",String.format("FinalTheta after turn back: %f", (drive.encoderArray.theta)));
    }

    public void collectSkyStone(boolean side, int skystone){
        drive.setBlockSweeper(OUT);
        if(side == BLUE) {
            drive.strafeDistance(POWER_MAX,DETECT_STRAFE);
            switch (skystone) {
                case LEFT_SS:
                    drive.driveDistance(.8, BLOCK_LENGTH-1);
                    opmode.idle();
                    collectionBlueLeft();
                    opmode.idle();
                    drive.turnDegree(.3,.01);
                    drive.encoderArray.theta=0;
                    drive.driveDistance(1, - (92));
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case CENTER_SS:
                    drive.driveDistance(.8, -(7 + BLOCK_LENGTH));
                    opmode.idle();
                    collection(side);

                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));

                    drive.encoderArray.theta = 0;
                    drive.driveDistance(1, - (60 + BLOCK_LENGTH));
                    break;
                case RIGHT_SS:
                    drive.driveDistance(.8,-7);
                    opmode.idle();
                    collection(side);

                    drive.driveDistance(1,-(60 + BLOCK_LENGTH*2));
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));

                    break;
            }
        } else if (side == RED) {
            drive.strafeDistance(POWER_MAX,DETECT_STRAFE);
            Log.i("FINALTHETA",String.format("FinalTheta after strafe: %f", (drive.encoderArray.theta)));
            switch (skystone) {
                case LEFT_SS:
                    drive.driveDistance(1,-22.5);
                    opmode.idle();
                    collection(side);
                    drive.driveDistance(1,90 + BLOCK_LENGTH*2);
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case CENTER_SS:
                    drive.driveDistance(1,-13);//.5 -> 1
                    Log.i("FINALTHETA",String.format("FinalTheta after very first drive to collect: %f", (drive.encoderArray.theta)));
                    opmode.idle();
                    collection(side);
                    drive.driveDistance(1,90 + BLOCK_LENGTH);
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case RIGHT_SS:
                    drive.driveDistance(1,-7);
                    opmode.idle();
                    collectionRightRed();
                    drive.driveDistance(1,90);
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
            }
        }
    }

    public void moveToFoundation(boolean side){
        if (side == BLUE) {
            drive.turnDegree(.67, -88);
        } else if (side == RED){
            drive.turnDegree(.67,-92);
        }

//        drive.turnToDegree(.67, -90);
        Log.i("THETA", String.format("delta theta: \t%f\n", drive.encoderArray.getDeltaTheta()));
        Log.i("THETA", String.format("theta: \t%f\n", drive.encoderArray.theta));

        drive.driveDistance(.5, -7.5);
        //Log.i("ACTIONTIME", String.format("moveToFoundation: \t%f\n",opModeTime.milliseconds()));
    }

    public void moveFoundation (boolean side) {
//        drive.turnDegree(.3,-0.1);
        drive.setFoundation(FDOWN);
        opmode.sleep(400);//500 -> 300
        drive.driveDistance(1, 22.5);
        drive.setRollerMotors(OUT,1);
        arcWhileExtending(.67, side);
        drive.stopRollerMotors();
        opmode.idle();
        drive.driveDistanceWithTime(1, -13,2000);
        opmode.sleep(200);
        drive.setDeliveryGrabber(OUT);

        //Log.i("ACTIONTIME", String.format("moveFoundation: \t%f\n",opModeTime.milliseconds()));
    }

    public void moveToSecondSS(boolean side, boolean state){
        drive.setFoundation(FUP);
        drive.setBlockSweeper(OUT);

        drive.setDeliveryRotation(IN);
        if(state == WALL) {
            if (side == BLUE) {
                drive.strafeDistance(.8, 19);
            } else if (side == RED) {
                drive.strafeDistance(.8, -26);
            }
        } else if (state == BRIDGE) {
            if (side == BLUE) {
                drive.strafeDistance(.8, -2);
                drive.turnDegree(.3,.1);
            } else if (side == RED) {
                drive.strafeDistance(.8, -2);
            }
        }
    }

    public void collectSecondSkyStone(boolean side){
        drive.setDeliveryExtender(IN);
        if(side == BLUE) {
            switch (skystone) {
                case LEFT_SS:
                    drive.driveDistance(1,62);
                    opmode.idle();
                    collectionBlueLeft();
                    drive.driveDistance(1,-(62));
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case CENTER_SS:
                    drive.driveDistance(1,80 + BLOCK_LENGTH);
                    opmode.idle();
                    collection2(side);
                    drive.driveDistance(1,-(80 + BLOCK_LENGTH));
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case RIGHT_SS:
                    drive.driveDistance(1,80 + BLOCK_LENGTH*2);
                    opmode.idle();
                    collection2(side);
                    drive.driveDistance(1,-(80+ BLOCK_LENGTH*2));
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
            }
        } else if (side == RED) {
            switch (skystone) {
                case LEFT_SS:
                    drive.driveDistance(1,79 + BLOCK_LENGTH*2);
                    opmode.idle();
                    collection2(side);
                    drive.driveDistance(POWER_MAX,52 + BLOCK_LENGTH*2);
                    drive.setRollerMotors(OUT, 1);
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case CENTER_SS:
                    drive.driveDistance(1,80 + BLOCK_LENGTH);
                    Log.i("FINALTHETA",String.format("FinalTheta after very first drive to collect: %f", (drive.encoderArray.theta)));
                    opmode.idle();
                    collection2(side);
                    //drive.driveDistance(1,-88);
                    drive.driveDistance(1,52 + BLOCK_LENGTH);
                    drive.setRollerMotors(OUT, 1);
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
                case RIGHT_SS:
                    drive.driveDistance(1,78);
                    opmode.idle();
                    collection2(side);
                    drive.driveDistance(1,52);
                    drive.setRollerMotors(OUT, 1);
                    opmode.idle();
                    Log.i("ACTIONTIME", String.format("collectSkystone: \t%f\n",opModeTime.milliseconds()));
                    break;
            }
        }
    }

    public void deliverSkystone (boolean side) {
        drive.setDeliveryRotation(OUT);
        opmode.sleep(300);
        drive.setDeliveryGrabber(OUT);
    }

    public void park (boolean side, boolean state) {
        if (side == RED) {
        drive.setRollerMotors(OUT,0);
        drive.setBlockSweeper(IN);
        drive.driveDistance(.8,-15);
        } else if (side == BLUE) {
            drive.setDeliveryExtender(OUT);
            opmode.sleep(750);
            deliverSkystone(BLUE);
            drive.strafeDistance(.5,-.1);
            drive.driveDistance(1,35);

        }
        drive.setDeliveryRotation(IN);
        drive.setDeliveryExtender(IN);
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

    

    public void arcWhileExtending (double powerLimit, boolean side) {
       //true = in
       //false = out

       double deltaY = 0;
       double deltaTheta = 0;

       drive.encoderArray.readEncoderValue();
       
       drive.encoderArray.resetAll();

       drive.setDeliveryExtender(OUT);

       if (side == RED) {
           while(opmode.opModeIsActive()) {
               drive.encoderArray.readEncoderValue();
               deltaTheta = drive.encoderArray.getDeltaTheta();

               if (deltaTheta <= -90)
                   break;

               drive.setSelectPowerAll(-1, 1, 0, 0);
               //Log.i("POWER",String.format("Delta Y: %f\tPower: %f\n", deltaY, calculatePowerStraight(powerLimit,distance,deltaY)));
//                telemetryEncoderArray();
           }
       } else {
           while(opmode.opModeIsActive()) {
               drive.encoderArray.readEncoderValue();
               deltaY = drive.encoderArray.getDeltaY();
               deltaTheta = drive.encoderArray.getDeltaTheta();
               if (deltaTheta >= 90)
                   break;

               drive.setSelectPowerAll(1, -1, 0, 0);
           }
       }
       drive.setDeliveryRotation(OUT);
       drive.stopDriving();
   }

    //set chassis motors


    public void collectionRightRed() {
        drive.turnToDegree(.67, 45);
        Log.i("FINALTHETA",String.format("FinalTheta after turn in: %f", (drive.encoderArray.theta)));

        drive.setRollerMotors(IN, .6);
        drive.driveDistance(1, 20);
        Log.i("FINALTHETA",String.format("FinalTheta after drive forward: %f", (drive.encoderArray.theta)));

        drive.driveDistance(1, -20);
        Log.i("FINALTHETA",String.format("FinalTheta after drive back: %f", (drive.encoderArray.theta)));
        drive.setBlockSweeper(IN);

        drive.stopRollerMotors();


        drive.turnToDegree(.67, 0);
        drive.setBlockSweeper(OUT);
        Log.i("FINALTHETA",String.format("FinalTheta after turn back: %f", (drive.encoderArray.theta)));
        drive.setDeliveryGrabber(IN);
    }

    public void collection(boolean side){
        if(side == BLUE) {
            /*drive.turnDegree(.67, 45);
            drive.setRollerMotors(IN, .6);
            drive.driveDistance(1, 15);
            drive.driveDistance(1, -15);
            drive.setBlockSweeper(IN);
            drive.stopRollerMotors();
            drive.turnToDegree(.67, 0);
            drive.setBlockSweeper(OUT);
            drive.setDeliveryGrabber(IN);*/
            drive.strafeDistance(.8, -11);
            drive.setRollerMotors(IN, .6);
            drive.driveDistance(.5, 5);
            drive.driveDistance(.5, -5);
            opmode.idle();
            drive.setBlockSweeper(IN);
            drive.strafeDistance(.8, 11.5);
            drive.setDeliveryGrabber(IN);
            drive.setBlockSweeper(OUT);

        } else if(side == RED) {
//            drive.strafeDistance(.3, -5);

            drive.turnDegree(.67, 45);
            Log.i("FINALTHETA",String.format("FinalTheta after turn in: %f", (drive.encoderArray.theta)));

            drive.setRollerMotors(IN, .6);
            drive.driveDistance(1, 15);
            Log.i("FINALTHETA",String.format("FinalTheta after drive forward: %f", (drive.encoderArray.theta)));

            drive.driveDistance(1, -15);
            Log.i("FINALTHETA",String.format("FinalTheta after drive back: %f", (drive.encoderArray.theta)));
            drive.setBlockSweeper(IN);
            drive.stopRollerMotors();
            drive.turnToDegree(.67, 0);
            drive.setDeliveryGrabber(IN);
            drive.setBlockSweeper(OUT);
            Log.i("FINALTHETA",String.format("FinalTheta after turn back: %f", (drive.encoderArray.theta)));

        }
    }

    public void collection2 (boolean side){
        if(side == BLUE) {
            drive.strafeDistance(.8, -14);
            drive.setRollerMotors(IN, .6);
            drive.driveDistance(.5, 5);
            drive.driveDistance(.5, -5);
            opmode.idle();
            drive.setBlockSweeper(IN);
            drive.strafeDistance(.8, 14);
            drive.setDeliveryGrabber(IN);
            drive.setBlockSweeper(OUT);
        } else if(side == RED) {
//            drive.strafeDistance(.3, -5);
            drive.turnDegree(.67, -45);
            Log.i("FINALTHETA",String.format("FinalTheta after turn in: %f", (drive.encoderArray.theta)));

            drive.setRollerMotors(IN, .6);
            drive.driveDistance(1, 15);
            Log.i("FINALTHETA",String.format("FinalTheta after drive forward: %f", (drive.encoderArray.theta)));

            drive.driveDistance(1, -15);
            Log.i("FINALTHETA",String.format("FinalTheta after drive back: %f", (drive.encoderArray.theta)));

            //drive.setBlockSweeper(IN);
            drive.stopRollerMotors();
            drive.turnDegree(.67, -135);
            Log.i("FINALTHETA",String.format("FinalTheta after turn back: %f", (drive.encoderArray.theta)));
            //drive.setDeliveryGrabber(true);
            //drive.setDeliveryExtender(OUT);
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