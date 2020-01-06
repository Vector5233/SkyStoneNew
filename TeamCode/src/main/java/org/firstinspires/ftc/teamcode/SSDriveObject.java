package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/* TODO
   To prevent robot from lumping at the endpoint
   1) make function that makes the robot smoothly accelerating stars and stops
   2) lowering the power
   3) Having a function that calculates the ticks for a given distance

 */

public class SSDriveObject extends Object{
    Servo hookHrz, hookVrt, deliveryGrabber, deliveryRotation, cameraServo, leftFoundation, rightFoundation, blockSweeper, capServo;
    CRServo deliveryExtender;
    DcMotor frontRight, frontLeft, backRight, backLeft, rightRoller, leftRoller;
    LinearOpMode opmode;
    Encoder left;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    final double ROBOT_RADIUS = 13.5;
    final double TICKS_PER_INCH_STRAIGHT = (383.6*2) / (4 * 3.14159265358979323846264);
    final double TICKS_PER_INCH_TURN = TICKS_PER_INCH_STRAIGHT;
    final double TICKS_PER_INCH_STRAFE = (TICKS_PER_INCH_STRAIGHT)*1.15;
    final double TICKS_PER_DEGREE = (3.14159 / 180) *  ROBOT_RADIUS * TICKS_PER_INCH_TURN;
    final double TOLERANCE = 2;  // in degrees
    final double MAXSPEED = 0.65;

    final boolean BLUE = true;
    final boolean RED = false;

    final boolean FOUNDATION = false;
    final boolean NORMAL = true;

    final double WEBCAM_TO_BLOCKS = 9.5;

    final double CENTER_PIXELS = 400.0;
    final double BLOCK_LENGTH = 8.0;
    final double ARM_TO_WEBCAM = 5.875;
    final double FRONT_CENTER_TO_WEBCAM = 3.5;
    final double ROBOT_CENTER_TO_COLLECTOR = 10;

    final int TFOD_TIMEOUT = 500;

    //final double TOLERANCE = ??;
    //final double ROOT2 = 1.414;
    //final int CAMERA_MIDPOINT = ??;
    //final int SAMPLING_FORWARD = ?;

    double inchPerPixel;

    double SS_leftPixel;
    double SS_rightPixel;

    double displacement;
    double secondDisplacement = displacement - 24;

    boolean isVirtual = false;

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    ElapsedTime tfodTimeout;
    ElapsedTime rollerTimeout;

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
        setBlockSweeper(true);
        setCameraServo(1);
        setHookVrt(.9);
        setHookHrz(0.11);



        opmode.telemetry.addLine("initialized");
        opmode.telemetry.update();
    }

    public void detectReady(){

//            setHookHrz(0.5);
//            setHookVrt(1);
//            opmode.sleep(500);

            driveDistance(1, 23);

    }

    public void collectSkyStone(boolean side){
        if(side) {
            /*strafeDistance(1, displacement);
            setHookVrt(0);
            opmode.sleep(500);
            driveDistance(1, -5);
            setHookHrz(0);*/
            turnDegree(.67,-90);
            driveDistance(1,displacement - ROBOT_CENTER_TO_COLLECTOR);
            strafeDistanceNoAccel(1,-10);
            
        } else{

        }
        setRollerMotors(true, 1);
        //how to check if the block is collected or not (next round)
        driveDistance(1,6);
        opmode.sleep(400);
        setBlockSweeper(true);
        opmode.sleep(500);
        setBlockSweeper(false);
        opmode.idle();
        setRollerMotors(true,0.0);
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

    public void moveFoundation (boolean side) {
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



        if (side) {
            strafeDistanceNoAccel(1, 18);
            opmode.sleep(400);
            setFoundation(true);
            opmode.sleep(400);
            driveDistance(1, 26.75);
            opmode.sleep(400);
            setFoundation(false);
            opmode.sleep(400);
            driveDistance(1,-1);
            opmode.sleep(400);
            strafeDistance(1, -38);
            opmode.sleep(400);
            turnDegree(.67,78);
            opmode.sleep(400);
            strafeDistance(1,-27);
            opmode.sleep(400);
            driveDistance(1,-10);

        } else if (!side) {
            opmode.telemetry.addLine("red Foundation moving");
            driveDistance(1, -22);
            opmode.sleep(250);
            setFoundation(true);
            opmode.sleep(500);
            strafeDistanceNoAccel(1,-5.5);

            opmode.sleep(400);
            driveDistance(1, 25.75);

            opmode.sleep(400);
            turnDegree(.67,-7.5);

            opmode.sleep(400);
            setFoundation(false);
            opmode.sleep(400);
//            driveDistance(1,-1.1);
//            opmode.sleep(400);
            strafeDistance(1, 40);
            opmode.sleep(400);
            driveDistance(1,-12);
            opmode.sleep(400);
            turnDegree(.67,7);
            opmode.sleep(400);
            strafeDistance(1,13);
//            opmode.sleep(400);
//            turnDegree(.67,-78);
//            opmode.sleep(400);
//            strafeDistance(1,20);
//            opmode.sleep(400);
//            driveDistance(1,-12);

        }






        //turnDegree(.4, -5);

        /*turnDegree(.4,1.5);
        opmode.sleep(400);

        driveDistance(1,-1);
        opmode.sleep(400);

        strafeDistanceNoAccel(1,-39);
        opmode.sleep(400);

        turnDegree(.67,90);
        opmode.sleep(400);

        strafeDistanceNoAccel(.8,23);
        opmode.sleep(400);

        driveDistance(.5,-15);
        //park(side, FOUNDATION);*/
    }

    public void park (boolean side, boolean state) {
        //Side
        //true = blue
        //false = red
        //State
        //true = normal
        //false = foundation

        if(state) {
            if (side) {
                opmode.telemetry.addLine("Blue normal");
                opmode.telemetry.update();
                driveDistance(1, -53);
            } else {
                opmode.telemetry.addLine("Red normal");
                opmode.telemetry.update();
                driveDistance(1,-53);
            }
        } else {
            if (side) {
                opmode.telemetry.addLine("Blue foundation");
                opmode.telemetry.update();

                opmode.sleep(400);
                driveDistance(1,13);
            } else {
                opmode.telemetry.addLine("Red foundation");
                opmode.telemetry.update();
                /*turnDegree(.4,-1.5);
                opmode.sleep(250);
                driveDistance(1,-1);
                opmode.sleep(250);
                driveDistance(1,35);*/
                /*opmode.sleep(400);
                driveDistance(1,12);*/
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
            strafeDistanceNoAccel(1,25);
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

    public void initTfod(){
        int tfodMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void detectStones(){
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
        double powerMin = 0.22;
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

    public void driveDistanceNoAccel(double power, double distance) {
        int ticks = (int) (distance * TICKS_PER_INCH_STRAIGHT);


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.telemetry.addLine("Encoders reset");
        opmode.telemetry.update();
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (ticks > 0) {
            while((frontLeft.getCurrentPosition() <= ticks) && opmode.opModeIsActive()) {
                setDrivePowerAll(power);
                opmode.telemetry.addLine("cruising");
                telemetryDcMotor();

            }
        } else if (ticks < 0) {
            power = -power;
            while((frontLeft.getCurrentPosition() >= ticks) && opmode.opModeIsActive()) {
                setDrivePowerAll(power);
                opmode.telemetry.addLine("cruising");
                telemetryDcMotor();
            }
        }
    }

    public void driveDistance(double powerLimit, double distance) {
        final double PERCENT = .1;
        double powerMin = 0.22;
        int ticks = (int) (distance * TICKS_PER_INCH_STRAIGHT);


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.telemetry.addLine("Encoders reset");
        opmode.telemetry.update();
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (ticks > 0) {
            while((frontLeft.getCurrentPosition() <= ticks) && opmode.opModeIsActive()) {
                if (frontLeft.getCurrentPosition() < PERCENT * ticks) {
                    setDrivePowerAll(Math.max((1 / PERCENT) * powerLimit * frontLeft.getCurrentPosition() / ticks, powerMin));
                    opmode.telemetry.addLine("accelerating");

                } else if (frontLeft.getCurrentPosition() < (1 - PERCENT) * ticks) {
                    setDrivePowerAll(powerLimit);
                    opmode.telemetry.addLine("cruising");

                } else {
                    setDrivePowerAll(Math.max(-(1 / PERCENT) * powerLimit * (frontLeft.getCurrentPosition() - ticks) / ticks, powerMin));
                    opmode.telemetry.addLine("decelerating");

                }
                telemetryDcMotor();
                opmode.telemetry.update();
            }
        } else if (ticks < 0) {
            powerLimit = -powerLimit;
            powerMin = -powerMin;
            while((frontLeft.getCurrentPosition() >= ticks) && opmode.opModeIsActive()) {
                if (frontLeft.getCurrentPosition() > PERCENT * ticks) {
                    setDrivePowerAll(Math.min((1 / PERCENT) * powerLimit * frontLeft.getCurrentPosition() / ticks, powerMin));
                    opmode.telemetry.addLine("accelerating");

                } else if (frontLeft.getCurrentPosition() > (1 - PERCENT) * ticks) {
                    setDrivePowerAll(powerLimit);
                    opmode.telemetry.addLine("cruising");

                } else {
                    setDrivePowerAll(Math.min(-(1 / PERCENT) * powerLimit * (frontLeft.getCurrentPosition() - ticks) / ticks, powerMin));
                    opmode.telemetry.addLine("decelerating");

                }
                telemetryDcMotor();
                opmode.telemetry.update();
            }
        }
        stopDriving();
        opmode.telemetry.addLine("done driving");
    }


    public void strafeDistanceNoAccel(double powerLimit, double distance) {
        final double PERCENT = .25;
        final double STRAFECORRECTION = 30.0/37.0;
        double powerMin = 0.3;
        int ticks = (int) (distance * TICKS_PER_INCH_STRAFE * STRAFECORRECTION);
        opmode.telemetry.addData("ticks", ticks);
        opmode.telemetry.update();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.telemetry.addLine("Encoders reset");
        opmode.telemetry.update();
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (ticks > 0) {
            while((frontLeft.getCurrentPosition() <= ticks) && opmode.opModeIsActive()) {
                setStrafePowerAll(powerLimit);
                telemetryDcMotor();
                opmode.telemetry.update();
            }
        } else if (ticks < 0) {
            powerLimit = -powerLimit;
            powerMin = -powerMin;
            while((frontLeft.getCurrentPosition() >= ticks) && opmode.opModeIsActive()) {
                setStrafePowerAll(powerLimit);
                telemetryDcMotor();
                opmode.telemetry.update();
            }
        }

        stopDriving();
    }

    public void strafeDistance(double powerLimit, double distance) {
        final double PERCENT = .25;
        final double STRAFECORRECTION = 30.0/37.0;
        double powerMin = 0.22;
        int ticks = (int) (distance * TICKS_PER_INCH_STRAFE * STRAFECORRECTION);
        opmode.telemetry.addData("ticks", ticks);
        opmode.telemetry.update();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.telemetry.addLine("Encoders reset");
        opmode.telemetry.update();
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (ticks > 0) {
            while((frontLeft.getCurrentPosition() <= ticks) && opmode.opModeIsActive()) {
                if (frontLeft.getCurrentPosition() < PERCENT * ticks) {
                    setStrafePowerAll(Math.max((1 / PERCENT) * powerLimit * frontLeft.getCurrentPosition() / ticks, powerMin));
                    opmode.telemetry.addLine("accelerating");
                    //telemetryDcMotor();
                    telemetryWheelPower();
                } else if (frontLeft.getCurrentPosition() < (1 - PERCENT) * ticks) {
                    setStrafePowerAll(powerLimit);
                    opmode.telemetry.addLine("cruising");
                    //telemetryDcMotor();
                    telemetryWheelPower();
                } else {
                    setStrafePowerAll(Math.max(-(1 / PERCENT) * powerLimit * (frontLeft.getCurrentPosition() - ticks) / ticks, powerMin));
                    opmode.telemetry.addLine("decelerating");
                    //telemetryDcMotor();
                    telemetryWheelPower();
                }
                telemetryDcMotor();
                opmode.telemetry.update();
            }
        } else if (ticks < 0) {
            powerLimit = -powerLimit;
            powerMin = -powerMin;
            while((frontLeft.getCurrentPosition() >= ticks) && opmode.opModeIsActive()) {
                if (frontLeft.getCurrentPosition() > PERCENT * ticks) {
                    setStrafePowerAll(Math.min((1 / PERCENT) * powerLimit * frontLeft.getCurrentPosition() / ticks, powerMin));
                    opmode.telemetry.addLine("accelerating");

                } else if (frontLeft.getCurrentPosition() > (1 - PERCENT) * ticks) {
                    setStrafePowerAll(powerLimit);
                    opmode.telemetry.addLine("cruising");

                } else {
                    setStrafePowerAll(Math.min(-(1 / PERCENT) * powerLimit * (frontLeft.getCurrentPosition() - ticks) / ticks, powerMin));
                    opmode.telemetry.addLine("decelerating");

                }
                telemetryDcMotor();
                opmode.telemetry.update();
            }
        }

        stopDriving();
    }

    public void turnDegree(double power, double degrees) {
        // distance in inches
        //conjecture instead of moving 12", wheels will go 12"*cos(45)= 8.5"

        int ticks = (int) (TICKS_PER_DEGREE * degrees);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.telemetry.addLine("Encoders reset");
        opmode.telemetry.update();
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        getAngleTelemetry("TURN START");


        if (ticks > 0) {
            // positive (CCW) turn => left motor goes in (-) direction
            while ((frontLeft.getCurrentPosition() >= -ticks) && opmode.opModeIsActive()) {
                setTurnPowerAll(-power);
                opmode.telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
                opmode.telemetry.update();
            }
        } else if (ticks < 0) {
            while ((frontLeft.getCurrentPosition() <= -ticks) && opmode.opModeIsActive()) {
                setTurnPowerAll(power);
                opmode.telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
                opmode.telemetry.update();
            }
        }

        stopDriving();
        getAngleTelemetry("TURN END");
    }

    //set chassis motor

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
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    public void setTurnPowerAll(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    //set dc motors

    public void setRollerMotors (boolean direction, double power/*, int time*/) {
        //direction true = forward
        //direction false = backward
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

    //set servos

    public void setHookHrz (double position) {
        hookHrz.setPosition(position);
    }

    public void setHookVrt (double position) {
        hookVrt.setPosition(position);
    }

    public void setCameraServo (double position) {
        cameraServo.setPosition(position);
    }

    /*public void setFoundationRight (boolean launch) {
        //launch true = grabber down
        //launch false = grabber up
        if (launch) {
            rightFoundation.setPosition(0);
        }
        else if (!launch) {
            rightFoundation.setPosition(0.7);
        }
    }*/

    public void setFoundation (boolean launch) {
        //launch true = grabber down
        //launch false = grabber up
        if (!launch) {
            leftFoundation.setPosition(0);
            rightFoundation.setPosition(.5);
        }
        else {
            leftFoundation.setPosition(0.5);
            rightFoundation.setPosition(0);
        }
    }

    

    public void setBlockSweeper (boolean kick) {
        //kick true = blockSweeper up
        //kick false = blockSweeper down

        if (kick) {
            blockSweeper.setPosition(0.95);

        } else if (!kick) {
            blockSweeper.setPosition(0.25);

        }
    }

    public void setDeliveryGrabber (boolean grab) {
        if(grab){
            deliveryGrabber.setPosition(.5);
        } else{
            deliveryGrabber.setPosition(0);
        }
    }

    public void setDeliveryRotation (boolean rotate) {
        //rotate true = rotate out
        //rotate false = rotate in

        if (rotate) {
            deliveryRotation.setPosition(1);
        }
        else {
            deliveryRotation.setPosition(0);
        }
    }

    public void setDeliveryExtender (double power, int time) {
        deliveryExtender.setPower(power);
    }

    //telemetry

    public void telemetryDcMotor(){
        opmode.telemetry.addData("FR", frontRight.getCurrentPosition());
        opmode.telemetry.addData("FL", frontLeft.getCurrentPosition());
        opmode.telemetry.addData("BR", backRight.getCurrentPosition());
        opmode.telemetry.addData("BL", backLeft.getCurrentPosition());
        opmode.telemetry.update();
    }

    public void telemetryWheelPower(){
        opmode.telemetry.addData("FR", frontRight.getPower());
        opmode.telemetry.addData("FB", frontLeft.getPower());
        opmode.telemetry.addData("BR", backRight.getPower());
        opmode.telemetry.addData("BL", backLeft.getPower());
        opmode.telemetry.update();
    }

    public void getAngleTelemetry (String status){
        opmode.telemetry.addLine(status);
        opmode.telemetry.addData("   encoder: ", frontLeft.getCurrentPosition()/TICKS_PER_DEGREE);
        opmode.telemetry.update();
    }
    public void telemetryEncoder (){
        opmode.telemetry.addData("x",left.getDisplacement());

    }
}
