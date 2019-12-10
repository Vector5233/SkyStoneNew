package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="SSTestAutoOp", group="TeamCode")

public class SSTestAutoOp extends LinearOpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft, rightRoller, leftRoller;
    Servo hookHrz, hookVrt, deliveryGrabber, deliveryRotation, camera, leftFoundation, blockSweeper;
    CRServo deliveryExtender;
    LinearOpMode opmode;
    ModernRoboticsI2cGyro gyro;
    SSDriveObject drive;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    final double WEBCAM_TO_BLOCKS = 9.5;

    final double CENTER_PIXELS = 400;
    final double BLOCK_LENGTH = 8;
    final double ARM_TO_WEBCAM = 9;

    final int TFOD_TIMEOUT = 500;

    double inchPerPixel = 0;

    double LS_leftPixel = 0;
    double LS_rightPixel = 0;

    double RS_leftPixel = 0;
    double RS_rightPixel = 0;

    double SS_leftPixel = 0;
    double SS_rightPixel = 0;

    double displacement = 0;
    double secondDisplacement = displacement - 24;

    boolean isVirtual = false;

    private ElapsedTime tfodTimeout;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

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

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AbLVQDn/////AAABma+zV9cCqU+7pLBUXgQ3J6II1u1B8Vg4mrnGfVawPjc1l7C6GWoddOaL6Wqj5kXPBVUh3U3WND38234Tm0h3+LKmmTzzaVPRwOk3J+zBwKlOvv93+u7chctULk8ZYEyf0NuuEfsGwpgJx7xL9hIFBoaB2G1SpbJIt+n94wz6EvfRYSusBEiST/lUqgDISIlaeOLPWEipHh46axomcrGVRRl09pg6pCt2h7rU6us+guN5nKhupTXvM+BTUYW3kCO9YsUjz16jLr7GyFh8wVQbRS3dikSX7kzVsdkLjZnJdyinYaB5oDXfmmXtaC6ZXeD6vKs62vpaydAq9VGAlCtnSyq2J4NLI+LOIOvdtsCwarfS";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        drive.setHookHrz(0.5);
        drive.setHookVrt(0.4);

        initTfod();

        if (tfod != null) {
            tfod.activate();
        }
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.strafeDistance(1, -24.5, 1000);
        detectStones();
        telemetry.addData("  SS left", "%.03f", SS_leftPixel);
        telemetry.addData(

                "  SS right", "%.03f", SS_rightPixel);
        telemetry.update();
        sleep(1000);

        getDisplacement();

        collectSkyStone();

        //drive till fnd
        //find navi img
        //cw 90
        //deliver
        //ccw 90
        //drive back til block
        //collect
        //drive till fnd
        //cw 90
        //deliver
        //strafe till line
        //park
        //init for teleop
    }

    public void initTfod(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void detectStones(){
        tfodTimeout = new ElapsedTime();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        int i = 0;
                        boolean skyFlag = false;

                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData("  left", "%.03f", recognition.getLeft());
                            telemetry.addData("  right", "%.03f", recognition.getRight());

                            if(recognition.getLabel().equals("Skystone")){
                                SS_leftPixel = recognition.getLeft();
                                SS_rightPixel = recognition.getRight();
                                skyFlag = true;
                                break;
                            }
                        }
                        telemetry.update();
                        sleep(1000);

                        if(!skyFlag) {
                            isVirtual = true;
                            createVirtualStone(updatedRecognitions.get(0).getRight(), updatedRecognitions.get(0).getLeft(), updatedRecognitions.get(1).getRight(), updatedRecognitions.get(0).getLeft());
                        }
                    }
                }
                if (tfodTimeout.milliseconds() >= TFOD_TIMEOUT) {
                    tfod.shutdown();
                    telemetry.addLine("Tfod Terminated");
                    telemetry.update();
                    break;
                }
            }
        }
    }

    public void createVirtualStone(double S1_rightPixel, double S1_leftPixel, double S2_rightPixel, double S2_leftPixel){
        double S1_size = S1_rightPixel - S1_leftPixel;
        double S2_size = S2_rightPixel - S2_leftPixel;

        SS_leftPixel = Math.max(S1_rightPixel, S2_rightPixel);
        SS_rightPixel = SS_leftPixel + (S1_size + S2_size)/2;

        telemetry.addLine("Skystone virtually created");
        telemetry.update();
    }

    public void getDisplacement(){
        inchPerPixel = Math.abs(BLOCK_LENGTH/(SS_rightPixel - SS_leftPixel));
        telemetry.addData("inch / pixel", "%.03f", inchPerPixel);

        //displacement = ((5/8)*(SS_rightPixel - SS_leftPixel) + SS_leftPixel - CENTER_PIXELS)*inchPerPixel - ARM_TO_WEBCAM;
        if(250 <= SS_rightPixel && SS_rightPixel <= 500) {
            telemetry.addLine("right");
            displacement = inchPerPixel * (SS_rightPixel - CENTER_PIXELS) - ARM_TO_WEBCAM + 13;
        }
        else if(600 <= SS_rightPixel && SS_rightPixel <= 800) {
            telemetry.addLine("left");
            displacement = inchPerPixel * (SS_leftPixel - CENTER_PIXELS) - ARM_TO_WEBCAM + 5;
        }
        else if(isVirtual) {
            telemetry.addLine("center");
            displacement = inchPerPixel * (CENTER_PIXELS + SS_rightPixel - 2 * SS_leftPixel) - ARM_TO_WEBCAM + 15;
        }

        telemetry.addData("displacement", "%.03f", displacement);
        telemetry.update();
        sleep(1000);
    }

    public void collectSkyStone(){
        hookVrt.setPosition(0.6);
        hookHrz.setPosition(1);

        sleep(500);
        drive.driveDistance(0.5, displacement, 1000);
        sleep(200);

        drive.setHookVrt(0.4);
        leftRoller.setPower(1);
        rightRoller.setPower(1);
        drive.setHookHrz(0);
        leftRoller.setPower(0);
        rightRoller.setPower(0);
        sleep(500);

        drive.setBlockSweeper(true);
        sleep(500);
        drive.setBlockSweeper(false);
    }
}
