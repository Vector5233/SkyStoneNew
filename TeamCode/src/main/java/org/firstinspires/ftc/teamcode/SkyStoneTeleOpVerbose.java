package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp(name="SkyStoneTeleOpVerbose", group="TeamCode")

public class SkyStoneTeleOpVerbose extends OpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft, rightRoller, leftRoller, rightLift, leftLift;
    Servo deliveryGrabber, deliveryRotation, leftFoundation, rightFoundation, blockSweeper, capServo, cameraServo;
    CRServo deliveryExtender;
    ModernRoboticsI2cGyro gyro;
    Encoder myLeft, myRight, myCenter;
    EncoderArray encoderArray;
    final double r1 = 5.3655;
    final double r2 = 5.850125;
    final double r3 = 3.2215;

    //change to .8 once finished with testing
    final double ROLLERPOWER = .8;

    final double TRIGGERTHRESHOLD = 0.5;

    double driveSpeed = 1;

    double rotationPos = 0;

    final String rotationOut = "rotationOut";
    final String rotationIn = "rotationIn";
    final String rotationHalf = "rotationHalf";
    final String rotationMovingHalf = "rotationMovingHalf";
    final String rotationMovingIn = "rotationMovingIm";
    final String rotationMovingOut = "rotationMovingOut";

    final String extenderIn = "extenderIn";
    final String extenderMovingOut = "extenderMovingOut";
    final String extenderMovingIn = "extenderMovingIn";
    final String extenderOut = "extenderOut";

    final String grabberOpen = "grabberOpen";
    final String grabberClose = "grabberClose";
    final String grabberClosing = "grabberClosing";
    final String grabberOpening = "grabberOpening";

    final String liftUp = "liftUp";
    final String liftDown = "liftDown";

    // motor speeds

    final double SLOWSPEED = .3;
    final double FASTSPEED = 1;

    //ifUnpressed should be changed to if_pressed in the rest of the code

    boolean if_pressedRT = false;
    boolean if_pressedLT = false;
    boolean if_pressedDpadDown = false;
    boolean if_pressedDpadHrz = false;
    boolean if_pressedDpadUp = false;
    boolean if_pressedGp1X = false;
    boolean if_pressedGp1Y = false;
    boolean if_pressedGp2X = false;
    boolean if_pressedGp1B = false;

    double[] radii = new double[3];

    String GrabberState = null;
    String RotationState = null;
    String ExtenderState = null;

    ElapsedTime rotationTime = new ElapsedTime();
    final int ROTATIONTIMEOUT = 500;
    ElapsedTime extenderTime = new ElapsedTime();
    final int EXTENDERTIMEOUT = 1875;
    ElapsedTime goTime = new ElapsedTime();
    boolean goFlag = true;
    String s, commandList;

    public void init() {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        rightRoller = hardwareMap.dcMotor.get("rightRoller");
        leftRoller = hardwareMap.dcMotor.get("leftRoller");

        rightLift = hardwareMap.dcMotor.get("rightLift");
        leftLift = hardwareMap.dcMotor.get("leftLift");



        deliveryGrabber = hardwareMap.servo.get("deliveryGrabber");
        deliveryRotation = hardwareMap.servo.get("deliveryRotation");

        leftFoundation = hardwareMap.servo.get("leftFoundation");
        rightFoundation = hardwareMap.servo.get("rightFoundation");

        blockSweeper = hardwareMap.servo.get("blockSweeper");

        capServo = hardwareMap.servo.get("capServo");

//        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        cameraServo = hardwareMap.servo.get("cameraServo");

        deliveryExtender = hardwareMap.crservo.get("deliveryExtender");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        rightRoller.setDirection(DcMotor.Direction.FORWARD);
        leftRoller.setDirection(DcMotor.Direction.REVERSE);

        rightLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.REVERSE);

        deliveryExtender.setDirection(CRServo.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GrabberState = grabberClose;
        RotationState = rotationIn;
        ExtenderState = extenderIn;

        deliveryRotation.setPosition(0);
        RotationState = rotationMovingIn;

        blockSweeper.setPosition(0.95);




        leftFoundation.setPosition(0.1);
        rightFoundation.setPosition(0.8);

        deliveryGrabber.setPosition(0);

        capServo.setPosition(0.725);
        cameraServo.setPosition(0);

        myLeft = new Encoder(frontLeft);
        myRight = new Encoder(frontRight);
        myCenter = new Encoder(backRight);

        encoderArray = new EncoderArray(myLeft, myRight, myCenter, r1, r2, r3);

        /*gyro.calibrate();

        while (gyro.isCalibrating()) {
            telemetry.addLine("gyro calibrating");
            telemetry.update();
        }
        telemetry.addLine("gyro calibrated");
        telemetry.update();*/
    }

    public void loop() {
        if (goFlag) {
            goFlag = false;
            goTime.reset();
        }
        setDriveMotors();
        setRollerMotors();
        setLiftMotors();
        setDeliveryMotors();
        setFoundationGrabber();
        setBlockSweeper();
        setCapServo();
        resetEncoder();
        setCameraServo();
        //calibrateEncoderArray();
        encoderArray.updateAll();
        telemetry.addData("gamepad1", encodeJoystick(gamepad1));
        telemetry.addData("gamepad2", encodeJoystick(gamepad2));
        s = encodeJoystick(gamepad1);
        Log.i("JOYSTICK 1", s);
        commandList = commandList.concat(s).concat("\n");
        s = encodeJoystick(gamepad2);
        Log.i("JOYSTICK 2", s);
        commandList = commandList.concat(s).concat("\n");
    }

    public void stop() {
        try {
            SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault());
            String currentDateandTime = sdf.format(new Date());
            Context context = hardwareMap.appContext;
            File path = context.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS);
            File outputFile = new File(path, currentDateandTime);
            OutputStream os = new FileOutputStream(outputFile);
            OutputStreamWriter osw = new OutputStreamWriter(os);
            osw.write(commandList);
            osw.close();
        } catch (IOException e) {
            ;
        }
        finally {
            ;
        }

    }

    private void resetEncoder() {
        if (gamepad1.y) {
            if (!if_pressedGp1Y) {
                encoderArray.resetAll();
                //gyro.calibrate();
                telemetry.addLine("Encoders reset");
                if_pressedGp1Y = true;
            }
        } else {
            if_pressedGp1Y = false;
        }
    }

    private String encodeJoystick(Gamepad g) {
        String s = "";
        s=s.concat(String.format("%.2f ",g.left_stick_x));
        s=s.concat(String.format("%.2f ",g.left_stick_y));
        s=s.concat(String.format("%.2f ",g.right_stick_x));
        s=s.concat(String.format("%.2f ",g.right_stick_y));
        s=s.concat(String.format("%.2f ",g.left_trigger));
        s=s.concat(String.format("%.2f ",g.right_trigger));
        if (g.a) {
            s=s.concat("1 ");
        } else {
            s=s.concat("0 ");
        }
        if (g.b) {
            s=s.concat("1 ");
        } else {
            s=s.concat("0 ");
        }
        if (g.x) {
            s=s.concat("1 ");
        } else {
            s=s.concat("0 ");
        }
        if (g.y) {
            s=s.concat("1 ");
        } else {
            s=s.concat("0 ");
        }
        if (g.left_bumper) {
            s=s.concat("1 ");
        } else {
            s=s.concat("0 ");
        }
        if (g.right_bumper) {
            s=s.concat("1 ");
        } else {
            s=s.concat("0 ");
        }
        if (g.left_stick_button) {
            s=s.concat("1 ");
        } else {
            s=s.concat("0 ");
        }
        if (g.right_stick_button) {
            s=s.concat("1 ");
        } else {
            s=s.concat("0 ");
        }
        if (g.dpad_up) {
            s=s.concat("1 ");
        } else {
            s=s.concat("0 ");
        }
        if (g.dpad_right) {
            s=s.concat("1 ");
        } else {
            s=s.concat("0 ");
        }
        if (g.dpad_down) {
            s=s.concat("1 ");
        } else {
            s=s.concat("0 ");
        }
        if (g.dpad_left) {
            s=s.concat("1 ");
        } else {
            s=s.concat("0 ");
        }

    return s;
    }


    /*private void calibrateEncoderArray() {
        if (gamepad1.x) {
            if (!if_pressedGp1X) {
                encoderArray.calibrate((double)gyro.getIntegratedZValue());
                radii = encoderArray.calibrate((double)gyro.getIntegratedZValue());
                telemetry.addLine("Encoders calibrated");

                if_pressedGp1X = true;
            }
        } else {
            if_pressedGp1X = false;
        }
    }*/

    private void setDriveMotors() {

        if (gamepad1.dpad_down) {
            if (!if_pressedDpadDown) {
                driveSpeed = SLOWSPEED;
                if_pressedDpadDown = true;
            }
        } else {
            if_pressedDpadDown = false;
        }

        if (gamepad1.dpad_up) {
            if (!if_pressedDpadUp) {
                if (leftFoundation.getPosition() == .5) {

                    driveSpeed = SLOWSPEED;
                    if_pressedDpadUp = true;
                } else {
                    driveSpeed = FASTSPEED;
                    if_pressedDpadUp = true;

                }
            }
        } else {
            if_pressedDpadUp = false;
        }

        frontRight.setPower((-gamepad1.right_stick_x - gamepad1.right_stick_y - gamepad1.left_stick_x) * driveSpeed);
        frontLeft.setPower((gamepad1.right_stick_x - gamepad1.right_stick_y + gamepad1.left_stick_x) * driveSpeed);
        backRight.setPower((gamepad1.right_stick_x - gamepad1.right_stick_y - gamepad1.left_stick_x) * driveSpeed);
        backLeft.setPower((-gamepad1.right_stick_x - gamepad1.right_stick_y + gamepad1.left_stick_x) * driveSpeed);
        encoderArray.readEncoderValue();
    }

    private void setRollerMotors() {
        if (gamepad1.right_bumper) {
            rightRoller.setPower(ROLLERPOWER);
            leftRoller.setPower(ROLLERPOWER);
        } else if (gamepad1.left_bumper) {
            rightRoller.setPower(-ROLLERPOWER);
            leftRoller.setPower(-ROLLERPOWER);
        } else {
            rightRoller.setPower(0);
            leftRoller.setPower(0);
        }
    }

    private void setLiftMotors() {
        final double UPPOWER = 0.8;
        final double DOWNPOWER = 0.3;
        final double THRESHOLD = 0.5;
        // TODO consider carefully what actions could harm the lift and how to avoid doing those things
        //if(minLift <= leftLift.getCurrentPosition() &&leftLift.getCurrentPosition() <= maxLift) {
        if (gamepad2.left_stick_y > THRESHOLD) {
            // lifting down

            rightLift.setPower(gamepad2.left_stick_y * DOWNPOWER);
            leftLift.setPower(gamepad2.left_stick_y * DOWNPOWER);

        } else if (gamepad2.left_stick_y > THRESHOLD) {
            rightLift.setPower(gamepad2.left_stick_y * DOWNPOWER);
            leftLift.setPower(gamepad2.left_stick_y * DOWNPOWER);
        } else if (gamepad2.left_stick_y < -THRESHOLD) {
            // lifting Up
            rightLift.setPower(gamepad2.left_stick_y * UPPOWER);
            leftLift.setPower(gamepad2.left_stick_y * UPPOWER);
        } else {
            rightLift.setPower(0);
            leftLift.setPower(0);
        }
    }

    private void setDeliveryMotors() {
        final double ROTATIONHALF = 0.5;
        final double ROTATIONOUT = 1;


        deliveryExtender.setPower(gamepad2.right_stick_y);


        if (gamepad2.right_bumper == true) {
            deliveryGrabber.setPosition(0);
        }

        if (gamepad2.left_bumper == true) {
            deliveryGrabber.setPosition(.5);
        }


        if ((gamepad2.right_trigger >= .5) && !if_pressedRT) {
            if (rotationPos < 1) {
                telemetry.addLine("rotation move out");
                rotationPos += ROTATIONHALF;
                deliveryRotation.setPosition(rotationPos);
                if_pressedRT = true;
                telemetry.update();
            }
            if_pressedRT = true;
        } else {
            if_pressedRT = false;
        }

        if ((gamepad2.left_trigger
                >= .5) && !if_pressedLT) {
            if (rotationPos > 0) {
                telemetry.addLine("rotation move in");
                rotationPos -= ROTATIONHALF;
                deliveryRotation.setPosition(rotationPos);
                if_pressedLT = true;
                telemetry.update();
            }
            if_pressedLT = true;
        } else {
            if_pressedLT = false;
        }


    }

    /*private void setHook() {
        if (!if_pressedGp1X && gamepad1.x) {
            if (hookHrz.getPosition() >= 0.5 && hookHrz.getPosition() <= 1) {
                hookHrz.setPosition(0);
                if_pressedGp1X = true;
            } else if (hookHrz.getPosition() >= 0 && hookHrz.getPosition() < 0.5) {
                hookHrz.setPosition(1);
                if_pressedGp1X = true;
            }
        } else {
            if (!gamepad1.x) {
                if_pressedGp1X = false;
            }
        }

        if (!if_pressedGp1Y) {
            if (gamepad1.y && (hookVrt.getPosition() >= 0.75 && hookVrt.getPosition() <= 1)) {
                hookVrt.setPosition(0.4);
                if_pressedGp1Y = true;
            } else if (gamepad1.y && (hookVrt.getPosition() >= 0 && hookVrt.getPosition() <= 0.75)) {
                hookVrt.setPosition(0.9);
                if_pressedGp1Y = true;
            }
        } else {
            if (!gamepad1.y) {
                if_pressedGp1Y = false;
            }
        }
    }*/

    private void setFoundationGrabber() {
        if (gamepad1.b && !if_pressedGp1B) {
            if (leftFoundation.getPosition() != .5 /*&& rightFoundation.getPosition() == 0*/) {
                leftFoundation.setPosition(.5);
                rightFoundation.setPosition(.25);

                driveSpeed = SLOWSPEED;

                if_pressedGp1B = true;
            } else /*if (leftFoundation.getPosition() != 5 && rightFoundation.getPosition() == 0.5)*/ {
                leftFoundation.setPosition(0);
                rightFoundation.setPosition(.8);
                if_pressedGp1B = true;

                driveSpeed = FASTSPEED;

            }

        } else {
            if (!gamepad1.b) {
                if_pressedGp1B = false;
            }
        }
    }

    private void setBlockSweeper() {
        if (gamepad1.right_trigger >= TRIGGERTHRESHOLD) {
            blockSweeper.setPosition(0.725);
        } else {
            blockSweeper.setPosition(1);
        }
    }

    private void setCapServo() {
        // better code
        if (gamepad2.x && !if_pressedGp2X) {
            if (capServo.getPosition() <= 0.1) {
                capServo.setPosition(0.725);
                if_pressedGp2X = true;
            } else if (capServo.getPosition() <= 0.85 && capServo.getPosition() >= 0.65) {
                capServo.setPosition(0);
                if_pressedGp2X = true;
            }
        } else {
            if (!gamepad2.x) {
                if_pressedGp2X = false;
            }
        }
    }

    /* private void setCapServo() {
        if (gamepad2.a) {
            capServo.setPosition(0);
        } else {
            capServo.setPosition(.8);
        }
    }

    */

    /*private void setDeliveryGrabber() {
        telemetry.addData("grabber state", GrabberState);
        telemetry.update();

        switch (GrabberState) {
            case grabberOpen:
                //deliveryGrabber.setPosition(0.35);
                grabberTime.reset();
                if (gamepad2.right_bumper) {
                    if (!if_pressedRB) {
                        GrabberState = grabberClosing;
                        if_pressedRB = true;
                    }
                }
                else {
                    if_pressedRB = false;
                }
                break;

            case grabberClose:
                //deliveryGrabber.setPosition(0.435);
                grabberTime.reset();
                if (gamepad2.left_bumper) {
                    if(!if_pressedLB) {
                        GrabberState = grabberOpening;
                        if_pressedLB = true;
                    }

                }
                else {
                    if_pressedLB = false;
                }
                break;

            case grabberOpening:
                deliveryGrabber.setPosition(0);
                if (grabberTime.milliseconds() >= GRABBERTIMEOUT) {
                    GrabberState = grabberOpen;
                }
                break;
            case grabberClosing:
            //default:
                deliveryGrabber.setPosition(0.6);
                if (grabberTime.milliseconds() >= GRABBERTIMEOUT) {
                    GrabberState = grabberClose;
                }
                break;

        }

    }*/

    private void setCameraServo() {
        return;
    }

    /*
        if (gamepad2.y){
            cameraServo.setPosition(0);
        } else{
            cameraServo.setPosition(1);
        }
    }*/

    private void setDeliveryExtender() {
        switch (ExtenderState) {
            case extenderOut:
                // There is no way to code the position of continuous servo...
                extenderTime.reset();
                if (gamepad2.right_stick_y <= -0.5 && extenderTime.milliseconds() >= EXTENDERTIMEOUT) {
                    if (RotationState == rotationIn) {
                        ExtenderState = extenderMovingIn;
                    } else {
                        RotationState = rotationMovingIn;
                    }

                }

                break;
            case extenderIn:
                extenderTime.reset();
                if (gamepad2.right_stick_y >= 0.5 && extenderTime.milliseconds() >= EXTENDERTIMEOUT) {
                    if (RotationState == rotationIn) {
                        ExtenderState = extenderMovingOut;
                    } else {
                        RotationState = rotationMovingIn;
                    }

                }

                break;

            case extenderMovingIn:
                deliveryExtender.setPower(1);
                if (extenderTime.milliseconds() >= EXTENDERTIMEOUT) {
                    ExtenderState = extenderIn;
                }
                break;

            case extenderMovingOut:
                deliveryExtender.setPower(-1);
                if (extenderTime.milliseconds() >= EXTENDERTIMEOUT) {
                    ExtenderState = extenderOut;
                }
                break;

        }
    }

    private void setDeliveryRotation() {
        switch (RotationState) {
            case rotationIn:
                if (ExtenderState == extenderOut) {
                    if (gamepad2.dpad_down) {
                        if (!if_pressedDpadDown) {
                            RotationState = rotationMovingIn;
                            if_pressedDpadDown = true;
                        }
                    } else {
                        if_pressedDpadDown = false;
                    }
                    if (gamepad2.dpad_left || gamepad2.dpad_right) {
                        if (!if_pressedDpadHrz) {
                            RotationState = rotationMovingHalf;
                            if_pressedDpadHrz = true;
                        }
                    } else {
                        if_pressedDpadHrz = false;
                    }
                }
                break;
            case rotationOut:
                if (ExtenderState == extenderOut) {
                    if (gamepad2.dpad_up) {
                        if (!if_pressedDpadUp) {
                            RotationState = rotationMovingOut;
                            if_pressedDpadUp = true;
                        }
                    } else {
                        if_pressedDpadUp = false;
                    }
                    if (gamepad2.dpad_left || gamepad2.dpad_right) {
                        if (!if_pressedDpadHrz) {
                            RotationState = rotationMovingHalf;
                            if_pressedDpadHrz = true;
                        }
                    } else {
                        if_pressedDpadHrz = false;
                    }
                }
                break;
            case rotationHalf:
                if (ExtenderState == extenderOut) { // not sure about this commented by Wyatt
                    if (gamepad2.dpad_down) {
                        if (!if_pressedDpadDown) {
                            RotationState = rotationMovingIn;
                            if_pressedDpadDown = true;
                        }
                    } else {
                        if_pressedDpadDown = false;
                    }
                    if (gamepad2.dpad_up) {
                        if (!if_pressedDpadUp) {
                            RotationState = rotationMovingOut;
                            if_pressedDpadUp = true;
                        }
                    } else {
                        if_pressedDpadUp = false;
                    }
                }
                break;
            case rotationMovingIn:
                deliveryRotation.setPosition(0);
                if (rotationTime.milliseconds() >= ROTATIONTIMEOUT) {
                    RotationState = rotationIn;
                }

                break;
            case rotationMovingOut:
                deliveryRotation.setPosition(1);
                if (rotationTime.milliseconds() >= ROTATIONTIMEOUT) {
                    RotationState = rotationOut;
                }
                break;
            case rotationMovingHalf:
                deliveryRotation.setPosition(0.5);
                if (rotationTime.milliseconds() >= ROTATIONTIMEOUT) {
                    RotationState = rotationHalf;
                }
                break;
        }
    }

    public void telemetryDcMotor() {
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("BR", backRight.getCurrentPosition());
        telemetry.addData("BL", backLeft.getCurrentPosition());

        telemetry.update();
    }
}