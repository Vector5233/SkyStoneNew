package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

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

    //final double TOLERANCE = ??;
    //final double ROOT2 = 1.414;
    //final int CAMERA_MIDPOINT = ??;
    //final int SAMPLING_FORWARD = ?;

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
    }

    public void detectReady(){
        setHookHrz(0.5);
        setHookVrt(1);
        opmode.sleep(500);

        driveDistance(0.7,.7, 23);
    }

    public void collectSkyStone(double displacement){
        strafeDistance(1, displacement);
        setHookVrt(0);
        opmode.sleep(500);
        driveDistance(1,1, -5);
        setHookHrz(0);
        setRollerMoters(true, 1, 1000);
        //how to check if the block is collected or not (next round)
        setBlockSweeper(true);
        opmode.sleep(750);
        setBlockSweeper(false);
    }

    public void moveFoundation (boolean side) {
        driveDistance(1,1,  -24.5);
        setFoundationLeft(true);
        opmode.sleep(1000);
        driveDistance(1,.7, 29);
        setFoundationLeft(false);
        opmode.sleep(1000);
        //park(side, FOUNDATION);
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
                driveDistance(1,1, -53);
            } else {
                opmode.telemetry.addLine("Red normal");
                opmode.telemetry.update();
                driveDistance(1,1,-53);
            }
        } else {
            if (side) {
                opmode.telemetry.addLine("Blue foundation");
                opmode.telemetry.update();

                strafeDistanceNoAccel(1,-53);
            } else {
                opmode.telemetry.addLine("Red foundation");
                opmode.telemetry.update();

                strafeDistanceNoAccel(1,53);
            }


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

    public void driveDistance(double powerLeft, double powerRight, double distance) {
        final double PERCENT = .1;
        double powerMin = 0.22;
        int ticks = (int) (distance * TICKS_PER_INCH_STRAIGHT);


        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.telemetry.addLine("Encoders reset");
        opmode.telemetry.update();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        if (powerLeft == powerRight) {
            if (ticks > 0) {
                while ((frontLeft.getCurrentPosition() <= ticks) && opmode.opModeIsActive()) {
                    if (frontLeft.getCurrentPosition() < PERCENT * ticks) {
                        setDrivePowerAll(Math.max((1 / PERCENT) * powerLeft * frontLeft.getCurrentPosition() / ticks, powerMin));
                        opmode.telemetry.addLine("accelerating");
                        telemetryDcMotor();
                    } else if (frontLeft.getCurrentPosition() < (1 - PERCENT) * ticks) {
                        setDrivePowerAll(powerLeft);
                        opmode.telemetry.addLine("cruising");
                        telemetryDcMotor();
                    } else {
                        setDrivePowerAll(Math.max(-(1 / PERCENT) * powerLeft * (frontLeft.getCurrentPosition() - ticks) / ticks, powerMin));
                        opmode.telemetry.addLine("decelerating");
                        telemetryDcMotor();
                    }
                    opmode.telemetry.update();
                }
            } else if (ticks < 0) {
                powerLeft = -powerLeft;
                powerMin = -powerMin;
                while ((frontLeft.getCurrentPosition() >= ticks) && opmode.opModeIsActive()) {
                    if (frontLeft.getCurrentPosition() > PERCENT * ticks) {
                        setDrivePowerAll(Math.max((1 / PERCENT) * powerLeft * frontLeft.getCurrentPosition() / ticks, powerMin));
                        opmode.telemetry.addLine("accelerating");
                        telemetryDcMotor();
                    } else if (frontLeft.getCurrentPosition() > (1 - PERCENT) * ticks) {
                        setDrivePowerAll(powerLeft);
                        opmode.telemetry.addLine("cruising");
                        telemetryDcMotor();
                    } else {
                        setDrivePowerAll(Math.max(-(1 / PERCENT) * powerLeft * (frontLeft.getCurrentPosition() - ticks) / ticks, powerMin));
                        opmode.telemetry.addLine("decelerating");
                        telemetryDcMotor();
                    }
                    opmode.telemetry.update();
                }
            }
        } else {
            if (ticks > 0) {
                while ((frontLeft.getCurrentPosition() <= ticks) && opmode.opModeIsActive()) {
                    if (frontLeft.getCurrentPosition() < PERCENT * ticks) {
                        setDrivePowerLeft(Math.min((1 / PERCENT) * powerLeft * frontLeft.getCurrentPosition() / ticks, powerMin));
                        setDrivePowerRight(Math.min((1 / PERCENT) * powerRight * frontLeft.getCurrentPosition() / ticks, powerMin));
                        opmode.telemetry.addLine("accelerating");
                        telemetryDcMotor();
                    } else if (frontLeft.getCurrentPosition() < (1 - PERCENT) * ticks) {
                        setDrivePowerLeft(powerLeft);
                        setDrivePowerRight(powerRight);
                        opmode.telemetry.addLine("cruising");
                        telemetryDcMotor();
                    } else {
                        setDrivePowerLeft(Math.min(-(1 / PERCENT) * powerLeft * (frontLeft.getCurrentPosition() - ticks) / ticks, powerMin));
                        setDrivePowerRight(Math.min(-(1 / PERCENT) * powerRight * (frontLeft.getCurrentPosition() - ticks) / ticks, powerMin));
                        opmode.telemetry.addLine("decelerating");
                        telemetryDcMotor();
                    }
                    opmode.telemetry.update();
                }
            } else if (ticks < 0) {
                powerLeft = -powerLeft;
                powerRight = -powerRight;
                powerMin = -powerMin;
                while ((frontLeft.getCurrentPosition() >= ticks) && opmode.opModeIsActive()) {
                    if (frontLeft.getCurrentPosition() > PERCENT * ticks) {
                        setDrivePowerLeft(Math.min((1 / PERCENT) * powerLeft * frontLeft.getCurrentPosition() / ticks, powerMin));
                        setDrivePowerRight(Math.min((1 / PERCENT) * powerRight * frontLeft.getCurrentPosition() / ticks, powerMin));
                        opmode.telemetry.addLine("accelerating");
                        telemetryDcMotor();
                    } else if (frontLeft.getCurrentPosition() > (1 - PERCENT) * ticks) {
                        setDrivePowerLeft(powerLeft);
                        setDrivePowerRight(powerRight);
                        opmode.telemetry.addLine("cruising");
                        telemetryDcMotor();
                    } else {
                        setDrivePowerLeft(Math.min(-(1 / PERCENT) * powerLeft * (frontLeft.getCurrentPosition() - ticks) / ticks, powerMin));
                        setDrivePowerRight(Math.min(-(1 / PERCENT) * powerRight * (frontLeft.getCurrentPosition() - ticks) / ticks, powerMin));
                        opmode.telemetry.addLine("decelerating");
                        telemetryDcMotor();
                    }
                    opmode.telemetry.update();
                }
            }
        }
        stopDriving();
    }

    public void strafeDistanceNoAccel(double powerLimit, double distance) {
        final double PERCENT = .25;
        final double STRAFECORRECTION = 30.0/37.0;
        double powerMin = 0.3;
        int ticks = (int) (distance * TICKS_PER_INCH_STRAFE * STRAFECORRECTION);
        opmode.telemetry.addData("ticks", ticks);
        opmode.telemetry.update();

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.telemetry.addLine("Encoders reset");
        opmode.telemetry.update();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

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

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.telemetry.addLine("Encoders reset");
        opmode.telemetry.update();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

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
                    telemetryDcMotor();
                } else if (frontLeft.getCurrentPosition() > (1 - PERCENT) * ticks) {
                    setStrafePowerAll(powerLimit);
                    opmode.telemetry.addLine("cruising");
                    telemetryDcMotor();
                } else {
                    setStrafePowerAll(Math.min(-(1 / PERCENT) * powerLimit * (frontLeft.getCurrentPosition() - ticks) / ticks, powerMin));
                    opmode.telemetry.addLine("decelerating");
                    telemetryDcMotor();
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

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.telemetry.addLine("Encoders reset");
        opmode.telemetry.update();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

        getAngleTelemetry("TURN START");

        opmode.sleep(1500);
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

    public void setRollerMoters (boolean direction, double power, int time) {
        //direction true = forward
        //direction false = backward
        rollerTimeout = new ElapsedTime();
        final int ROLLER_TIMEOUT = time;

        leftRoller.setPower(power);
        rightRoller.setPower(power);

        if(direction){
            leftRoller.setDirection(DcMotorSimple.Direction.FORWARD);
            rightRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if(!direction){
            leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        while ((rightRoller.isBusy() || leftRoller.isBusy()) && opmode.opModeIsActive()){
            if (rollerTimeout.milliseconds() > ROLLER_TIMEOUT)
                break;
        }
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

    public void setFoundationLeft (boolean launch) {
        //launch true = grabber down
        //launch false = grabber up
        if (!launch) {
            leftFoundation.setPosition(0);
            //rightFoundation.setPosition(0);
        }
        else {
            leftFoundation.setPosition(0.7);
            //rightFoundation.setPosition(0.7);
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
        opmode.telemetry.addData("FB", frontLeft.getCurrentPosition());
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
}
