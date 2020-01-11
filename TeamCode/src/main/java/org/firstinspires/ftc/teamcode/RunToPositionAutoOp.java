package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="RunToPositionAutoOp", group="Blue")

public class RunToPositionAutoOp extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        telemetry.addLine("initialized");
        telemetry.update();
    }
    public void runOpMode(){
        drive.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

