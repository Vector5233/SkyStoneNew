package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="OdometeryTest", group="TeamCode")

public class OdometeryTest extends OpMode {
    DcMotor testOdometer;

    public void init() {
        testOdometer = hardwareMap.dcMotor.get("testOdometer");
        testOdometer.setDirection(DcMotor.Direction.FORWARD);

    }

    public void loop() {
        testOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("odometer value", testOdometer.getCurrentPosition());
    }
}


