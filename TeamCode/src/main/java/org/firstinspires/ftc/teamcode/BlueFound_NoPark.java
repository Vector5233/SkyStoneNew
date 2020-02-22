package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueFound_NoPark", group="Blue")

public class BlueFound_NoPark extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.strafeDistance(.5, 8);
        idle();
        drive.driveDistance(.5, -35);
        idle();
        drive.moveFoundation(drive.BLUE);
        telemetry.addLine("FoundationMoved");
        telemetry.update();
        sleep(500);
    }
}

