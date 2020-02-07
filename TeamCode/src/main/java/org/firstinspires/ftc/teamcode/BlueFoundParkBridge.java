package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueFoundParkBridge", group="Blue")

public class BlueFoundParkBridge extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);
        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.driveDistance(.5, -29);
        idle();
        drive.strafeDistance(.5, 8);
        idle();
        drive.moveFoundation(drive.BLUE);
        telemetry.addLine("FoundationMoved");
        telemetry.update();
        idle();
        drive.turnToDegree(.3,90);
        idle();
        drive.park(drive.BLUE, drive.BRIDGE);
    }
}

