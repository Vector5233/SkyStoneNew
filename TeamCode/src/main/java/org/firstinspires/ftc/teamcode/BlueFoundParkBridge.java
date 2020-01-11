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



        drive.moveFoundation(drive.BLUE);
        telemetry.addLine("FoundationMoved");
        telemetry.update();
        sleep(500);
        drive.park(drive.BLUE, drive.BRIDGE);
        telemetry.addLine("Parked");
        telemetry.update();
        sleep(500);
    }
}

