package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueFoundPark", group="Blue")

public class  BlueFoundPark extends LinearOpMode {
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
        sleep(2500);
        drive.park(drive.BLUE, drive.FOUNDATION);
        telemetry.addLine("Parked");
        telemetry.update();
        sleep(2500);
    }
}

