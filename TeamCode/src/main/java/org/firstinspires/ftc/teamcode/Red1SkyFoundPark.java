package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red1SkyFoundPark", group="Blue")

public class Red1SkyFoundPark extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.detectReady(drive.RED);
        drive.detectStones();
        drive.getDisplacement();
        drive.collectSkyStone(drive.RED);
        drive.moveToFoundation(drive.RED);
        drive.moveFoundation(drive.RED);
        drive.park(drive.BLUE, drive.FOUNDATION);
    }
}

