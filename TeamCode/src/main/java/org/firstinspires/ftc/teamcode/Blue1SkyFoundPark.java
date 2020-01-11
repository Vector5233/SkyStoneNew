package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue1SkyFoundPark", group="Blue")

public class Blue1SkyFoundPark extends LinearOpMode {
    SSDriveObject drive;

    public void initialize(){//lll
        drive = new SSDriveObject(this);

        drive.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

//        drive.detectReady(drive.BLUE);
        drive.detectStones();
//        drive.getDisplacement();
//        drive.collectSkyStone(drive.BLUE);
//        drive.deliverSkystone(drive.BLUE);
//        drive.skystonePark(drive.BLUE);
        /*drive.moveToFoundation(drive.BLUE);
        drive.moveFoundation(drive.BLUE);
        drive.park(drive.BLUE, drive.FOUNDATION);*/
    }
}

