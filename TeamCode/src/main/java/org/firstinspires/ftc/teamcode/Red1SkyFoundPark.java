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

        drive.goToDetectPosition(drive.RED);
        sleep(2000);
        int skystone = drive.detectReady(drive.RED);
        telemetry.addLine(skystoneString(skystone));
        telemetry.update();
        drive.collectSkyStone(drive.RED,skystone);
        /*drive.detectReady();
        drive.detectStones();
        drive.getDisplacement();
//        drive.collectSkyStone();
        drive.moveToFoundation(drive.RED);
        drive.moveFoundation(drive.RED);
        drive.park(drive.BLUE, drive.FOUNDATION);*/
    }
    public String skystoneString(int skystone){
        switch(skystone) {
            case 0:
                return "left";
            case 1:
                return "center";
            case 2:
                return "right";
            default:
                return "oops";
        }
    }
}

