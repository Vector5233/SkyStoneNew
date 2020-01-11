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

        drive.goToDetectPosition();
        /*sleep(2000);
        int skystone = drive.detectReady();
        telemetry.addLine(skystoneString(skystone));
        telemetry.update();*/
        drive.collectSkyStone(drive.BLUE,drive.LEFT);

//

//        drive.getDisplacement();
//        drive.collectSkyStone(drive.BLUE);
//        drive.deliverSkystone(drive.BLUE);
//        drive.skystonePark(drive.BLUE);

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

