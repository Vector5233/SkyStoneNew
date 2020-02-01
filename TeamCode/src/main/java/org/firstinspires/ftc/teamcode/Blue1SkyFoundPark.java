package org.firstinspires.ftc.teamcode;

import android.util.Log;

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
        sleep(500);
        drive.encoderArray.readEncoderValue();
        drive.encoderArray.updateAll();
        drive.encoderArray.resetAll();
        Log.i("FINAL POSITION",drive.getFinalPosition());
        sleep(1700);
        int skystone = drive.detectStonesStatic(drive.BLUE);
        telemetry.addLine(skystoneString(skystone));
        telemetry.update();
        Log.i("STATIC DETECTION", String.format("Number of Stones Detected: %f",drive.numberOfStones));
        Log.i("STATIC DETECTION","SkyStone Pos: " + skystoneString(skystone));


//        drive.collectSkyStone(drive.BLUE,drive.LEFT);

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

