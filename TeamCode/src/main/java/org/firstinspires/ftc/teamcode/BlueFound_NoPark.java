package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueFound_NoPark", group="Blue")

public class BlueFound_NoPark extends LinearOpMode {
    SSDriveObject game;
    BaseDriveObject drive;

    public void initialize(){
        game = new SSDriveObject(this, drive);

        game.initialize();
    }
    public void runOpMode(){
        initialize();
        waitForStart();

        drive.strafeDistance(.5, 8);
        idle();
        drive.driveDistance(.5, -35);
        idle();
        game.moveFoundation(game.BLUE);
        telemetry.addLine("FoundationMoved");
        telemetry.update();
        sleep(500);
    }
}

