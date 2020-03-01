package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "OpenCVTestAutoOp")
@Disabled
public class OpenCVTestAutoOp extends LinearOpMode {
    OpenCvCamera webcam;
    private SkyStoneDetector detector = new SkyStoneDetector();
    private String position;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(detector);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        telemetry.addData("is started", isStarted());

        while (!isStarted()) {
            position = detector.position;
            telemetry.addData("position", position);
            telemetry.update();
            Log.i("OPENCV", String.format("driveDistance: \t%s\n",position));
        }

        // code while robot is running
        if (position.equals("LEFT")) {
            // left code
        }
    }
}
