package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkyStoneDetector extends OpenCvPipeline {
    OpenCvInternalCamera phoneCam;
    public Mat workingMatrix = new Mat();
    public String position = "LEFT";
    public SkyStoneDetector() {

    }

    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = workingMatrix.submat(30, 70, 175, 205);
        Mat matCenter = workingMatrix.submat(100, 140, 175, 205);
        Mat matRight = workingMatrix.submat(170, 210, 175, 205);

        Imgproc.rectangle(workingMatrix, new Rect(175, 170, 30, 40), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(175, 100, 30, 40), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(175, 30, 30, 40), new Scalar(0,255,0));

        double leftTotal = Core.sumElems(matLeft).val[2];
        double centerTotal = Core.sumElems(matCenter).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];

        if (leftTotal > centerTotal) {
            if (leftTotal > rightTotal) {
                // left is skystone
                position = "LEFT";
            } else {
                // right is skystone
                position = "RIGHT";
            }
        } else {
            if (centerTotal > rightTotal) {
                // center is skystone
                position = "CENTER";
            } else {
                // right is skystone
                position = "RIGHT";
            }
        }

        return workingMatrix;
    }
}
