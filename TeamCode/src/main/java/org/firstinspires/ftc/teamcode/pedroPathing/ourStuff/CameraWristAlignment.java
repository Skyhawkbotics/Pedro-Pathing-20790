/*package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CameraWristAlignment extends OpenCvPipeline {
    Mat mat = new Mat();
    Rect ROI = new Rect(new Point(240, 120),new Point(300,145));//TODO: Figure out how big the ROI will be and change the dimensions of the rect depending on what you find

    Mat MAT;
    @Override
    public Mat processFrame(Mat input){
        // ---THRESHOLDING---
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowerBound = new Scalar(195.0/2,55,100);
        Scalar upperBound = new Scalar(255.0/2,100,100);
        Core.inRange(mat,lowerBound,upperBound,mat);
        // ---DIVIDE---
        MAT = mat.submat(ROI);
        // ---AVERAGE---
        double percentageInRectValue = Math.round(Core.mean(MAT).val[2] / 255);
        MAT.release();
        mat.release();
        // ---COMPARE---
        final double THRESHOLD = 10;
        if (percentageInRectValue > THRESHOLD) {
            telemetry.addData(); //Aaaaa no enum! Try reading the FTC guide and using that instead
        }
        else {
            //A
        }
        return null;
    }*/


}
