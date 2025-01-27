package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff.camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;


public class EmptyProcessor implements VisionProcessor {
    @Override
    public void init(int width, int height, CameraCalibration cameraCalibration) {

    }

    @Override
    public Object processFrame (Mat frame, long captureTimeNanos) {

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onScreenWidth, int onScreenHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext) {

    }

}
