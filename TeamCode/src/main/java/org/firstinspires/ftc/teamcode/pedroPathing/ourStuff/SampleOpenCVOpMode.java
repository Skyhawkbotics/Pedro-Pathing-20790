package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous()
public class SampleOpenCVOpMode extends opmode_MAIN {

    private DrawRectangleSimpleProcessor drawRectangleSimpleProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        drawRectangleSimpleProcessor = new DrawRectangleSimpleProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), drawRectangleSimpleProcessor);
    }
    @Override
    public void init_loop(){

    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {

    }
}
