package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Config
@Autonomous(name = "left_auto", group = "Auto")
public class left_auto extends OpMode {
   //Standard variables
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private String navigation;
    private int pathState, viperState, misumiState, viperWristState,misumiWristState, viperClawState, misumiClawState;

    //Actuators in use!
    private DcMotorEx up, out;
    private Servo servo_outtake_wrist;
    private CRServo servo_outtake;
    private Servo servo_intake;
    private CRServo servo_intake_wrist;

    //Sensors in use!
    private TouchSensor up_zero;

    //start pose
    private Pose startPose = new Pose(39,82.5,Math.toRadians(0));
    //Path variable!
    private PathChain pushAll;

    public void buildPaths() {

        pushAll = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(8.990, 88.000, Point.CARTESIAN),
                                new Point(38.955, 82.405, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(38.955, 82.405, Point.CARTESIAN),
                                new Point(31.297, 122.025, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(31.297, 122.025, Point.CARTESIAN),
                                new Point(11.320, 132.347, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(11.320, 132.347, Point.CARTESIAN),
                                new Point(33.461, 133.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-15))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(33.461, 133.000, Point.CARTESIAN),
                                new Point(11.320, 132.180, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(145))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(11.320, 132.180, Point.CARTESIAN),
                                new Point(33.295, 133.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(35))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(33.295, 133.000, Point.CARTESIAN),
                                new Point(11.487, 132.347, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(145))
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                follower.followPath(pushAll, true);
                setPathState(-1);
                break;
        }
    }

    public void autonomousActionUpdate() {
        //Misumi slide component states
        switch(misumiState){
            case 0:
                //going to closed position

            case 1:
                //extended length for picking up samples
        }
        switch(misumiWristState){
            case 0:
                //out of the way
                break;
            case 1:
                //sample collection position
                break;
            case 2:
                //transfer position
                break;
        }
        switch(misumiClawState){
            case 0:
                //servos are set to power 0
                break;
            case 1:
                //servos are set to power -1
                break;
            case 2:
                //servos are set to power 1
                break;
        }

        //Viper slide component states
        switch(viperState){
            case 0:
                //position of the arm is at 0 ticks
                break;
            case 1:
                //position of the viper slide is at transfer position
                break;
            case 2:
                //position of the viper slide is at sample-dropping length (max)
                break;
        }
        switch(viperWristState){
            case 0:
                //transfer position
                break;
            case 1:
                //specimen hang position
                break;
            case 2:
                //sample-dropping position
                break;
        }
        switch(viperClawState){
            case 0:
                //servos are set to power 0
                break;
            case 1:
                //servos are set to power -1
                break;
            case 2:
                //servos are set to power 1
                break;
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
        telemetry.addData("Path State",pathState);
        telemetry.addData("Position",follower.getPose().toString());
        telemetry.update();
    }
}

