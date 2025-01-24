package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

@Config
@Autonomous(name = "left_auto_trajectory_only", group = "Auto")
public class left_auto_trajectory_only extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private String navigation;
    private int pathState;

    //start pose
    private Pose startPose = new Pose(39,82.5,Math.toRadians(0));
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
                    .setTangentHeadingInterpolation() //tanget?
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
            switch(pathState){
                case 1:
                    follower.followPath(pushAll,true);
                    setPathState(-1);
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
        telemetry.addData("Path State",pathState);
        telemetry.addData("Position",follower.getPose().toString());
        telemetry.update();
    }
}

