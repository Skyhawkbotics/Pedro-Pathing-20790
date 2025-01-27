package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff.auto_testing;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;


@Config
@Autonomous(name = "left_auto_trajectory_only", group = "auto test")
public class left_auto_trajectory_only extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private String navigation;
    private int pathState;

    //start pose
    private Pose startPose = new Pose(9,88,Math.toRadians(0));
    //Setting up variables for poses used throughout the pathchain
    private Pose hangPose = new Pose(39,82.5,Math.toRadians(0)); //TODO: Change to Eric's values because they are tuned!
    private Pose pivot1 = new Pose(31.3,122,Math.toRadians(0));
    private Pose basket = new Pose(11.3,132.3,Math.toRadians(153));
    //setting up the pathChain
    private PathChain specimen_hang, pivot1_1, basket_1, pivot2_1, basket_2, pivot2_2, basket_3;

    public void buildPaths() {

        specimen_hang = follower.pathBuilder()
                .addPath(
                        //Hanging specimen path
                        new BezierLine(
                                new Point(startPose),
                                new Point(hangPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pivot1_1 = follower.pathBuilder()
                .addPath(
                        //Driving to pivot point 1
                        new BezierLine(
                                new Point(hangPose),
                                new Point(pivot1)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        basket_1 = follower.pathBuilder()
                .addPath(
                        //To basket
                        new BezierLine(
                                new Point(pivot1),
                                new Point(basket)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        pivot2_1 = follower.pathBuilder()
                .addPath(
                        //To pivot point 2
                        new BezierLine(
                                new Point(11.320, 132.347, Point.CARTESIAN),
                                new Point(33.461, 133.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-15))
                .build();

        basket_2 = follower.pathBuilder()
                .addPath(
                        //To basket
                        new BezierLine(
                                new Point(33.461, 133.000, Point.CARTESIAN),
                                new Point(11.320, 132.180, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(145))
                .build();

        pivot2_2 = follower.pathBuilder()
                .addPath(
                        //To pivot point 2
                        new BezierLine(
                                new Point(11.320, 132.180, Point.CARTESIAN),
                                new Point(33.295, 133.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(35))
                .build();

        basket_3 = follower.pathBuilder()
                .addPath(
                        //To basket
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
                case 0://hanging specimen
                    follower.followPath(specimen_hang);
                    if (follower.getPose().getX() > (hangPose.getX()) - 1) {
                        setPathState(1);
                    }
                    break;
                case 1:
                    follower.followPath(pivot1_1);
                    if (follower.getPose().getX() > (pivot1.getX()) - 1 && follower.getPose().getY() > (pivot1.getY()) - 1) {//TODO: I think the signs might be wrong. Figure out their coordinate system
                        setPathState(2);
                    }
                    break;
                case 2:
                    follower.followPath(basket_1);
                    if (follower.getPose().getX() > (basket.getX()) && follower.getPose().getY() > (basket.getY())) {
                        setPathState(-1); //TODO: For now until the rest of the path is added
                    }
/*                case 3:
                    follower.followPath(pivot2_1);
                    if (follower.getPose().getX() > (.getX())) //TODO: oops need to add variable :'( */
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

