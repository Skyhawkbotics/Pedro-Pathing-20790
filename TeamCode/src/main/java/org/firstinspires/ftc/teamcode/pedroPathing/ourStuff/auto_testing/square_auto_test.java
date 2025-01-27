package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff.auto_testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
@Config
@Autonomous(name = "square", group = "Autonomous Pathing Tuning")
// 18.5 inches away from observation zone
public class square_auto_test extends OpMode {
    private Follower follower;

    private Telemetry telemetryA;

    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(20, 20, Math.toRadians(0)); //TODO
    private PathChain forward,up,back,down;
    private final Pose forward_end = new Pose(50,20, Math.toRadians(0));

    private final Pose up_end = new Pose(50,50,Math.toRadians(0));

    private final Pose back_end = new Pose(20,50, Math.toRadians(0));

    private final Pose down_end = startPose;
    public void buildPaths() {
        forward = follower.pathBuilder()
        .addPath(
                // Line 1
                new BezierLine(
                        new Point(startPose),
                        new Point(forward_end)
                )
        )
                .setTangentHeadingInterpolation()
                .build();
        up = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(forward_end),
                                new Point(up_end)
                        )
                )
                .setConstantHeadingInterpolation(up_end.getHeading())
                .build();
        back = follower.pathBuilder()
                .addPath(

                        // Line 3
                        new BezierLine(
                                new Point(up_end),
                                new Point(back_end)
                        )
                )
                .setConstantHeadingInterpolation(back_end.getHeading())
                .build();
        down = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(back_end),
                                new Point(down_end)
                        )
                )
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(forward);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(up);
                    setPathState(2);
                }
                break;

            case 2: // Wait until the robot is near the first sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(back);
                    setPathState(3);
                }
                break;

            case 3: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(down);
                    setPathState(4);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {

        // These loop the actions and movement of the robot
        autonomousPathUpdate();
        follower.update();

        //follower.telemetryDebug(telemetryA);


        // Feedback to Driver Hub
        telemetryA.addData("path state", pathState);

        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        telemetryA.addData("path timer elapsed time", pathTimer.getElapsedTimeSeconds());
        telemetryA.addData("Follower busy", follower.isBusy());
        telemetryA.update();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

    }
    @Override
    public void start() {
        //setBackdropGoalPose();
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
