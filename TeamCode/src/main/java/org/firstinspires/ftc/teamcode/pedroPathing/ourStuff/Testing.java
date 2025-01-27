package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff;

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


@Autonomous(name = "Testing", group = "auto test")
public class Testing extends opmode_MAIN{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, armState, clawState;
    private String navigation;

    //Poses

    private final Pose startPose = new Pose(8,63,0);
    private final Pose endPose = new Pose(28,83,Math.toRadians(180));

    private Path straightTest;
    private Path curveTest;

    public void buildPaths() {
        straightTest = new Path(new BezierLine(new Point(startPose),new Point(endPose)));
        straightTest.setLinearHeadingInterpolation(startPose.getHeading(),endPose.getHeading());

        curveTest = new Path(new BezierCurve(new Point(endPose), new Point(38,18), new Point(startPose)));
        curveTest.setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading());
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(straightTest);
                setPathState(1);
                break;

            case 1:
                if( follower.getPose().getX() > (endPose.getX() - 1) && follower.getPose().getY() > (endPose.getY() - 1)) {
                    follower.followPath(curveTest);
                    setPathState(-1);
                    break;
            }

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