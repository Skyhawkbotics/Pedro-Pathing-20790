package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Config
@Autonomous(name = "right_auto_hangs", group = "AUTO")
// 18.5 inches away from observation zone
public class right_auto_hangs extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, outtimer;
    private int pathState, armState, outclawState, outgrabState, inclawState, ingrabState; // Different cases and states of the different parts of the robot

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Centerstage, this would be blue far side/red human player station.)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y. **/
    //Start Pose
    private Pose startPose = new Pose(18, 40, Math.toRadians(180)); //TODO

    private Pose hangPose = new Pose(36.5, 67.0, Math.toRadians(0)); // TODO

    private Pose hangPose1 = new Pose(36.0, 63.0, Math.toRadians(0)); // TODO

    private Pose pickupPose = new Pose(8, 20, Math.toRadians(180)); // TODO : THISx value

    private Pose pushPose = new Pose(25, 38, Math.toRadians(0));

    private Pose firstpoint = new Pose(36.0,40.0, Math.toRadians(0));

    private Pose secondpoint = new Pose(63.00, 40.00, Math.toRadians(0));
    private Pose secondpoint1 = new Pose(63.00, 35.00, Math.toRadians(0));


    private Pose thirdpoint = new Pose(63, 22.00, Math.toRadians(0));

    private Pose control_p2 = new Pose(51.49198520345253, 44.74475955610358, Math.toRadians(0));

    private Pose curve_curve = new Pose(75, 37, Math.toRadians(0));

    // Paths

    private PathChain hangs;


    // Motors

    private DcMotorEx up;
    private Servo servo_outtake_wrist;
    private CRServo servo_outtake;



    private TouchSensor up_zero;
    private Telemetry telemetryA;

    double intake_wrist_pos_transfer = 0;
    double outtake_wrist_pos_transfer = 0;
    int up_hanging_position = 1750; //DONE: calibrate this value, viper slide position to
    int up_hanging_position_done = 1290; //TODO: calibrate this value, position of viper slide when releasing after speciman is on the bar.
    public void buildPaths() {
        hangs = follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(startPose),
                                        new Point(10.000, 40.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                // Line 2
                                new BezierLine(
                                        new Point(10.000, 40.000, Point.CARTESIAN),
                                        new Point(36.000, 65.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                        .addPath(
                                // Line 3
                                new BezierLine(
                                        new Point(36.000, 65.000, Point.CARTESIAN),
                                        new Point(38.000, 65.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 4
                                new BezierLine(
                                        new Point(38.000, 65.000, Point.CARTESIAN),
                                        new Point(26.000, 65.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(26.000, 65.000, Point.CARTESIAN),
                                        new Point(26.000, 65.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                // Line 6
                                new BezierLine(
                                        new Point(26.000, 65.000, Point.CARTESIAN),
                                        new Point(startPose)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(hangs);

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


        // Feedback to Driver Hub
        telemetryA.addData("path state", pathState);

        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        telemetryA.addData("pathtimer elapsed time", pathTimer.getElapsedTimeSeconds());
        telemetryA.addData("Follower busy", follower.isBusy());

        telemetryA.addData("headingPID error", follower.headingError);
        telemetryA.addData("drivePID error", follower.driveError);
        telemetryA.update();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.4);


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        //setup arm variable
        up = hardwareMap.get(DcMotorEx.class, "up");
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up.setDirection(DcMotorSimple.Direction.REVERSE);


        servo_outtake = hardwareMap.get(CRServo.class,"outtake");


        servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");



        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");




        //huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

    }
    @Override
    public void start() {
        //setBackdropGoalPose();
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
