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
    private Servo servo_intake_wrist;
    private CRServo servo_intake;

    //Sensors in use!
    private TouchSensor up_zero;
    private TouchSensor out_zero;

    //Position variables for robot!
    int up_hanging_position = 1750; //TODO: Check pushes in tuning test to see if value has been changed
    int up_hanging_position_done = 1400; //TODO: Check pushes in tuning test to see if value has been changed
    int up_basket_position = 3650;
    double viper_wrist_position_basket = 1;
    double viper_wrist_position_transfer = 0;
    double misumi_wrist_position_transfer = 0;
    double misumi_wrist_position_grab = 1;
    int viper_position_transfer = 150;
    double viper_wrist_position_hang = 0.5;
    int misumi_position_grab = 100; //TODO: Calibrate values!!!

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
                if (!out_zero.isPressed()) {
                    out.setPower(-0.5);
                } else if (out_zero.isPressed()) {
                    out.setPower(0);
                    out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                break;
            case 1:
                //extended length for picking up samples
                out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (out.getCurrentPosition() < misumi_position_grab) {
                    out.setPower(0.5);
                }
                else if (out.getCurrentPosition() > misumi_position_grab) {
                    out.setPower(-0.5);
                }
                else if (out.getCurrentPosition() == misumi_position_grab) {
                    out.setPower(0);
                }
                break;
        }
        switch(misumiWristState){
            case 0:
                //out of the way
                //????
                //Might just not do this one for now
                break;
            case 1:
                //sample collection position
                servo_intake_wrist.setPosition(misumi_wrist_position_grab);
                break;
            case 2:
                //transfer position
                servo_intake_wrist.setPosition(misumi_wrist_position_transfer);
                break;
        }
        switch(misumiClawState){
            case 0:
                //servos are set to power 0
                servo_intake.setPower(0);
                break;
            case 1:
                //servos are set to power -1
                servo_intake.setPower(-1);
                break;
            case 2:
                //servos are set to power 1
                servo_intake.setPower(1);
                break;
        }

        //Viper slide component states
        switch(viperState){
            case 0:
                //position of the arm is at 0 ticks
                telemetry.addData("Lowered position", true);
                up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (!up_zero.isPressed()) {
                    up.setPower(-1);
                } else if (up_zero.isPressed()) {
                    up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                break;
            case 1:
                //position of the viper slide is at transfer position
                up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (up.getCurrentPosition() < viper_position_transfer) {
                    up.setPower(0.5);
                }
                else if (up.getCurrentPosition() > viper_position_transfer) {
                    up.setPower(-1);
                }
                else {
                    up.setPower(0.01);
                }
                break;
            case 2:
                //position of the viper slide is at sample-dropping length (max)
                up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (up.getCurrentPosition() < up_basket_position){
                    up.setPower(1);
                }
                else {
                    up.setPower(0.01);
                }
                break;
            case 3:
                //specimen hanging height
                up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (up.getCurrentPosition() < up_hanging_position){
                    up.setPower(0.5);
                }
                else if (up.getCurrentPosition() > up_hanging_position){
                    up.setPower(0.01);
                }
        }
        switch(viperWristState){
            case 0:
                //transfer position
                servo_outtake_wrist.setPosition(viper_wrist_position_transfer);
                break;
            case 1:
                //specimen hang position
                servo_outtake_wrist.setPosition(viper_wrist_position_hang);
                break;
            case 2:
                //sample-dropping position
                servo_outtake_wrist.setPosition(viper_wrist_position_basket);
                break;
        }
        switch(viperClawState){
            case 0:
                //servos are set to power 0
                servo_outtake.setPower(0);
                break;
            case 1:
                //servos are set to power -1
                servo_outtake.setPower(-1);
                break;
            case 2:
                //servos are set to power 1
                servo_outtake.setPower(1);
                break;
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void setViperState(int vState) {
        viperState = vState;
    }
    public void setMisumiState(int mState) {
        misumiState = mState;
    }
    public void setViperWristState(int vWState) {
        viperWristState = vWState;
    }
    public void setMisumiWristState(int mWState) {
        misumiWristState = mWState;
    }
    public void setViperClawState(int vCState) {
        viperClawState = vCState;
    }
    public void setMisumiClawState(int mCState) {
        misumiClawState = mCState;
    }



    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        //Initialize all states here!
        setViperState(0);
        setMisumiState(0);
        setMisumiWristState(2);
        setMisumiClawState(0);
        setViperWristState(0);
        setViperClawState(0);

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

