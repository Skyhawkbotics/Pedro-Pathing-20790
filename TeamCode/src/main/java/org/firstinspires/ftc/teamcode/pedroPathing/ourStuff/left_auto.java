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
    private int up_hanging_position = 1750; //TODO: Check pushes in tuning test to see if value has been changed
    private int up_hanging_position_done = 1320; //TODO: Check pushes in tuning test to see if value has been changed
    private int up_basket_position = 3650;
    private double viper_wrist_position_basket = 1;
    private double viper_wrist_position_transfer = 0;
    private double misumi_wrist_position_transfer = 0;
    private double misumi_wrist_position_grab = 1;
    private int viper_position_transfer = 150;
    private double viper_wrist_position_hang = 0.5;
    private int misumi_position_grab = 100; //TODO: Calibrate values!!!
    //Time-related variables
    private double before_transfer_time = 0;

    //start pose
    private Pose startPose = new Pose(9,88,Math.toRadians(0));//TODO: This isn't right according to path generator. Test!
    //Setting up variables for poses used throughout the pathchain
    private Pose hangPose = new Pose(39,82.5,Math.toRadians(0)); //TODO: Change to Eric's values because they are tuned!
    private Pose pivot1 = new Pose(31.3,122,Math.toRadians(0));
    private Pose basket = new Pose(11.3,132.3,Math.toRadians(153));
    //private Pose pivot2 = new Pose()

    //Path variable!
    private PathChain specimen_hang, pivot1_1, basket_1, pivot2_1, basket_2, pivot2_2, basket_3;

    public void buildPaths() {

/*        pushAll = follower.pathBuilder()
                .addPath(
                        //Hanging specimen path
                        new BezierLine(
                                new Point(8.990, 88.000, Point.CARTESIAN),
                                new Point(38.955, 82.405, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        //Driving to pivot point 1
                        new BezierLine(
                                new Point(38.955, 82.405, Point.CARTESIAN),
                                new Point(31.297, 122.025, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        //To basket
                        new BezierLine(
                                new Point(31.297, 122.025, Point.CARTESIAN),
                                new Point(11.320, 132.347, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        //To pivot point 2
                        new BezierLine(
                                new Point(11.320, 132.347, Point.CARTESIAN),
                                new Point(33.461, 133.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-15))
                .addPath(
                        //To basket
                        new BezierLine(
                                new Point(33.461, 133.000, Point.CARTESIAN),
                                new Point(11.320, 132.180, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(145))
                .addPath(
                        //To pivot point 2
                        new BezierLine(
                                new Point(11.320, 132.180, Point.CARTESIAN),
                                new Point(33.295, 133.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(35))
                .addPath(
                        //To basket
                        new BezierLine(
                                new Point(33.295, 133.000, Point.CARTESIAN),
                                new Point(11.487, 132.347, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(145))
                .build();
        */

        specimen_hang = follower.pathBuilder()
                .addPath(
                        //Hanging specimen path
                        new BezierLine(
                                new Point(8.990, 88.000, Point.CARTESIAN),
                                new Point(38.955, 82.405, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pivot1_1 = follower.pathBuilder()
                .addPath(
                        //Driving to pivot point 1
                        new BezierLine(
                                new Point(38.955, 82.405, Point.CARTESIAN),
                                new Point(31.297, 122.025, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        basket_1 = follower.pathBuilder()
                .addPath(
                        //To basket
                        new BezierLine(
                                new Point(31.297, 122.025, Point.CARTESIAN),
                                new Point(11.320, 132.347, Point.CARTESIAN)
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

        //park = follower.pathBuilder()
        //Stuff to work on later


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0://hanging specimen
                follower.followPath(specimen_hang);

                setViperState(3);
                setViperWristState(1);

                if (follower.getPose().getX() > (hangPose.getX()) - 1) {
                    setViperState(4);//down
                    setPathState(1);
                }
                break;
            case 1://releasing specimen
                if (pathTimer.getElapsedTime() > 0.75) {
                    setViperClawState(2);//spit out specimen
                    setPathState(2);
                }
                break;
            case 2:
                follower.followPath(pivot1_1);

                setMisumiState(1);
                if (follower.getPose().getX() > (pivot1.getX()) - 1 && follower.getPose().getY() > (pivot1.getY()) - 1) {//TODO: I think the signs might be wrong. Figure out their coordinate system
                    setMisumiState(1);
                }
                if (misumiState == 1) {
                    setMisumiClawState(); //TODO: Figure out if bumper or trigger is intake
                    setMisumiWristState(1);
                }
                if (pathTimer.getElapsedTime() > 5) { //TODO: Change time value because it could damage servo!!!
                    setMisumiClawState(0);
                    setMisumiWristState(0);
                    setViperState(1);
                    setViperWristState(0);
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(basket_1);
                before_transfer_time = pathTimer.getElapsedTime();

                //set states for transfer! Don't know these values yet

                if (pathTimer.getElapsedTime() - before_transfer_time >= 2) {//TODO:Test if this work and tune values
                    setMisumiClawState(0); //turning off the claws
                    setViperClawState(0); //turning off the claws
                    setViperWristState(2); //setting wrist to dropping position
                    setViperState(2); //extending viper to dropping height
                }

                if (follower.getPose().getX() > (basket.getX()) && follower.getPose().getY() > (basket.getY()) && up.getCurrentPosition() >= up_basket_position) {
                    setViperClawState(); //TODO: What is the value for spitting out?
                }
                if (pathTimer.getElapsedTime() >= 10) {
                    setPathState(-1); //TODO: For now until the rest of the path is added
                }
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
                break;
            case 4:
                //specimen hanging done height
                up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (up.getCurrentPosition() > up_hanging_position_done) {
                    up.setPower(-0.7);
                } else if (up.getCurrentPosition() <= up_hanging_position_done) {
                    up.setPower(0.01);
                }
                break;
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

        //Initializing hardware (sensors and actuators)
        //setup arm variable
        up = hardwareMap.get(DcMotorEx.class, "up");
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up.setDirection(DcMotorSimple.Direction.REVERSE);

        //example position setup
        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo_outtake = hardwareMap.get(CRServo.class,"outtake");

        servo_intake = hardwareMap.get(CRServo.class, "intake");

        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");

        servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");

        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");
        out_zero = hardwareMap.get(TouchSensor.class, "out_zero");

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
        telemetry.addData("Viper State", viperState);
        telemetry.addData("Viper Claw state", viperClawState);
        telemetry.addData("Viper Wrist State", viperWristState);
        telemetry.addData("Misumi State", misumiState);
        telemetry.addData("Misumi Claw State", misumiClawState);
        telemetry.addData("Misumi Wrist State", misumiWristState);

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("viperPOS", up.getCurrentPosition());
        telemetry.addData("misumiPOS", out.getCurrentPosition());
        telemetry.addData("pathtimer elapsed time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.update();
    }
}

