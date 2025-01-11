package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

    /**
     * This is an example auto that showcases movement and control of three servos autonomously.
     * It is able to detect the team element using a huskylens and then use that information to go to the correct spike mark and backdrop position.
     * There are examples of different ways to build paths.
     * A custom action system have been created that can be based on time, position, or other factors.
     *
     * @author Baron Henderson - 20077 The Indubitables
     * @version 2.0, 9/8/2024
     */

    @Config
    @Autonomous(name = "square_TEST", group = "TEST")
    public class square_auto_test extends OpMode {
        // cool
        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;
        private int pathState, armState, clawState, grabState;
        private String navigation;

        /** Create and Define Poses + Paths
         * Poses are built with three constructors: x, y, and heading (in Radians).
         * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
         * (For Centerstage, this would be blue far side/red human player station.)
         * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y. **/
        //Start Pose
        private Pose startPose = new Pose(9.8, 60, Math.toRadians(0));
        //Spike mark locations

        // Poses and Paths for Purple and Yellow
        private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;
        private Path testFirstHang;
        private PathChain square;
        // Motors
        private DcMotorEx up, out;
        private Servo servo_outtake_wrist;
        private CRServo servo_outtake;

        private TouchSensor up_zero;
        private Telemetry telemetryA;
        private int up_true_target_pos;
        int up_hanging_position = 1750; //DONE: calibrate this value, viper slide position to
        int up_hanging_position_done = 1400; //TODO: calibrate this value, position of viper slide when releasing after speciman is on the bar.

        /** Generate Spike Mark and Backdrop Paths based off of the team element location **/
    /*public void setBackdropGoalPose() {
        spikeMarkGoalPose = new Pose(LeftSpikeMark.getX(), LeftSpikeMark.getY(), Math.toRadians(270));
        initialBackdropGoalPose = new Pose(LeftBackdrop.getX(), LeftBackdrop.getY(), Math.toRadians(270));
        firstCycleBackdropGoalPose = new Pose(WhiteBackdrop.getX(), WhiteBackdrop.getY(), Math.toRadians(270));
        scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(8.5, 80.5, Point.CARTESIAN), new Point(48, 135, Point.CARTESIAN), new Point(LeftSpikeMark)));
    }*/

        /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
         * It is necessary to do this so that all the paths are built before the auto starts. **/
        public void buildPaths() {
            /** There are two major types of paths components: BezierCurves and BezierLines.
             *    * BezierCurves are curved, and require > 3 points. There are the start and end points, and the control points.
             *    - Control points manipulate the curve between the start and end points.
             *    - A good visualizer for this is [this](https://www.desmos.com/calculator/3so1zx0hcd).
             *    * BezierLines are straight, and require 2 points. There are the start and end points. **/
            square = follower.pathBuilder()
                    .addPath(
                            // Line 1
                            new BezierLine(
                                    new Point(9.757, 84.983, Point.CARTESIAN),
                                    new Point(45.340, 85.360, Point.CARTESIAN)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            // Line 2
                            new BezierLine(
                                    new Point(45.340, 85.360, Point.CARTESIAN),
                                    new Point(44.977, 58.156, Point.CARTESIAN)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            // Line 3
                            new BezierLine(
                                    new Point(44.977, 58.156, Point.CARTESIAN),
                                    new Point(8.826, 58.277, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 4
                            new BezierLine(
                                    new Point(8.826, 58.277, Point.CARTESIAN),
                                    new Point(9.552, 84.514, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }

        /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
         * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() function on line 193)
         * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. **/
        public void autonomousPathUpdate() {
            switch (pathState) {
                case 11:
                    follower.followPath(square, true); //seems to fix wierd pid issue if we run this first, this just goes forward one inch, but it doesn't actually move because of stupid pid issue
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) { //just wait for a bit, idk why it was in the example... TODO: maybe remove this later or shorten it
                        setPathState(12);
                    }
                    break;

            }
        }

        /** This switch is called continuously and runs the necessary actions, when finished, it will set the state to -1.
         * (Therefore, it will not run the action continuously) **/
        public void autonomousActionUpdate() {
            switch (armState) { //most of the code stolen from opmode_main
                case 0: //going to bottom position
                    telemetry.addData("Lowered position", true);
                    if (!up_zero.isPressed()) {
                        up.setPower(-1);
                    } else if (up_zero.isPressed()) {
                        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    break;
                case 1: //going to hanging position
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    telemetry.addData("Hang position", true);
                    if (up.getCurrentPosition() < up_hanging_position) {
                        up.setPower(0.5);
                        telemetry.addData("arm moving", true);
                    } else if (up.getCurrentPosition() >= up_hanging_position) {
                        up.setPower(0.01);
                    }
                    break;
                case 2: //going to hanging position
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    telemetry.addData("hang position 2", true);
                    if (up.getCurrentPosition() < up_hanging_position_done) {
                        up.setPower(0.5);
                        telemetry.addData("arm moving", true);
                    } else if (up.getCurrentPosition() >= up_hanging_position_done) {
                        up.setPower(-0.5);
                    }
                    break;
            }
            switch (clawState) {
                case 0:
                    servo_outtake_wrist.setPosition(0.3);
                    telemetry.addData("claw position 1 ", true);
                    break;
                case 1:
                    servo_outtake_wrist.setPosition(0.75);
                    telemetry.addData("claw position 2", true);

            }
            switch (grabState) {
                case 0:
                    servo_outtake.setPower(0);
                    break;
                case 1: //release
                    servo_outtake.setPower(1);
                    break;
                case 2: //grab
                    servo_outtake.setPower(-1);
                    break;
            }
        }



        /** These change the states of the paths and actions
         * It will also reset the timers of the individual switches **/
        public void setPathState(int pState) {
            pathState = pState;
            pathTimer.resetTimer();
        }

        public void setArmState(int aState) {
            armState = aState;
        }
        public void setGrabState(int gstate) {
            grabState = gstate;
        }

        public void setClawState(int cState) {
            clawState = cState;
        }

        /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
        @Override
        public void loop() {

            // These loop the actions and movement of the robot
            autonomousPathUpdate();
            follower.update();
            autonomousActionUpdate();

            follower.telemetryDebug(telemetryA);


            // Feedback to Driver Hub
            telemetryA.addData("path state", pathState);
            telemetryA.addData("arm state", armState);
            telemetryA.addData("claw state", clawState);
            telemetryA.addData("x", follower.getPose().getX());
            telemetryA.addData("y", follower.getPose().getY());
            telemetryA.addData("heading", follower.getPose().getHeading());
            telemetryA.addData("armPOS", up.getCurrentPosition());
            telemetryA.addData("pathtimer elapsed time", pathTimer.getElapsedTimeSeconds());
            telemetryA.addData("arm power", up.getPower());
            telemetryA.addData("armmode", up.getMode());
            telemetryA.update();
        }

        /** This method is called once at the init of the OpMode. **/
        @Override
        public void init() {
            pathTimer = new Timer();
            actionTimer = new Timer();
            opmodeTimer = new Timer();

            opmodeTimer.resetTimer();

            follower = new Follower(hardwareMap);
            follower.setStartingPose(startPose);

            telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetryA.update();

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

            servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");



            up_zero = hardwareMap.get(TouchSensor.class, "up_zero");




            //huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        }

        /** This method is called continuously after Init while waiting for "play". **/
        @Override
        public void init_loop() {

            // Scanning for Team Element
        /*HuskyLens.Block[] blocks = huskyLens.blocks();
        for (int i = 0; i < blocks.length; i++) {
            //----------------------------1----------------------------\\
            if (blocks[i].x <= 100 && blocks[i].id == 2) {
                navigation = "left";
            }
            if (blocks[i].x > 100 && blocks[i].x <= 270 && blocks[i].id == 2) {
                navigation = "middle";
            }
            if (blocks[i].x > 270 && blocks[i].id == 2) {
                navigation = "right";
            }
        }*/

            // After 4 Seconds, Robot Initialization is complete
            if (opmodeTimer.getElapsedTimeSeconds() > 4) {
                telemetryA.addData("Init", "Finished");
            }
        }

        /** This method is called once at the start of the OpMode.
         * It runs all the setup actions, including building paths and starting the path system **/
        @Override
        public void start() {
            //setBackdropGoalPose();
            buildPaths();
            opmodeTimer.resetTimer();
            setPathState(11); //starting PathState
            setArmState(0); //starting ArmState
        }

        /** We do not use this because everything should automatically disable **/
        @Override
        public void stop() {
        }
    }