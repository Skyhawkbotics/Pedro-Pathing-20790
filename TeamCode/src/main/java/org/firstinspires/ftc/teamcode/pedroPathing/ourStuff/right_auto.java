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

/**
 * This is an example auto that showcases movement and control of three servos autonomously.
 * It is able to detect the team element using a huskylens and then use that information to go to the correct spike mark and backdrop position.
 * There are examples of different ways to build paths.
 * A custom action system have been created that can be based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

@Autonomous(name = "right_auto", group = "AUTO")
public class right_auto extends OpMode {
// cool
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, armState, clawState;
    private String navigation;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Centerstage, this would be blue far side/red human player station.)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y. **/
    //Start Pose
    private Pose startPose = new Pose(8, 63.3, 0);
    //Spike mark locations
    private Pose LeftSpikeMark = new Pose(39.6, 63.3, Math.toRadians(270));
    private Pose MiddleSpikeMark = new Pose(59, 94.5, Math.toRadians(270));
    private Pose RightSpikeMark = new Pose(52, 82.75, Math.toRadians(270));
    //Backdrop zone locations
    private Pose LeftBackdrop = new Pose(44, 121.75, Math.toRadians(270));
    private Pose MiddleBackdrop = new Pose(49.5, 121.75, Math.toRadians(270));
    private Pose RightBackdrop = new Pose(58, 121.25, Math.toRadians(270));
    private Pose WhiteBackdrop = new Pose(40, 122.25, Math.toRadians(270));

    // Poses and Paths for Purple and Yellow
    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;
    private Path scoreSpikeMark, firstHang, scoreSpikeMarkChosen;

    // White Stack Cycle Poses + Path Chains
    private Pose TopTruss = new Pose(28, 84, Math.toRadians(270));
    private Pose BottomTruss = new Pose(28, 36, Math.toRadians(270));
    private Pose Stack = new Pose(46, 11.5, Math.toRadians(270));
    private PathChain pushAll, restHangs;
    // Motors
    private DcMotorEx up, out;
    private Servo servo_outtake_wrist;
    private CRServo servo_outtake;

    private TouchSensor up_zero;
    private int up_true_target_pos;
    int up_hanging_position = 1300; //TODO: calibrate this value, viper slide position to 

    /** Generate Spike Mark and Backdrop Paths based off of the team element location **/
    public void setBackdropGoalPose() {
        spikeMarkGoalPose = new Pose(LeftSpikeMark.getX(), LeftSpikeMark.getY(), Math.toRadians(270));
        initialBackdropGoalPose = new Pose(LeftBackdrop.getX(), LeftBackdrop.getY(), Math.toRadians(270));
        firstCycleBackdropGoalPose = new Pose(WhiteBackdrop.getX(), WhiteBackdrop.getY(), Math.toRadians(270));
        scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(8.5, 80.5, Point.CARTESIAN), new Point(48, 135, Point.CARTESIAN), new Point(LeftSpikeMark)));
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        /** There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require > 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://www.desmos.com/calculator/3so1zx0hcd).
         *    * BezierLines are straight, and require 2 points. There are the start and end points. **/

        firstHang = new Path(new BezierLine(new Point(9.800, 63.300, Point.CARTESIAN), new Point(39.600, 63.300, Point.CARTESIAN)));
        firstHang.setConstantHeadingInterpolation(Math.toRadians(0));
        firstHang.setPathEndTimeoutConstraint(0);

        /** This is a path chain, defined on line 66
         * It, well, chains multiple paths together. Here we use a constant heading from the board to the stack.
         * On line 97, we set the Linear Interpolation,
         * which means that Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path */


        pushAll = follower.pathBuilder()
            .addPath(new BezierLine(new Point(27.300, 63.300, Point.CARTESIAN), new Point(27.300, 45.000, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(new BezierLine(new Point(27.300, 45.000, Point.CARTESIAN), new Point(64.000, 45.000, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(new BezierLine(new Point(64.000, 45.000, Point.CARTESIAN), new Point(64.000, 29.500, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(new BezierLine(new Point(64.000, 29.500, Point.CARTESIAN), new Point(16.000, 29.500, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(new BezierLine(new Point(16.000, 29.500, Point.CARTESIAN), new Point(64.000, 29.500, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(new BezierLine(new Point(64.000, 29.500, Point.CARTESIAN), new Point(64.000, 19.000, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(new BezierLine(new Point(64.000, 19.000, Point.CARTESIAN), new Point(16.000, 19.000, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(new BezierLine(new Point(16.000, 19.000, Point.CARTESIAN), new Point(64.000, 19.000, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(new BezierLine(new Point(64.000, 19.000, Point.CARTESIAN), new Point(64.000, 9.000, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(new BezierLine(new Point(64.000, 9.000, Point.CARTESIAN), new Point(16.000, 9.000, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

        /*restHangs = follower.pathBuilder() //can add more later, but is useful as it can stay in the position while waiting to run the next thing.
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(30 + 14, 91.6, Point.CARTESIAN), new Point(13 + 14, 130.8, Point.CARTESIAN), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierCurve(new Point(BottomTruss), new Point(20.5 + 14, 10, Point.CARTESIAN), new Point(42 + 14, 35, Point.CARTESIAN), new Point(Stack)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();*/
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() function on line 193)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 2.6) { //just wait for a bit, idk why it was in the example... TODO: maybe remove this later or shorten it
                    setPathState(12);
                }
                break;
            case 12: //arm up, give it time to get up before moving, remember that timer resets when case changes, as stated above
                setArmState(1); //put arm up
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(13);
                }
                break;
            case 13: //drive to firsthang and wait before putting arm back down
                follower.followPath(firstHang);
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(14);
                }
                break;
            case 14: //arm down
                setArmState(0);
                break;
            case 15: //push the rest using pushAll
                follower.followPath(pushAll);
                setPathState(16);
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
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    up.setVelocity(-1200);
                    up_true_target_pos = 0;
                } else if (up_zero.isPressed()) {
                    up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                break;
            case 1: //going to hanging position
                telemetry.addData("Hang position", true);
                up.setTargetPosition(up_hanging_position);
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
        }
        switch (clawState) {
            case 0:
                servo_outtake_wrist.setPosition(0.45);
                telemetry.addData("Hang position 1 ", true);
                break;
            case 1:
                servo_outtake_wrist.setPosition(0.75);
                telemetry.addData("hang position 2", true);

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
        pathTimer.resetTimer();
    }

    public void setClawState(int cState) {
        clawState = cState;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the actions and movement of the robot
        follower.update();
        autonomousPathUpdate();
        autonomousActionUpdate();

        /*//Huskylens Setup
        Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
        rateLimit.expire();
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);*/

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
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

        //setup arm variable
        up = hardwareMap.get(DcMotorEx.class, "up");
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up.setDirection(DcMotorSimple.Direction.REVERSE);

        //example position setup
        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        out.setDirection(DcMotorSimple.Direction.REVERSE);

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
            telemetry.addData("Init", "Finished");
        }
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        setBackdropGoalPose();
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
