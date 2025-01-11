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
@Autonomous(name = "RIGHT_AUTO", group = "AUTO")
// 18.5 inches away from observation zone
public class tuning_test extends OpMode {
    // cool
    private Follower follower; // THe drivetrain with the calculations needed for path following
    private Timer pathTimer, actionTimer, opmodeTimer, outtimer; // Timers for progression of states


    private int pathState, armState, outclawState, outgrabState, inclawState, ingrabState; // Different cases and states of the different parts of the robot
    private String navigation;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Centerstage, this would be blue far side/red human player station.)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y. **/
    //Start Pose
    private Pose startPose = new Pose(10.121, 63.0, Math.toRadians(0)); //TODO

    private Pose hangPose = new Pose(35.951, 63.0, Math.toRadians(0)); // TODO

    private Pose hangPose1 = new Pose(36.0, 60.0, Math.toRadians(0)); // TODO

    private Pose pickupPose = new Pose(16, 43, Math.toRadians(180)); // TODO : THISx value

    private Pose firstpoint = new Pose(36.577, 41.371, Math.toRadians(0));

    private Pose control_p1 = new Pose(15.625154130702835, 22.905055487053016, Math.toRadians(0));

    private Pose control_p2 = new Pose(51.49198520345253, 44.74475955610358, Math.toRadians(0));


    // Paths

    private PathChain back_park, specimen_hang, back, park, hang2, hang3, push_pos;







    // Motors

    private DcMotorEx up, out;
    private Servo servo_outtake_wrist;
    private CRServo servo_outtake;

    private Servo servo_intake_wrist;

    private CRServo servo_intake;

    private TouchSensor up_zero;
    private Telemetry telemetryA;

    double intake_wrist_pos_transfer = 0;
    double outtake_wrist_pos_transfer = 0;
    int up_hanging_position = 1750; //DONE: calibrate this value, viper slide position to
    int up_hanging_position_done = 1310; //TODO: calibrate this value, position of viper slide when releasing after speciman is on the bar.

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /** There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require > 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://www.desmos.com/calculator/3so1zx0hcd).
         *    * BezierLines are straight, and require 2 points. There are the start and end points. **/

        /** This is a path chain, defined on line 66
         * It, well, chains multiple paths together. Here we use a constant heading from the board to the stack.
         * On line 97, we set the Linear Interpolation,
         * which means that Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path */
        // Heading Interpolation determines how much the robots heading changes along the path; Linear gradually transitions between state and end headin
        // Constant maintains the fixed heading
        // Tangential Aligns the heading with path direction
        // This is the preload path
        /*
        back = new Path(new BezierLine(new Point(hangPose), new Point(back_Pose)));
        back.setConstantHeadingInterpolation(Math.toRadians(0));
        park = new Path(new BezierLine(new Point(back_Pose), new Point(parkPose)));
        park.setConstantHeadingInterpolation(Math.toRadians(0));

         */

        specimen_hang = follower.pathBuilder() // Hangs, then picks up a specimen and drives forward again
                // purpose of this is to test tuning
                .addPath(
                        // Line 1
                        new BezierLine(
                                // new Point(startPose),
                                // new Point(hangPose),
                                new Point(startPose), // Start Pose
                                new Point(hangPose)   // hang pose
                        )
                )
                .setTangentHeadingInterpolation() // heading
                .build();
        back = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(hangPose), // hang pose
                                new Point(pickupPose) // observation zone pick up point
                        )
                )
                .setLinearHeadingInterpolation(hangPose.getHeading(),pickupPose.getHeading())

                //.setConstantHeadingInterpolation(Math.toRadians(0))
                //.setConstantHeadingInterpolation(Math.toRadians(180))
                //.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180)) // Turning
                .build();
        hang2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickupPose),
                                new Point(hangPose1)
                        )
                )
                .setLinearHeadingInterpolation(pickupPose.getHeading(), hangPose.getHeading())
                .build();
        hang3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickupPose),
                                new Point(hangPose1) // new hang pos
                        )
                )
                .setLinearHeadingInterpolation(pickupPose.getHeading(),hangPose1.getHeading())
                .build();
        push_pos = follower.pathBuilder()
        .addPath(
                // Line 1
                new BezierLine(
                        new Point(hangPose1),
                        new Point(36.577, 41.371, Point.CARTESIAN)
                )
        )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(36.577, 41.371, Point.CARTESIAN),
                                new Point(63.033, 41.194, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(63.033, 41.194, Point.CARTESIAN),
                                new Point(62.856, 22.905, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


        /*park = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(10.298, 39.063, Point.CARTESIAN), // observation zone
                                new Point(66.762, 39.596, Point.CARTESIAN) // forward straight
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0)) // turning
                .build();

         */

        // Now I'm adding a Path chain of actions that will go back and strafe and turn to park (for tuning)
        /*back = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hangPose), new Point(back_Pose)))
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(back_Pose), new Point(parkPose)))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();

         */
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() function on line 193)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Drive and hang pos
                follower.followPath(specimen_hang);

                setArmState(1); // hang pos
                setoutClawState(1); // set pos to hanging pos

                if(follower.getPose().getX() > (hangPose.getX() -1)) {
                    setArmState(2);// down
                    setPathState(1); // move on
                }
                break;
            case 1: // Hang and release
                if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                    setoutGrabState(1); // release
                    setPathState(2);
                }
                    break;
            case 2: // Pickup Position
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(back);
                        setArmState(0);
                        setoutGrabState(2); // in

                        //if (pathTimer.getElapsedTimeSeconds() > 3) {
                            if ((follower.getPose().getX() - pickupPose.getX()) < 1) { // prox sensor TODO : shorten?
                                setPathState(3);
                            }
                        //}
                    }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 2) {
                    setArmState(1); // raise
                    setoutGrabState(0);
                    setPathState(4);
                }
                break;
            case 4:
                    follower.followPath(hang2); // drive to hang pos
                    if (pathTimer.getElapsedTimeSeconds() > 4) { // waiting for it to reach pos // todo SHORTEN?
                        setArmState(2); // hang
                        if (up.getPower() == 0.01) {
                            setoutGrabState(1); // release
                            setPathState(4); // move on
                        }
                    }
                break;
            case 5:
               // if (pathTimer.getElapsedTimeSeconds() > 1) {
                    //follower.followPath(push_pos);
                  //  setoutGrabState(0);
                 //   setArmState(0);
                //}
               // break;




        }
    }
    /* back_park = follower.pathBuilder()
            .addPath(
            // Line 1
                        new BezierLine(
                    new Point(startPose),
                                new Point(59.660, 84.873, Point.CARTESIAN)
                        )
                                )
                                .setTangentHeadingInterpolation()
                .addPath(
            // Line 2
                        new BezierLine(
                    new Point(59.660, 84.873, Point.CARTESIAN),
                                new Point(10.121, 85.051, Point.CARTESIAN)
                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

     */

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
                if (up.getCurrentPosition() > up_hanging_position_done) {
                    up.setPower(-0.7);
                    telemetry.addData("arm moving", true);
                } else if (up.getCurrentPosition() <= up_hanging_position_done) {
                    up.setPower(0.01);
                }
                break;
        }
        switch (outclawState) {
            case 0:
                servo_outtake_wrist.setPosition(0);
                telemetry.addData("claw position 1 ", true);
                break;
            case 1:
                servo_outtake_wrist.setPosition(0.5);
                telemetry.addData("claw position 2", true);

        }
        switch (outgrabState) {
            case 0:
                servo_outtake.setPower(0);
                break;
            case 1: //release
                servo_outtake.setPower(1);
                break;
            case 2: //grab
                servo_outtake.setPower(-1);
                break;
            case 3: // hang release?
                if (up.getCurrentPosition() < 1500) {
                   servo_outtake.setPower(1);
                }
        }
        switch (inclawState) {
            case 0:
                servo_intake_wrist.setPosition(0);
                break;
            case 1:
                servo_intake_wrist.setPosition(0.5);
                break;

        }
        switch (ingrabState) {
            case 0:
                servo_intake.setPower(0);
                break;
            case 1: // Release?
                servo_intake.setPower(1);
                break;
            case 2:
                servo_intake.setPower(-1);
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
    public void setoutGrabState(int gstate) {
        outgrabState = gstate;
    }
    public void setinclawState(int icstate) {
        inclawState = icstate;
    }
    public void setIngrabState(int icstate) {
        ingrabState = icstate;
    }



    public void setoutClawState(int cState) {
        outclawState = cState;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the actions and movement of the robot
        autonomousPathUpdate();
        follower.update();
        autonomousActionUpdate();

        //follower.telemetryDebug(telemetryA);


        // Feedback to Driver Hub
        telemetryA.addData("path state", pathState);
        telemetryA.addData("arm state", armState);
        telemetryA.addData("claw state", outclawState);
        telemetryA.addData("out grab state", outgrabState);

        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        telemetryA.addData("armPOS", up.getCurrentPosition());
        telemetryA.addData("out servo", servo_outtake_wrist.getPosition());
        telemetryA.addData("pathtimer elapsed time", pathTimer.getElapsedTimeSeconds());
        telemetryA.addData("armmode", up.getMode());
        telemetryA.addData("Follower busy", follower.isBusy());
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

        servo_intake = hardwareMap.get(CRServo.class, "intake");

        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");

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
        setPathState(0);
        setArmState(0); //starting ArmState
        setoutGrabState(0);
        setinclawState(0);
        setIngrabState(0);
        setoutClawState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
