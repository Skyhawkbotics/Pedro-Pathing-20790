package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff;


import android.os.storage.StorageManager;

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
public class right_auto extends OpMode {
    // cool
    private Follower follower; // THe drivetrain with the calculations needed for path following
    private Timer pathTimer, actionTimer, opmodeTimer, outtimer; // Timers for progression of states


    private int pathState, armState, outclawState, outgrabState, inclawState, ingrabState; // Different cases and states of the different parts of the robot

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Centerstage, this would be blue far side/red human player station.)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y. **/
    //Start Pose
    private Pose startPose = new Pose(10.121, 58.0, Math.toRadians(0)); //TODO

    private Pose hangPose = new Pose(36.5, 67.0, Math.toRadians(0)); // TODO

    private Pose firstpushPose = new Pose(24,29, Math.toRadians(0));



    // Paths
    private PathChain hang1;
    private Path pushAll1, pushAll2, pushAll3, pushAll4, pushAll5, pushAll6, pushAll7, pushAll8;

    private Path readypickup, pickup, first_hang, first_hang_back, second_hang, second_hang_back, third_hang, third_hang_back, fourth_hang, fourth_hang_back;

    // Motors
    private DcMotorEx up, out;
    private Servo servo_outtake_wrist, servo_intake_wrist;
    private CRServo servo_outtake, servo_intake;
    private TouchSensor up_zero;
    private Telemetry telemetryA;
    double intake_wrist_pos_transfer = 0;
    double outtake_wrist_pos_transfer = 0;
    int up_hanging_position = 1750; //DONE: calibrate this value, viper slide position to
    int up_hanging_position_done = 1290; //TODO: calibrate this value, position of viper slide when releasing after speciman is on the bar.

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        hang1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose),
                                new Point(hangPose)
                        )
                )
                .setConstantHeadingInterpolation(hangPose.getHeading())
                .build();
        pushAll1 = new Path(
                new BezierCurve(
                        new Point(hangPose),
                        new Point(13.231, 39.455, Point.CARTESIAN),
                        new Point(19.964, 33.549, Point.CARTESIAN),
                        new Point(75.367, 40.518, Point.CARTESIAN),
                        new Point(58.000, 24.000, Point.CARTESIAN)
                )
        );
        pushAll1.setConstantHeadingInterpolation(0);
        pushAll3 = new Path(
                new BezierLine(
                        new Point(60.000, 29.000, Point.CARTESIAN),
                        new Point(firstpushPose)
                )
        );
        pushAll3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        pushAll4 = new Path (
                new BezierCurve(
                        new Point(24.000, 29.000, Point.CARTESIAN),
                        new Point(63.174, 31.123, Point.CARTESIAN),
                        new Point(60.000, 18.000, Point.CARTESIAN)
                )
        );
        pushAll4.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        pushAll5 = new Path(
                new BezierLine(
                        new Point(60.000, 18.000, Point.CARTESIAN),
                        new Point(15.000, 18.000, Point.CARTESIAN)
                )
        );
        pushAll5.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        pushAll6 = new Path(
                new BezierCurve(
                        new Point(24.000, 18.000, Point.CARTESIAN),
                        new Point(63.174, 20.594, Point.CARTESIAN),
                        new Point(60.000, 16.000, Point.CARTESIAN)
                )
        );
        pushAll6.setConstantHeadingInterpolation(0);
        pushAll7 = new Path (
                new BezierLine(
                        new Point(60.000, 11.000, Point.CARTESIAN),
                        new Point(16, 11, Point.CARTESIAN)
                )
        );
        pushAll7.setConstantHeadingInterpolation(0);
        readypickup = new Path(
                // Line 1
                new BezierLine(
                        new Point(17.000, 9.000, Point.CARTESIAN),
                        new Point(13.000, 16.000, Point.CARTESIAN)
                )
        );
        readypickup.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270));
        pickup = new Path(
                        // Line 2
                        new BezierLine(
                                new Point(13.000, 16.000, Point.CARTESIAN),
                                new Point(13.000, 9.000, Point.CARTESIAN)
                        )
                );
        pickup.setConstantHeadingInterpolation(Math.toRadians(270));
        first_hang = new Path(
                        // Line 3
                        new BezierCurve(
                                new Point(13.000, 9.000, Point.CARTESIAN),
                                new Point(14.317, 59.162, Point.CARTESIAN),
                                new Point(36.000, 65.000, Point.CARTESIAN)
                        )
                );
        first_hang.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));
        first_hang_back = new Path(
                        // Line 4
                        new BezierCurve(
                                new Point(36.000, 65.000, Point.CARTESIAN),
                                new Point(14.199, 59.162, Point.CARTESIAN),
                                new Point(13.000, 11.000, Point.CARTESIAN)
                        )
                );
        first_hang_back.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270));
        second_hang = new Path(
                        // Line 5
                        new BezierCurve(
                                new Point(13.000, 11.000, Point.CARTESIAN),
                                new Point(14.199, 59.162, Point.CARTESIAN),
                                new Point(36.000, 65.000, Point.CARTESIAN)
                        )
                );
        second_hang.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));
        second_hang_back = new Path(
                        // Line 6
                        new BezierCurve(
                                new Point(36.000, 65.000, Point.CARTESIAN),
                                new Point(14.199, 59.162, Point.CARTESIAN),
                                new Point(13.000, 11.000, Point.CARTESIAN)
                        )
                );
        second_hang_back.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270));
        third_hang = new Path(
                        // Line 7
                        new BezierCurve(
                                new Point(13.000, 11.000, Point.CARTESIAN),
                                new Point(14.199, 59.162, Point.CARTESIAN),
                                new Point(36.000, 65.000, Point.CARTESIAN)
                        )
                );
        third_hang.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));
        third_hang_back = new Path(
                // Line 6
                new BezierCurve(
                        new Point(36.000, 65.000, Point.CARTESIAN),
                        new Point(14.199, 59.162, Point.CARTESIAN),
                        new Point(13.000, 11.000, Point.CARTESIAN)
                )
        );
        third_hang_back.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270));
        fourth_hang = new Path(
                // Line 7
                new BezierCurve(
                        new Point(13.000, 11.000, Point.CARTESIAN),
                        new Point(14.199, 59.162, Point.CARTESIAN),
                        new Point(36.000, 65.000, Point.CARTESIAN)
                )
        );
        fourth_hang.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));
        fourth_hang_back = new Path(
                // Line 6
                new BezierCurve(
                        new Point(36.000, 65.000, Point.CARTESIAN),
                        new Point(14.199, 59.162, Point.CARTESIAN),
                        new Point(13.000, 11.000, Point.CARTESIAN)
                )
        );
        fourth_hang_back.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270));

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() function on line 193)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 2:
                follower.followPath(hang1);
                setArmState(1);
                setPathState(3);
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    setArmState(0);
                    setPathState(4);
                }
                break;
            case 4:
                setoutGrabState(1);
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(5);
                }

            case 5: //PUSHALL START // curves to behind
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(pushAll1);
                    setArmState(0);
                    setPathState(7);
                }
                break;
            case 6:
                 /* if(!follower.isBusy()) {
                    follower.followPath(pushAll2);
                    setPathState(7);
                }

                 */
                break;
            case 7:
                if (/*follower.getPose().getX() > 27 && follower.getPose().getY() > 28
               */!follower.isBusy()) {
                    follower.followPath(pushAll3); // striagt back
                    setPathState(8);
                }
                break;
            case 8:
                if(follower.getPose().roughlyEquals((firstpushPose), 75)){
                //if (/*follower.getPose().getX() > 57 && follower.getPose().getY() > 23*/ !follower.isBusy()) { // curve
                    follower.followPath(pushAll4); // curvje forward
                    setPathState(9);
                }
                break;
            case 9:
                if (follower.getPose().getX() > 59 && follower.getPose().getY() > 17) {
                    follower.followPath(pushAll5); // striaght back
                    setPathState(12); //skip pushing third one to save time (very sad)
                }
                break;
            /*case 10:
                if (!follower.isBusy()) {
                    follower.followPath(pushAll6);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) { //maybe change to pos later, but it made it so it wouldnt push the third one so idk
                    follower.followPath(pushAll7);
                    setPathState(12);
                }
                break; //PUSHALL EN// D*/
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(readypickup);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(pickup);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    follower.followPath(first_hang);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    follower.followPath(first_hang_back);
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    follower.followPath(second_hang);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    follower.followPath(second_hang_back);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    follower.followPath(third_hang);
                    setPathState(19);
                }
                break;
                case 19:
                if(!follower.isBusy()) {
                    follower.followPath(third_hang_back);
                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy()) {
                    follower.followPath(fourth_hang);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
                    follower.followPath(fourth_hang_back);
                    setPathState(22);
                }
                break;
            case 22:
                telemetryA.addLine("fucking done.....     oh hi ruben lol");

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
        switch (armState) {
            case -1:
                up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//most of the code stolen from opmode_main
            case 0: //going to bottom position
                telemetry.addData("Lowered position", true);
                if (!up_zero.isPressed()) {
                    up.setPower(-1);
                } else if (up_zero.isPressed()) {
                    up.setPower(0);
                    up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                break;
            case 1: //going to hanging position
                up.setTargetPosition(up_hanging_position);
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        follower.update();

        autonomousPathUpdate();
        autonomousActionUpdate();


        // Feedback to Driver Hub
        telemetryA.addData("path state", pathState);
        telemetryA.addData("arm state", armState);
        telemetryA.addData("claw state", outclawState);
        telemetryA.addData("out grab state", outgrabState);

        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        //telemetryA.addData("armPOS", up.getCurrentPosition());
        //telemetryA.addData("out servo", servo_outtake_wrist.getPosition());
        telemetryA.addData("pathtimer elapsed time", pathTimer.getElapsedTimeSeconds());
        //telemetryA.addData("armmode", up.getMode());
        telemetryA.addData("Follower busy", follower.isBusy());

        telemetryA.addData("headingPID error", follower.headingError);
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
/*
        //example position setup
        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
        servo_outtake = hardwareMap.get(CRServo.class,"outtake");

        servo_intake = hardwareMap.get(CRServo.class, "intake");

        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");

        servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");

        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {


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
        setPathState(2);
        /* setArmState(0); //starting ArmState
        setoutGrabState(0);
        setinclawState(0);
        setIngrabState(0);
        setoutClawState(0);

         */
    }
    // run this



    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
