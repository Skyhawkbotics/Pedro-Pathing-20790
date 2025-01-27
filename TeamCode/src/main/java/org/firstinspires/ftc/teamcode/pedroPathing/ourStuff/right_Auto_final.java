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
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Config
@Autonomous(name = "RIGHT_AUTO_FINAL", group = "AUTO")
// 18.5 inches away from observation zone
public class right_Auto_final extends OpMode {
    // cool
    private Follower follower; // THe drivetrain with the calculations needed for path following
    private Timer pathTimer, opmodeTimer; // Timers for progression of states

    private int pathState, armState, outclawState, outgrabState; // Different cases and states of the different parts of the robot

    private final Pose startPose = new Pose(10.121, 67.0, Math.toRadians(0)); //TODO


    private final Pose hangPose1 = new Pose(36.0, 67.0, Math.toRadians(0)); // TODO

    private final Pose PushallEnd = new Pose(24, 9, Math.toRadians(0));

    // Pickup
    private final Pose waitPose = new Pose(22, 40, Math.toRadians(180));

    private final Pose pickupPose = new Pose(10, 40, Math.toRadians(180));

    private final Pose parkPose = new Pose(10, 20, Math.toRadians(0));

    // other hangs

    private final Pose hangPose = new Pose(36, 65, Math.toRadians(0));

    // First initial hang
    private PathChain hang1;
    // Push paths
    private Path pushAll1, pushAll2, pushAll3, pushAll4, pushAll5, pushAll6, pushAll7;
    private Path towaitPose;
    private Path pickup;
    private Path hang;
    private Path back;
    private Path park;


    // Motors
    private DcMotorEx up;
    private Servo servo_outtake_wrist;
    private CRServo servo_outtake;
    private TouchSensor up_zero;
    private Telemetry telemetryA;

    int up_hanging_position = 1750;
    /// Calibrate
    int up_hanging_position_done = 1290;

    /// Calibrate
    public void buildPaths() {

        hang1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose),
                                new Point(hangPose1)
                        )
                )
                .setLinearHeadingInterpolation(pickupPose.getHeading(), hangPose.getHeading())
                .build();

        pickup = new Path(
                new BezierLine(
                        new Point(waitPose),
                        new Point(pickupPose)
                )
        );
        pickup.setConstantHeadingInterpolation(waitPose.getHeading());
        hang = new Path(
                new BezierLine(
                        new Point(pickupPose),
                        new Point(hangPose)
                )
        );
        hang.setLinearHeadingInterpolation(pickupPose.getHeading(), hangPose.getHeading());
        back = new Path(
                new BezierLine(
                        new Point(hangPose),
                        new Point(waitPose)
                )
        );
        back.setLinearHeadingInterpolation(hangPose.getHeading(), waitPose.getHeading());


        // PUSH ALL paths

        pushAll1 = new Path( // from First hanged to push all ready position
                new BezierCurve(
                        new Point(hangPose1),
                        new Point(25.394, 55.277, Point.CARTESIAN),
                        new Point(20.903, 34.684, Point.CARTESIAN),
                        new Point(60.000, 35.000, Point.CARTESIAN)
                )
        );
        pushAll1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));

        pushAll2 = new Path( // pushes first one in
                new BezierLine(
                        new Point(60.000, 35.000, Point.CARTESIAN),
                        new Point(60.000, 29.000, Point.CARTESIAN)
                )
        );
        pushAll2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));

        pushAll3 = new Path( // Strafes down
                new BezierLine(
                        new Point(60.000, 29.000, Point.CARTESIAN),
                        new Point(24.000, 29.000, Point.CARTESIAN)
                )
        );
        pushAll3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        pushAll4 = new Path( // Goes forward in a curve probably to push the second sample
                new BezierCurve(
                        new Point(24.000, 29.000, Point.CARTESIAN),
                        new Point(63.174, 31.123, Point.CARTESIAN),
                        new Point(60.000, 18.000, Point.CARTESIAN)
                )
        );
        pushAll4.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        pushAll5 = new Path( // Pushes second
                new BezierLine(
                        new Point(60.000, 18.000, Point.CARTESIAN),
                        new Point(24.000, 18.000, Point.CARTESIAN)
                )
        );
        pushAll5.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        pushAll6 = new Path( // goes behind third
                new BezierCurve(
                        new Point(24.000, 18.000, Point.CARTESIAN),
                        new Point(63.174, 20.594, Point.CARTESIAN),
                        new Point(60.000, 16.000, Point.CARTESIAN)
                )
        );
        pushAll6.setConstantHeadingInterpolation(0);
        pushAll7 = new Path( // pushes third
                new BezierLine(
                        new Point(60.000, 9.000, Point.CARTESIAN),
                        new Point(PushallEnd)
                )
        );
        pushAll7.setConstantHeadingInterpolation(0);
        towaitPose = new Path(
                new BezierLine(
                        new Point(PushallEnd),
                        new Point(waitPose)
                )
        );
        towaitPose.setLinearHeadingInterpolation(PushallEnd.getHeading(), waitPose.getHeading());
        park = new Path(
                new BezierLine(
                        new Point(hangPose),
                        new Point(parkPose)
                )
        );
        park.setConstantHeadingInterpolation(parkPose.getHeading());


    }

    public void autonomousPathUpdate() {
        switch (pathState) { // 4 seconds
            case 0:
                follower.followPath(hang1); // drive to hang pos
                // set arms to position
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTime() > 2) {
                    // Arm Code for hang
                    setPathState(2);
                }
                break;
            case 2: // Readying push all
                if (!follower.isBusy()) {
                    // Put arms down
                    follower.followPath(pushAll1); // from First hanged to push all ready position
                    setPathState(3);
                }
                break;

            case 3: //PUSHALL START
                if (!follower.isBusy()) {
                    follower.followPath(pushAll1);
                    setPathState(4);
                }
                break;
            case 4:  // pushes first one in
                if (!follower.isBusy()) {
                    follower.followPath(pushAll2); // pushes first one in
                    setPathState(5);
                }
                break;
            case 5:  // Strafes down
                if (!follower.isBusy()) {
                    follower.followPath(pushAll3);  // Strafes down
                    setPathState(6);
                }
                break;
            case 6: // Goes forward in a curve probably to push the second sample
                if (!follower.isBusy()) {
                    follower.followPath(pushAll4); // Goes forward in a curve probably to push the second sample
                    setPathState(7);
                }
                break;
            case 7: // Pushes second
                if (!follower.isBusy()) {
                    follower.followPath(pushAll5); // Pushes second
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(pushAll6); // goes behind third
                    setPathState(9);
                }
                break;
            case 10:  // pushes third
                if (!follower.isBusy()) {
                    follower.followPath(pushAll7);  // pushes third
                    setPathState(11);
                }
                break; //PUSHALL END
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(towaitPose, true);  // pushes third
                    setPathState(12);
                }
                break;
            case 12: /// START OF FIRST HANG Grabs
                // Beginning of a loop for a bunch of hangs
                if (!follower.isBusy()) {
                    //setArmState(0);
                    //setoutClawState(0);
                    //setoutGrabState(2);
                    follower.followPath(pickup);
                    if (pathTimer.getElapsedTime() > 4) { // Time to pick up
                        setPathState(13);
                    }
                }
                break;

            case 13: // going to hang
                //setArmState(1);
                //setoutGrabState(0);
                //setoutClawState(1);
                follower.followPath(hang);

                if (pathTimer.getElapsedTime() > 4) { // Time to reach hang pose
                    // hang it with arm code
                    setPathState(14);
                }

                break;
            case 14: /// FIRST HANG DONE, HANG IT
                //if (follower.getPose().getX() > (hangPose.getX() - 1) && follower.getPose().getY() > (hangPose.getY() - 1)) { // hang it
                //setoutGrabState(3);
                //setArmState(2);
                //setoutClawState(2);
                if (pathTimer.getElapsedTime() > 3) { // waiting for hang to finish
                    follower.followPath(back, true);
                    //setArmState(0);
                    //setoutGrabState(2);
                    //setoutClawState(0);

                    setPathState(15);
                }
                //}
                break;
            case 15: /// START OF SECOND HANG
                // Pick up
                if (pathTimer.getElapsedTime() > 3) { // waiting for back to be back, then pickup
                    //setArmState(0);
                    //setoutClawState(0);
                    //setoutGrabState(2);
                    follower.followPath(pickup);
                    setPathState(16);
                }
                break;
            case 16: // going to hang pos
                if (pathTimer.getElapsedTime() > 3) { // waiting for pickup, then follows hang path
                    //setArmState(1);
                    //setoutGrabState(0);
                    //setoutClawState(1);
                    follower.followPath(hang);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTime() > 3) { // waits for hang path to finish, then hangs
                    // hangs
                    setPathState(18);
                }
                break;
            case 18: /// End of second hang
                if (pathTimer.getElapsedTime() > 3) { // waits for hang to hang
                    // arm code reset
                    follower.followPath(back, true);
                    setPathState(19);
                }
                break;
            case 19: /// Start of THIRD HANG
                if (!follower.isBusy()) {
                    //setArmState(0);
                    //setoutClawState(0);
                    //setoutGrabState(2);
                    follower.followPath(pickup);
                    if (pathTimer.getElapsedTime() > 4) { // Time to pick up
                        setPathState(20);
                    }
                }
                break;
            case 20: // going to hang pos
                //setArmState(1);
                //setoutGrabState(0);
                //setoutClawState(1);
                follower.followPath(hang);

                if (pathTimer.getElapsedTime() > 4) { // Time to reach hang pose
                    // hang it with arm code
                    setPathState(21);
                }

                break;
            case 21:
                if (pathTimer.getElapsedTime() > 3) { // waits for hang path to finish, then hangs
                    // hangs
                    setPathState(22);
                }
                break;
            case 22: /// END OF THIRD HANG
                //if (follower.getPose().getX() > (hangPose.getX() - 1) && follower.getPose().getY() > (hangPose.getY() - 1)) { // hang it
                //setoutGrabState(3);
                //setArmState(2);
                //setoutClawState(2);
                if (pathTimer.getElapsedTime() > 3) { // waiting for hang to finish
                    follower.followPath(back, true);
                    //setArmState(0);
                    //setoutGrabState(2);
                    //setoutClawState(0);

                    setPathState(23);
                }
                break;
            case 23: /// Start of Fourth Hang, Pick up
                if (pathTimer.getElapsedTime() > 3) { // waiting for back to be back, then pickup
                    //setArmState(0);
                    //setoutClawState(0);
                    //setoutGrabState(2);
                    follower.followPath(pickup);
                    setPathState(24);
                }
                break;
            case 24: // going to hang pos
                if (pathTimer.getElapsedTime() > 3) { // waiting for pickup, then follows hang path
                    //setArmState(1);
                    //setoutGrabState(0);
                    //setoutClawState(1);
                    follower.followPath(hang);
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTime() > 3) { // waits for hang path to finish, then hangs
                    // hangs
                    setPathState(26);
                }
            case 26: /// END OF FOURTH HANG
                //if (follower.getPose().getX() > (hangPose.getX() - 1) && follower.getPose().getY() > (hangPose.getY() - 1)) { // hang it
                //setoutGrabState(3);
                //setArmState(2);
                //setoutClawState(2);
                if (pathTimer.getElapsedTime() > 3) { // waiting for hang to finish
                    follower.followPath(back, true);
                    //setArmState(0);
                    //setoutGrabState(2);
                    //setoutClawState(0);

                    setPathState(27);
                }
                break;
            case 27: /// Start of Fifth Hang, Pick up
                if (pathTimer.getElapsedTime() > 3) { // waiting for back to be back, then pickup
                    //setArmState(0);
                    //setoutClawState(0);
                    //setoutGrabState(2);
                    follower.followPath(pickup);
                    setPathState(28);
                }
                break;
            case 28: // going to hang pos
                if (pathTimer.getElapsedTime() > 3) { // waiting for pickup, then follows hang path
                    //setArmState(1);
                    //setoutGrabState(0);
                    //setoutClawState(1);
                    follower.followPath(hang);
                    setPathState(29);
                }
                break;
            case 29:
                if (pathTimer.getElapsedTime() > 3) { // waits for hang path to finish, then hangs
                    // hangs
                    setPathState(30);
                }
            case 30: /// END OF Fifth HANG, park
                //if (follower.getPose().getX() > (hangPose.getX() - 1) && follower.getPose().getY() > (hangPose.getY() - 1)) { // hang it
                //setoutGrabState(3);
                //setArmState(2);
                //setoutClawState(2);
                if (pathTimer.getElapsedTime() > 3) { // waiting for hang to finish
                    follower.followPath(park, true);
                    //setArmState(0);
                    //setoutGrabState(2);
                    //setoutClawState(0);
                    if (!follower.isBusy()) {
                        telemetryA.addData("hoary you funking did it", true);
                        setPathState(-1);
                    }
                }
                break;


        }
    }

    public void autonomousActionUpdate() {
        switch (armState) {
            // Run to position on initialized??

            case 0: //going to bottom position
                up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Lowered position", true);
                if (!up_zero.isPressed()) {
                    up.setPower(-1);
                } else if (up_zero.isPressed()) {
                    up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                break;
            case 1: //going to hanging position
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up.setTargetPosition(up_hanging_position);
                if (up_hanging_position - up.getCurrentPosition() < 3) {
                    telemetry.addData("Hang Pos", up.getCurrentPosition());
                }
                break;
            case 2: //going to hanging position done
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up.setTargetPosition(up_hanging_position_done);
                if (up_hanging_position_done - up.getCurrentPosition() < 3) {
                    telemetry.addData("Hang Pos", up.getCurrentPosition());
                }
                break;
        }
        switch (outclawState) {
            case 0: // Pickup Position
                servo_outtake_wrist.setPosition(0); // TODO : calibrate
                telemetry.addData("claw position 1 ", true);
                break;
            case 1:
                servo_outtake_wrist.setPosition(0.5); /// Hang Position
                telemetry.addData("claw position 2", true);
                break;
            case 2: // Hang position more force
                servo_outtake_wrist.setPosition(0.3); /// more force?
                break;

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
    }


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


    public void setoutClawState(int cState) {
        outclawState = cState;
    }

    @Override
    public void loop() {

        // These loop the actions and movement of the robot
        follower.update();

        autonomousPathUpdate();
        autonomousActionUpdate();


        // Feedback to Driver Hub
        // States
        telemetryA.addData("         States                  ", 0);

        telemetryA.addData("path state", pathState);
        //telemetryA.addData("arm state", armState);
        //telemetryA.addData("claw state", outclawState);
        // telemetryA.addData("out grab state", outgrabState);
        telemetryA.addData("               Poses            ", 0);

        // Poses
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        telemetryA.addData("            Pathing               ", 0);

        // Pathing help
        telemetryA.addData("Follower busy", follower.isBusy());
        telemetryA.addData("headingPID error", follower.headingError);
        telemetryA.addData("          Arms           ", 0);


        // Positions of arms
        telemetryA.addData("armPOS", up.getCurrentPosition());
        telemetryA.addData("out servo", servo_outtake_wrist.getPosition());
        telemetryA.addData("path timer elapsed time", pathTimer.getElapsedTimeSeconds());
        telemetryA.addData("arm mode", up.getMode());


        telemetryA.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        buildPaths();
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
        servo_outtake = hardwareMap.get(CRServo.class, "outtake");

        servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");

        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {


        // After 4 Seconds, Robot Initialization is complete
        if (opmodeTimer.getElapsedTimeSeconds() > 4) {
            telemetryA.addData("Let's win", "DRIVER!");
        }
    }


    @Override
    public void start() {
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(0);
        setArmState(0); //starting ArmState
        setoutGrabState(0);
        setoutClawState(0);
    }


    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}
