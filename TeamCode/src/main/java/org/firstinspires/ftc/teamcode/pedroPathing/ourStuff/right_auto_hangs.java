package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff;


import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;




@Autonomous(name = "right_auto_hangs", group = "AUTO")
    public class right_auto_hangs extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, armState, outclawState, outgrabState;
    //Poses

    private final Pose waitPose = new Pose(22, 40, Math.toRadians(180));

    private final Pose pickupPose = new Pose(10, 40, Math.toRadians(180));

    private final Pose hangPose = new Pose(36, 65, Math.toRadians(0));

    private final Pose turnPose_s = new Pose(16, 65, Math.toRadians(180));

    private final Pose turnPose_e = new Pose(26, 65, Math.toRadians(180));


    private Path pickup;
    private Path hang;
    private Path back;

    private Path pickup2;

    private Path turn;
    // Motors
    private DcMotorEx up, out;
    private Servo servo_outtake_wrist;
    private CRServo servo_outtake;
    private TouchSensor up_zero;
    private Telemetry telemetryA;
    int up_hanging_position = 1750; //DONE: calibrate this value, viper slide position to
    int up_hanging_position_done = 1290; // calibrate

    public void buildPaths() {
        /// Concept Idea :
        /*
        Linear interpolations for turns to minimize time wasted
        hold ends on the wait time before pick up
        biezer lines now, later will be curves.

         */
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


        /*turn = new Path(
                new BezierLine(
                        new Point(turnPose_s),
                        new Point(turnPose_e)
                )
        );
        turn.setLinearHeadingInterpolation(turnPose_s.getHeading(),turnPose_e.getHeading());

         */
//
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Grabs
                follower.followPath(pickup);
                //setArmState(0);
                //setoutClawState(0);
                //setoutGrabState(2);
                if(pathTimer.getElapsedTime() > 4) { // Time to pick up
                    setPathState(1);
                }
                break;

            case 1: // going to hang
                //setArmState(1);
                //setoutGrabState(0);
                //setoutClawState(1);
                follower.followPath(hang);

                if (pathTimer.getElapsedTime() > 4) {
                    // hang it

                    setPathState(2);
                }
                break;
            case 2:
                //if (follower.getPose().getX() > (hangPose.getX() - 1) && follower.getPose().getY() > (hangPose.getY() - 1)) { // hang it
                    //setoutGrabState(3);
                    //setArmState(2);
                    //setoutClawState(2);
                    if (pathTimer.getElapsedTime() > 3) { // waiting for hang to finish
                        follower.followPath(back, true);
                        //setArmState(0);
                        //setoutGrabState(2);
                        //setoutClawState(0);

                        setPathState(3);
                    }
                //}
                break;
            case 3:
                if(pathTimer.getElapsedTime() > 3) { // waiting for back to be back, then pickup
                    //setArmState(0);
                    //setoutClawState(0);
                    //setoutGrabState(2);
                    follower.followPath(pickup,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTime() > 3) { // waiting for pickup, then follows hangpath
                    //setArmState(1);
                    //setoutGrabState(0);
                    //setoutClawState(1);
                    follower.followPath(hang);
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTime() > 3) { // waits for hang path to finish, then hangs
                    // hangs
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTime() > 3) {
                    follower.followPath(back, true);
                }
                break;

        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
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
                servo_outtake_wrist.setPosition(0.5); // Hang Position
                telemetry.addData("claw position 2", true);
                break;
            case 2: // Hang position more force
                servo_outtake_wrist.setPosition(0.3); // more force?
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

    public void setArmState(int aState) {
        armState = aState;
    }

    public void setoutClawState(int cState) {
        outclawState = cState;
    }

    public void setoutGrabState(int gstate) {
        outgrabState = gstate;
    }


    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(waitPose);
        buildPaths();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        //setup arm variable
        up = hardwareMap.get(DcMotorEx.class, "up");
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up.setDirection(DcMotorSimple.Direction.REVERSE);

        //example position setup
        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo_outtake = hardwareMap.get(CRServo.class, "outtake");

        servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");

        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Pose", follower.getPose().toString());
        // Feedback to Driver Hub
        telemetryA.addData("path state", pathState);
        telemetryA.addData("arm state", armState);
        telemetryA.addData("claw state", outclawState);
        telemetryA.addData("out grab state", outgrabState);

        telemetryA.addData("heading", follower.getPose().getHeading());
        telemetryA.addData("armPOS", up.getCurrentPosition());
        telemetryA.addData("pathtimer elapsed time", pathTimer.getElapsedTime());
        telemetryA.addData("armmode", up.getMode());
        telemetryA.addData("Follower busy", follower.isBusy());

        // check these
        telemetry.addData("Drive error", follower.driveError);

        telemetryA.addData("headingPID error", follower.headingError);
        telemetryA.update();
    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

}


