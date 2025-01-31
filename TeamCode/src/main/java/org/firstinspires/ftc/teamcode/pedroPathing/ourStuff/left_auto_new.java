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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.NanoTimer;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Config
@Autonomous(name = "left_auto", group = "AUTO")
public class left_auto_new extends OpMode {
    private Follower follower; // THe drivetrain with the calculations needed for path following
    private Timer actionTimer, opmodeTimer, outtimer; // Timers for progression of states

    private NanoTimer pathTimer;
    private int pathState, upState, vWristState, servoOuttakeState; // Different cases and states of the different parts of the robot
    private Pose startPose = new Pose(8.65,86.23,Math.toRadians(90)); //facing the basket with the front lined up with the edge of the second tile from the left
    private Pose first_basket = new Pose(8.82, 132.84, Math.toRadians(90));

    private Path basket1;

    // Motors
    private DcMotorEx up, out;
    private Servo servo_outtake_wrist, servo_intake_wrist;
    private CRServo servo_outtake, servo_intake;
    private TouchSensor up_zero;
    private Telemetry telemetryA;
    double intake_wrist_pos_transfer = 0;
    double outtake_wrist_pos_transfer = 0;
    int up_basket_position = 1330;

    public void buildPaths() {
        basket1 = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(first_basket)
                )
        );
        basket1.setTangentHeadingInterpolation();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(basket1);
                setUpState(1);
                setVWristState(2);
                setServoOuttakeState(2);
                break;



        }
    }

    public void autonomousActionUpdate() {
        switch (upState) {
            case 0:
                break;
            case 1:
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up.setTargetPosition(up_basket_position);
                break;
        }
        switch (vWristState) {
            case 0:
                break;
            case 1:
                break;
            case 2:
                servo_outtake_wrist.setPosition(1);
                break;
        }
        switch (servoOuttakeState) {
            case 0:
                break;
            case 1:
                break;
            case 2:
                servo_outtake.setPower(1); //TODO: I have no idea if this is the right way
                break;
        }

    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void setUpState(int uState) {
        upState = uState;
    }
    public void setVWristState(int vwState) {
        vWristState = vwState;
    }
    public void setServoOuttakeState(int outtakeState) {
        servoOuttakeState = outtakeState;
    }

    @Override
    public void loop() {

        // These loop the actions and movement of the robot
        follower.update();

        autonomousPathUpdate();
        autonomousActionUpdate();


        // Feedback to Driver Hub
        telemetryA.addData("path state", pathState);
        telemetryA.addData("UP state", upState);
        telemetryA.addData("viper wrist state", vWristState);
        telemetryA.addData("outtake servo state", servoOuttakeState);

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
        pathTimer = new NanoTimer();
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
        out.setTargetPosition(0);
        out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        out.setPower(1);

        servo_outtake = hardwareMap.get(CRServo.class,"outtake");

        //servo_intake = hardwareMap.get(CRServo.class, "intake");

        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");

        servo_intake_wrist.setPosition(-1);

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
        setPathState(0);
        setUpState(1); //starting ArmState
        setVWristState(2);
        setServoOuttakeState(0);


    }
    // run this



    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
