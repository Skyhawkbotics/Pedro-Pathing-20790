package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

/**
 * This is the TeleOpEnhancements OpMode. It is an example usage of the TeleOp enhancements that
 * Pedro Pathing is capable of.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/21/2024
 */
@TeleOp(name = "opmode_MAIN", group = "MAIN")
public class opmode_MAIN extends OpMode {
    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    //define these up here instead of in the init section like rr, idk why but it seems to work fine.
    private Servo servo_outtake_wrist;
    private Servo servo_intake_wrist;
    private CRServo servo_intake;
    private CRServo servo_outtake;
    private DcMotorEx up;
    private DcMotorEx out;
    private TouchSensor up_zero;
    private TouchSensor out_zero;
    private TouchSensor out_out;

    //from rr version of opmode_MAIN
    int arm_upper_lim = 4350;
    int up_true_target_pos;
    int out_true_target_pos;
    double servo_outtake_wrist_location = 0;
    double servo_intake_wrist_location = 0;


    //vars for set positions for transfer:
    /// DONE FOR NOW (do when we try full auto transfer: CHANGE THESE
    // int transfer_step = 0;
    double intake_wrist_pos_transfer = 0;
    double outtake_wrist_pos_transfer = 0;
    int out_pos_transfer = 0;//TODO: edit this for calibration!

    int up_specimen_hang = 1907; // Viper

    double outtake_specimen_hang = 0.45;
    int up_pos_transfer1 = 0;

    // double up_pos_transfer2 = 10;
    // double up_pos_transfer3 = 20;
    // double outtake_wrist_pos_ready = 300;


    //time stuff
    double last_time = 0;
    private final ElapsedTime runtime = new ElapsedTime();

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();


        //from rr version

        //setup arm to use velocity
        //setup arm variable
        up = hardwareMap.get(DcMotorEx.class, "up");
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up.setDirection(DcMotorSimple.Direction.REVERSE);

        //example position setup
        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //example velocity setup
        //up = hardwareMap.get(DcMotorEx.class, "up");
        //up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //up.setDirection(DcMotorSimple.Direction.REVERSE);

        //setup servos for intake and outtake
        servo_intake = hardwareMap.get(CRServo.class, "intake");
        servo_outtake = hardwareMap.get(CRServo.class, "outtake");
        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");
        servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");


        //initialize touch sensor
        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");
        out_zero = hardwareMap.get(TouchSensor.class, "out_zero");
        out_out = hardwareMap.get(TouchSensor.class, "out_out");
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        //drive code from TeleOpEnhancements
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();


        //from rr version

        //viper slide
        if (gamepad2.left_stick_y < -0.1) { //left stick -, is going up! (I think it's inverted)
            //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
            up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            up.setVelocity(gamepad2.left_stick_y * -1200);
            up_true_target_pos = 0;
        } else if (!up_zero.isPressed() && gamepad2.left_stick_y > 0.1) { //left stick +, going down
            up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            up.setVelocity(gamepad2.left_stick_y * -1200);
            up_true_target_pos = 0;
        } else if (up_zero.isPressed() && gamepad2.left_stick_y > 0.1) { // Lower limit for up
            telemetry.addData("Lower Limit Reached", up_zero);
            up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            up.setPower(500);
            //use position mode to stay up, as otherwise it would fall down. do some fancy stuff with up_true_target_pos to avoid the issue of it very slightly falling every tick
            if (up_true_target_pos == 0) {
                up.setTargetPosition(up.getCurrentPosition());
                up_true_target_pos = up.getCurrentPosition();
            }
            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        // Misumi Slide
        if (gamepad2.right_stick_y > 0.1 && !out_out.isPressed()) { //out
            //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
            out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            out.setVelocity(gamepad2.right_stick_y * 1000);
            out_true_target_pos = 0;
        } else if (gamepad2.right_stick_y < -0.1 && !out_zero.isPressed()) { //in
            out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            out.setVelocity(gamepad2.right_stick_y * 1000);
            out_true_target_pos = 0;
        } else {
            out.setPower(500);
            //use position mode to stay up, as otherwise it would fall down. do some fancy stuff with up_true_target_pos to avoid the issue of it very slightly falling every tick
            if (out_true_target_pos == 0) {
                out.setTargetPosition(out.getCurrentPosition());
                out_true_target_pos = out.getCurrentPosition();
                out.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        }


        //make sure the upper and lower limits are actually at the upper and lower limits
        if (up.getCurrentPosition() < 0) {
            up.setTargetPosition(0);
        } else if (up.getCurrentPosition() > arm_upper_lim) {
            up.setTargetPosition(arm_upper_lim);
        }


        // Gamepad2.right_trigger is analog, so we need a comparative statement to use it as a digital button.
        //servo intake control
        if (gamepad2.right_trigger > 0.8/* && servo_CLAW_position < 1000000000*/) { //NO LONGER NEEDED: find a better solution for this limits so we can actually use them
            servo_intake.setPower(1);
        } else if (gamepad2.right_bumper) { //NO LONGER NEEDED: these limits too.
            servo_intake.setPower(-1);
        } else {
            servo_intake.setPower(0);
        }


        //Continuous servo outtake control
        if (gamepad2.left_trigger > 0.8/* && servo_CLAW_position < 1000000000*/) { //NO LONGER NEEDED: find a better solution for this limits so we can actually use them
            servo_outtake.setPower(1);
        } else if (gamepad2.left_bumper) { //NO LONGER NEEDED: these limits too.
            servo_outtake.setPower(-1);
        } else {
            servo_outtake.setPower(0);
        }

        // manual outtake wrist location
        if (gamepad2.dpad_up) {
            servo_outtake_wrist_location += 0.03;
        }
        if (gamepad2.dpad_down) {
            servo_outtake_wrist_location -= 0.03;
        }

        if (servo_outtake_wrist_location > 1) {
            servo_outtake_wrist_location = 1;
        } else if (servo_outtake_wrist_location < 0) {
            servo_outtake_wrist_location = 0;
        }

        servo_outtake_wrist.setPosition(servo_outtake_wrist_location);

        // manual intake wrist location
        if (gamepad2.dpad_left) {
            servo_intake_wrist_location += 0.05;
        }
        if (gamepad2.dpad_right) {
            servo_intake_wrist_location -= 0.05;
        }
        // limits
        if (servo_intake_wrist_location > 1) {
            servo_intake_wrist_location = 1;
        } else if (servo_intake_wrist_location < 0) {
            servo_intake_wrist_location = 0;
        }

        servo_intake_wrist.setPosition(servo_intake_wrist_location);


        //Encoder Transfer Method
        if (gamepad2.b) { // He needs to hold B down for entire thing to work
            //Add a variable and thing for setting the viper slide position to about 250 to avoid smashing stuff together
            up.setTargetPosition(2);
            servo_outtake_wrist_location = outtake_wrist_pos_transfer;
            servo_intake_wrist_location = intake_wrist_pos_transfer;
            out.setTargetPosition(0);
            telemetry.addData("Misumi Slide Moving", true);
        }
        if (gamepad2.a) { //transfer
            servo_outtake.setPower(-1);
            servo_intake.setPower(1);
        }
        if (gamepad2.y) { //goto hanging position
            up.setTargetPosition(up_specimen_hang);
            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servo_outtake_wrist_location = 0.3;
        }
    }
}
