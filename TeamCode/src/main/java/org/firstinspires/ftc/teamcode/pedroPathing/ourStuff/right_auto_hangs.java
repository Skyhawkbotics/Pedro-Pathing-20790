package org.firstinspires.ftc.teamcode.pedroPathing.ourStuff;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
    private int pathState, armState, clawState;

    //Poses

    private final Pose startPose = new Pose(18,40, Math.toRadians(180));

    private final Pose pickupPose = new Pose(10,40, Math.toRadians(180));

    private final Pose hangPose = new Pose(38,65,Math.toRadians(0));

    private final Pose turnPose_s = new Pose(16,65, Math.toRadians(180));

    private final Pose turnPose_e = new Pose(26, 65, Math.toRadians(180));


    private Path pickup;
    private Path hang;
    private Path back;

    private Path turn;

    public void buildPaths() {
        pickup = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(pickupPose)
                )
        );
        pickup.setConstantHeadingInterpolation(startPose.getHeading());
        hang = new Path(
                new BezierLine(
                        new Point(pickupPose),
                        new Point(hangPose)
                )
        );
        hang.setLinearHeadingInterpolation(pickupPose.getHeading(),hangPose.getHeading());
        back = new Path(
                new BezierLine(
                        new Point(hangPose),
                        new Point(startPose)
                )
        );
        back.setTangentHeadingInterpolation();
        /*turn = new Path(
                new BezierLine(
                        new Point(turnPose_s),
                        new Point(turnPose_e)
                )
        );
        turn.setLinearHeadingInterpolation(turnPose_s.getHeading(),turnPose_e.getHeading());

         */

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pickup, true);
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(1);
                }
                break;

            case 1:
                //if(follower.getPose().getX() > (pickupPose.getX() - 1) && follower.getPose().getY() > (pickupPose.getY() - 1))) {
                if(!follower.isBusy()) {
                    follower.followPath(hang, true);
                    if (pathTimer.getElapsedTimeSeconds() > 4) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                //if(follower.getPose().getX() > (hangPose.getX() - 1) && follower.getPose().getY() > (hangPose.getY() - 1))) {
                if(!follower.isBusy()) {
                    follower.followPath(back, true);
                }

                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
            pathState = pState;
            pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State",pathState);
        telemetry.addData("X pos", follower.getPose().toString());
        telemetry.update();
    }
}


