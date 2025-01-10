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

public class left_auto {
    public class GeneratedPath {

        public GeneratedPath() {
            PathBuilder builder = new PathBuilder();

            builder
                    .addPath(
                            // Line 1
                            new BezierLine(
                                    new Point(8.990, 88.000, Point.CARTESIAN),
                                    new Point(38.955, 82.405, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 2
                            new BezierLine(
                                    new Point(38.955, 82.405, Point.CARTESIAN),
                                    new Point(31.297, 122.025, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 3
                            new BezierLine(
                                    new Point(31.297, 122.025, Point.CARTESIAN),
                                    new Point(11.320, 132.347, Point.CARTESIAN)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            // Line 4
                            new BezierLine(
                                    new Point(11.320, 132.347, Point.CARTESIAN),
                                    new Point(33.461, 133.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-15))
                    .addPath(
                            // Line 5
                            new BezierLine(
                                    new Point(33.461, 133.000, Point.CARTESIAN),
                                    new Point(11.320, 132.180, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(145))
                    .addPath(
                            // Line 6
                            new BezierLine(
                                    new Point(11.320, 132.180, Point.CARTESIAN),
                                    new Point(33.295, 133.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(35))
                    .addPath(
                            // Line 7
                            new BezierLine(
                                    new Point(33.295, 133.000, Point.CARTESIAN),
                                    new Point(11.487, 132.347, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(145));
        }
    }
}
