package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class GeneratedPath {

    public GeneratedPath() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.800, 63.300, Point.CARTESIAN),
                                new Point(39.600, 63.300, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(39.600, 63.300, Point.CARTESIAN),
                                new Point(27.300, 63.300, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(27.300, 63.300, Point.CARTESIAN),
                                new Point(27.300, 45.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(27.300, 45.000, Point.CARTESIAN),
                                new Point(64.000, 45.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(64.000, 45.000, Point.CARTESIAN),
                                new Point(64.000, 29.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(64.000, 29.500, Point.CARTESIAN),
                                new Point(16.000, 29.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(16.000, 29.500, Point.CARTESIAN),
                                new Point(64.000, 29.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(64.000, 29.500, Point.CARTESIAN),
                                new Point(64.000, 19.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(64.000, 19.000, Point.CARTESIAN),
                                new Point(16.000, 19.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(16.000, 19.000, Point.CARTESIAN),
                                new Point(64.000, 19.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(64.000, 19.000, Point.CARTESIAN),
                                new Point(64.000, 9.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(64.000, 9.000, Point.CARTESIAN),
                                new Point(16.000, 9.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
    }
}
