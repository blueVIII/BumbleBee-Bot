package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class GeneratedPath {
        PathBuilder builder = new PathBuilder();
    public GeneratedPath() {
        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(5.000, 40.000, Point.CARTESIAN),
                                new Point(45.000, 70.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(45.000, 70.000, Point.CARTESIAN),
                                new Point(38.000, 70.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(38.000, 70.000, Point.CARTESIAN),
                                new Point(38.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(38.000, 30.000, Point.CARTESIAN),
                                new Point(55.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(55.000, 30.000, Point.CARTESIAN),
                                new Point(55.000, 21.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(55.000, 21.000, Point.CARTESIAN),
                                new Point(10.000, 21.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(10.000, 21.000, Point.CARTESIAN),
                                new Point(45.000, 65.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(45.000, 65.000, Point.CARTESIAN),
                                new Point(38.000, 65.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(38.000, 65.000, Point.CARTESIAN),
                                new Point(38.000, 20.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(38.000, 20.000, Point.CARTESIAN),
                                new Point(55.000, 20.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(55.000, 20.000, Point.CARTESIAN),
                                new Point(55.000, 10.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(55.000, 10.000, Point.CARTESIAN),
                                new Point(10.000, 10.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(10.000, 10.000, Point.CARTESIAN),
                                new Point(45.000, 60.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation();
    }

    public PathChain getPathChain() { return builder.build(); }
}
