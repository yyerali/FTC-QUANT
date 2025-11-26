package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoBlue12 - 9 Pizzas", group = "Autonomous")
@Configurable
public class AutoBlue12 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;

    private int pathState = 0;
    private Paths paths;

    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Starting pose set by you
        follower.setStartingPose(new Pose(48, 96, Math.toRadians(135)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        follower.followPath(paths.Path0);
        pathState = 0;
    }

    @Override
    public void loop() {

        follower.update();

        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update(telemetry);
    }

    // --------------------------------------------------------------------
    //                          PATH DEFINITIONS
    // --------------------------------------------------------------------

    public static class Paths {

        public PathChain Path0;
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {

            // ------------------------------------------------------
            // Path0 — initial move from starting pose to first pose
            // ------------------------------------------------------
            Path0 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(72, 8),
                            new Pose(48.000, 96.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            // ------------------------------------------------------
            // Path1
            // ------------------------------------------------------
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48.000, 96.000),
                            new Pose(60.000, 84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            // ------------------------------------------------------
            // Path2
            // ------------------------------------------------------
            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(60.000, 84.000),
                            new Pose(18.707, 83.746)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // ------------------------------------------------------
            // Path3
            // ------------------------------------------------------
            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.707, 83.746),
                            new Pose(60.000, 84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            // ------------------------------------------------------
            // Path4 — curve
            // ------------------------------------------------------
            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(60.000, 84.000),
                            new Pose(52.640, 54.816),
                            new Pose(18.707, 59.601)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // ------------------------------------------------------
            // Path5
            // ------------------------------------------------------
            Path5 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.707, 59.601),
                            new Pose(18.272, 70.477)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // ------------------------------------------------------
            // Path6
            // ------------------------------------------------------
            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.272, 70.477),
                            new Pose(14.574, 70.695)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            // ------------------------------------------------------
            // Path7
            // ------------------------------------------------------
            Path7 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(14.574, 70.695),
                            new Pose(60.000, 84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            // ------------------------------------------------------
            // Path8 — curve
            // ------------------------------------------------------
            Path8 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(60.000, 84.000),
                            new Pose(55.686, 29.583),
                            new Pose(18.707, 35.674)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // ------------------------------------------------------
            // Path9
            // ------------------------------------------------------
            Path9 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.707, 35.674),
                            new Pose(60.000, 84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }

    // --------------------------------------------------------------------
    //                       AUTONOMOUS STATE MACHINE
    // --------------------------------------------------------------------
    public int autonomousPathUpdate() {

        if (!follower.isBusy()) {

            switch (pathState) {

                case 0:
                    follower.followPath(paths.Path1);
                    break;

                case 1:
                    follower.followPath(paths.Path2);
                    break;

                case 2:
                    follower.followPath(paths.Path3);
                    break;

                case 3:
                    follower.followPath(paths.Path4);
                    break;

                case 4:
                    follower.followPath(paths.Path5);
                    break;

                case 5:
                    follower.followPath(paths.Path6);
                    break;

                case 6:
                    follower.followPath(paths.Path7);
                    break;

                case 7:
                    follower.followPath(paths.Path8);
                    break;

                case 8:
                    follower.followPath(paths.Path9);
                    break;

                case 9:
                    // Done
                    break;
            }

            pathState++;
        }

        return pathState;
    }
}
