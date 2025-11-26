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

@Autonomous(name = "AutoBlue12 - 9 pizzas", group = "Autonomous")
@Configurable // Panels
public class AutoBlue12 extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Updated starting pose to (72, 8) with a 90-degree heading (straight up)
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    // The start() method is intentionally removed.
    // The first path should be initiated from inside the autonomousPathUpdate() state machine
    // in your custom logic.

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
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

        // Path0 removed as per request
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
            // Path1
            // ------------------------------------------------------
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(60.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            // ------------------------------------------------------
            // Path2
            // ------------------------------------------------------
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.000, 84.000), new Pose(18.707, 83.746))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // ------------------------------------------------------
            // Path3
            // ------------------------------------------------------
            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.707, 83.746), new Pose(60.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            // ------------------------------------------------------
            // Path4 — curve
            // ------------------------------------------------------
            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(52.640, 54.816),
                                    new Pose(18.707, 59.601)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // ------------------------------------------------------
            // Path5
            // ------------------------------------------------------
            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.707, 59.601), new Pose(18.272, 70.477))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // ------------------------------------------------------
            // Path6 (Updated coordinates per request)
            // ------------------------------------------------------
            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.272, 70.477), new Pose(12.181, 70.477))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            // ------------------------------------------------------
            // Path7 (Start coordinate updated to match new Path6 end)
            // ------------------------------------------------------
            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.181, 70.477), new Pose(60.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            // ------------------------------------------------------
            // Path8 — curve
            // ------------------------------------------------------
            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(55.686, 29.583),
                                    new Pose(18.707, 35.674)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // ------------------------------------------------------
            // Path9
            // ------------------------------------------------------
            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.707, 35.674), new Pose(60.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }

    // --------------------------------------------------------------------
    //                       AUTONOMOUS STATE MACHINE
    // --------------------------------------------------------------------
    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}