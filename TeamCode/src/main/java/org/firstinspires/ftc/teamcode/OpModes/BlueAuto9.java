package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.AimerSystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;

@Autonomous(name = "CalculusBC ", group = "Autonomous")
@Configurable
public class BlueAuto9 extends LinearOpMode{
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private AutoBlue6.Paths paths;

    private RobotHardware robot;
    private AimerSystem aimer;
    private ElevatorSystem elevator;
    private IntakeSystem intake;
    private ShooterSystem shooter;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose StartPose = new Pose(62.850, 9.060, Math.toRadians(90));
    private final Pose Shooting = new Pose(62.850, 12.000, Math.toRadians(105));
    private final Pose PreIntake1 = new Pose(21.5, 23.000, Math.toRadians(250));
    private final Pose PreIntake2 = new Pose(14, 23.000, Math.toRadians(250));


    private final Pose Intake = new Pose(14, 14, Math.toRadians(250));
    private final Pose PreComeBack = new Pose(18.000, 15.000, Math.toRadians(270));
    private PathChain GoShooting;
    private PathChain GoPreIntake;
    private PathChain GoPreIntakeFirst;
    private PathChain GoIntake;
    private PathChain GoPreComeBack;






    public void runOpMode() throws InterruptedException {
        // ===== INIT =====
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new RobotHardware(hardwareMap, telemetry);
        robot.init();
        aimer = new AimerSystem(robot.guideServo);
        elevator = new ElevatorSystem(robot.elevatorServo);
        intake = new IntakeSystem(robot);
        shooter = new ShooterSystem(robot, hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(62.850, 9.060, Math.toRadians(90)));



        telemetry.addLine("Готов к старту");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        //первый путь
        pathBuilder();
        follower.followPath(GoShooting);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        //второй путь
        follower.followPath(GoPreIntake);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        //третий путь
        follower.followPath(GoPreIntakeFirst);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        follower.followPath(GoIntake);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }

        // ===== 3) Интейк =====
        follower.followPath(GoIntake);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            // Интейк работает всё время пока робот едет!
            intake.update(true, false);
        }

    }


    public void pathBuilder() {
        GoShooting = follower.pathBuilder()
                .addPath(new BezierLine(StartPose, Shooting))
                .setLinearHeadingInterpolation(StartPose.getHeading(), Shooting.getHeading())
                .build();

        GoPreIntake = follower.pathBuilder()
                .addPath(new BezierCurve(Shooting,
                        PreIntake1))
                .setTangentHeadingInterpolation()
                .build();
        GoPreIntakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(PreIntake1,PreIntake2))
                .setLinearHeadingInterpolation(PreIntake1.getHeading(),PreIntake2.getHeading())
                .build();
        GoIntake = follower.pathBuilder()
                .addPath(new BezierLine(PreIntake2,Intake))
                .setLinearHeadingInterpolation(PreIntake2.getHeading(),Intake.getHeading())
                .build();


    }
    }
