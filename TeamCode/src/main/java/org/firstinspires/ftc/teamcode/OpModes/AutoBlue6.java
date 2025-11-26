package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.AimerSystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PedroBezierAuto", group = "Autonomous")
@Configurable
public class AutoBlue6 extends LinearOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    private RobotHardware robot;
    private AimerSystem aimer;
    private ElevatorSystem elevator;
    private IntakeSystem intake;
    private ShooterSystem shooter;

    @Override
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
        follower.setStartingPose(new Pose(61.907, 9.869, Math.toRadians(110)));

        paths = new Paths(follower);

        telemetry.addLine("Готов к старту");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // ===== 1) Первые 3 выстрела =====
        shootThreeBalls();

        // ===== 2) Движение по Path1 =====
        follower.followPath(paths.Path1);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }

        // ===== 3) Интейк =====
        intake.update(true, false);
        sleep(2000); // собираем шары
        intake.update(false, false);

        // ===== 4) Возврат по Path2 задом =====
        follower.followPath(paths.Path2);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }

        // ===== 5) Вторые 3 выстрела =====
        shootThreeBalls();

        telemetry.addLine("Автоном завершён");
        telemetry.update();
    }

    // =========================
    // СТРЕЛЬБА (по логике AutoCode)
    // =========================
    private void shootThreeBalls() {
        // Поднять aimer заранее
        aimer.goUp();
        sleep(300);

        // Включаем шутер
        shooter.toggle();

        // Разгон RPM 2 секунды
        long rampStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - rampStart < 2000) {
            shooter.updateShooter(true);
        }

        // Функция выстрела
        Runnable shoot = () -> {
            elevator.goToTop();
            sleep(350);
            elevator.goToBottom();
            sleep(250);
        };

        // 🎯 Выстрел #1
        shoot.run();
        holdShooter(150);
        runIntake(1500, true);

        // 🎯 Выстрел #2
        shoot.run();
        holdShooter(150);
        runIntake(1500, true);

        // 🎯 Выстрел #3
        shoot.run();

        // Выключаем шутер
        shooter.stop();
        sleep(200);
    }

    private void holdShooter(long ms) {
        long t = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t < ms) {
            shooter.updateShooter(true);
        }
    }

    private void runIntake(long ms, boolean shooterOn) {
        long t = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t < ms) {
            intake.update(true, false);
            if (shooterOn) shooter.updateShooter(true);
        }
        intake.update(false, false);
    }

    // =========================
    // Paths
    // =========================
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(61.907, 9.869),
                            new Pose(52.486, 26.243),
                            new Pose(9.869, 9.645)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(9.869, 9.645),
                            new Pose(54.729, 26.243),
                            new Pose(62.131, 11)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed() // движение задом
                    .build();
        }
    }
}
