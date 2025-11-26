package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous(name = "TripleShotAuto", group = "Auto")
public class AutoCode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ------------------------------
        // INIT систем
        // ------------------------------
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);
        robot.init();

        AimerSystem aimer = new AimerSystem(robot.guideServo);
        ElevatorSystem elevator = new ElevatorSystem(robot.elevatorServo);
        IntakeSystem intake = new IntakeSystem(robot);
        ShooterSystem shooter = new ShooterSystem(robot, hardwareMap);

        telemetry.addLine("Готово к старту");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ================================
        // 1) Поднять Aimer в дальнюю позицию
        // ================================
        aimer.goUp(); // позиция 0.69
        sleep(300);

        // ================================
        // 2) Запустить шутер
        // ================================
        shooter.toggle();   // enable = true

        // ================================
        // 3) Ждем 2 секунды разгона RPM
        // ================================
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTime < 2000) {
            shooter.updateShooter(true); // true → RPM = 6200
            telemetry.addData("RPM L", shooter.getLeftRPM());
            telemetry.addData("RPM R", shooter.getRightRPM());
            telemetry.update();
        }

        // ================================
        // ФУНКЦИЯ СТРЕЛЬБЫ = поднять + опустить Elevator
        // ================================
        Runnable shoot = () -> {
            elevator.goToTop();
            sleep(350);   // время подачи мяча
            elevator.goToBottom();
            sleep(250);
        };

        // ================================
        // 🎯 ВЫСТРЕЛ #1
        // ================================
        shoot.run();

        // ================================
        // 7) Не выключаем мотор — RPM продолжают держаться
        //    Просто повторяем updateShooter()
        // ================================
        long t1 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t1 < 150) {
            shooter.updateShooter(true);
        }

        // ================================
        // 8) Intake 1.5 секунды
        // ================================
        long intakeTime1 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - intakeTime1 < 1500) {
            intake.update(true, false);  // intake ON
            shooter.updateShooter(true); // держим RPM
        }
        intake.update(false, false); // OFF

        // ================================
        // 🎯 ВЫСТРЕЛ #2
        // ================================
        shoot.run();

        // ================================
        // 11) Intake 1.5 секунды
        // ================================
        long intakeTime2 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - intakeTime2 < 1500) {
            intake.update(true, false);
            shooter.updateShooter(true);
        }
        intake.update(false, false);

        // ================================
        // 🎯 ВЫСТРЕЛ #3
        // ================================
        shoot.run();

        // ================================
        // 14) Выключить мотор шутера
        // ================================
        shooter.stop();
        sleep(200);

        telemetry.addLine("Автоном завершён");
        telemetry.update();
    }
}
