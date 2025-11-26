package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AimerSystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Robo-Centric TeleOp")
public class TeleOp extends OpMode {

    private RobotHardware robot;
    private Follower f;

    private ShooterSystem shooter;
    private ElevatorSystem elevator;
    private AimerSystem aimer;

    private boolean r2Last = false;
    private boolean xLast  = false;
    public static Pose start = new Pose(0,0,Math.toRadians(0));

    @Override
    public void init() {
        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(start);
        f.startTeleOpDrive(true);

        robot = new RobotHardware(hardwareMap, telemetry);
        robot.init();

        shooter = new ShooterSystem(robot, hardwareMap);
        elevator = new ElevatorSystem(robot.elevatorServo);
        aimer    = new AimerSystem(robot.guideServo);
    }

    @Override
    public void loop() {
        f.update();

        handleDriving();
        handleIntake();
        handleAimer();
        handleShooter();
        handleElevator();
        updateTelemetry();
    }

    private void handleDriving() {
        f.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
    }

    private void handleIntake() {
        if (robot.intakeMotor == null) return;

        if (gamepad1.left_trigger > 0.5)
            robot.intakeMotor.setPower(-1);
        else if (gamepad1.left_bumper)
            robot.intakeMotor.setPower(1);
        else
            robot.intakeMotor.setPower(0);
    }

    private void handleShooter() {
        boolean r2 = gamepad1.right_trigger > 0.5;

        // Toggle ON/OFF
        if (r2 && !r2Last) shooter.toggle();
        r2Last = r2;

        // Обновление скорости в зависимости от положения aimer
        shooter.updateShooter(aimer.isUp());
    }



    private void handleAimer() {
        boolean x = gamepad1.x;
        if (x && !xLast) aimer.toggle();
        xLast = x;
    }

    private void handleElevator() {
        if (robot.elevatorServo == null) return;

        if (gamepad1.right_bumper) elevator.goToTop();
        else                       elevator.goToBottom();
    }

    private void updateTelemetry() {
        telemetry.addData("Shooter", shooter.isShooting() ? "ON" : "OFF");
        telemetry.addData("Aimer", aimer.isUp() ? "UP" : "DOWN");
        telemetry.update();
    }
}
