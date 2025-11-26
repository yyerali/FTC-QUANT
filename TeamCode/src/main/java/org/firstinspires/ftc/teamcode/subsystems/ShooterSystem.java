package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class ShooterSystem {

    private final RobotHardware robot;
    private final VoltageSensor batterySensor;

    private boolean enabled = false;

    // RPM values for far and close shots
    private static final double RPM_UP = 6200;   // far
    private static final double RPM_DOWN = 3750; // close

    private static final double TICKS_PER_REV = 28;  // REV HD Hex
    private static final double NOMINAL_VOLTAGE = 12.0;

    // PIDF coefficients
    private double kP = 23.0;
    private double kI = 0.0;
    private double kD = 0.2;
    private double kF = 0.0003;

    // Ramp parameters
    private static final double MAX_RPM_STEP = 700; // max RPM increase per update

    private double currentTargetRPM = 0; // current RPM target for ramping

    public ShooterSystem(RobotHardware robot, HardwareMap hw) {
        this.robot = robot;

        VoltageSensor tempSensor = null;
        for (VoltageSensor sensor : hw.getAll(VoltageSensor.class)) {
            tempSensor = sensor;
            break;
        }
        this.batterySensor = tempSensor;

        initMotor(robot.leftShooter, true);
        initMotor(robot.rightShooter, false);
    }

    private void initMotor(DcMotorEx motor, boolean reversed) {
        if (motor == null) return;

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(reversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        PIDFCoefficients coeffs = new PIDFCoefficients(kP, kI, kD, kF);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
    }

    public void toggle() {
        enabled = !enabled;
        if (!enabled) stop();
    }

    // Call this periodically (e.g., in loop()) to update shooter
    public void updateShooter(boolean aimerUp) {
        if (!enabled) return;

        double targetRPM = aimerUp ? RPM_UP : RPM_DOWN;

        // Apply ramping
        if (currentTargetRPM < targetRPM) {
            currentTargetRPM += MAX_RPM_STEP;
            if (currentTargetRPM > targetRPM) currentTargetRPM = targetRPM;
        } else if (currentTargetRPM > targetRPM) {
            currentTargetRPM -= MAX_RPM_STEP;
            if (currentTargetRPM < targetRPM) currentTargetRPM = targetRPM;
        }

        setRPM(currentTargetRPM);
    }

    private void setRPM(double rpm) {
        double ticksPerSec = rpm * TICKS_PER_REV / 60.0;

        double voltage = batterySensor.getVoltage();
        if (voltage < 1) voltage = NOMINAL_VOLTAGE;
        double scaledF = kF;
        if (voltage < NOMINAL_VOLTAGE) scaledF = kF * (NOMINAL_VOLTAGE / voltage);

        PIDFCoefficients coeffs = new PIDFCoefficients(kP, kI, kD, scaledF);

        if (robot.leftShooter != null) {
            robot.leftShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
            robot.leftShooter.setVelocity(ticksPerSec);
        }

        if (robot.rightShooter != null) {
            robot.rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
            robot.rightShooter.setVelocity(ticksPerSec);
        }
    }

    public void stop() {
        enabled = false;
        currentTargetRPM = 0;
        if (robot.leftShooter != null) robot.leftShooter.setPower(0);
        if (robot.rightShooter != null) robot.rightShooter.setPower(0);
    }

    public boolean isShooting() {
        return enabled;
    }

    public double getLeftRPM() {
        if (robot.leftShooter == null) return 0;
        return robot.leftShooter.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    public double getRightRPM() {
        if (robot.rightShooter == null) return 0;
        return robot.rightShooter.getVelocity() * 60.0 / TICKS_PER_REV;
    }
}
