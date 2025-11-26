package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    // --- PIDF для шутеров ---
    private static final PIDFCoefficients SHOOTER_PIDF =
            new PIDFCoefficients(10, 0, 0, 12);

    // --- Оборудование ---
    public DcMotorEx leftShooter;
    public DcMotorEx rightShooter;
    public Servo guideServo;       // направитель
    public DcMotor intakeMotor;
    public Servo elevatorServo;    // серво элеватора

    public DcMotorEx LF;
    public DcMotorEx RF;
    public DcMotorEx LB;
    public DcMotorEx RB;
    public GoBildaPinpointDriver pinpoint;
    private final HardwareMap hw;
    private final Telemetry telemetry;

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hw = hardwareMap;
        this.telemetry = telemetry;
    }

    public void init() {

        // ----------------------
        // ЭЛЕВАТОР
        // ----------------------
        elevatorServo = getDevice(hw, telemetry, Servo.class, "elevatorServo");
        if (elevatorServo != null) {
            elevatorServo.setDirection(Servo.Direction.REVERSE);
            elevatorServo.setPosition(0.5);
        }

        // ----------------------
        // ШУТЕРЫ
        // ----------------------
        leftShooter = getDevice(hw, telemetry, DcMotorEx.class, "shooterLeft");
        rightShooter = getDevice(hw, telemetry, DcMotorEx.class, "shooterRight");

        if (leftShooter != null) {
            leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
            leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftShooter.setVelocityPIDFCoefficients(
                    SHOOTER_PIDF.p, SHOOTER_PIDF.i, SHOOTER_PIDF.d, SHOOTER_PIDF.f);
        }

        if (rightShooter != null) {
            rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
            rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightShooter.setVelocityPIDFCoefficients(
                    SHOOTER_PIDF.p, SHOOTER_PIDF.i, SHOOTER_PIDF.d, SHOOTER_PIDF.f);
        }

        // ----------------------
        // НАПРАВИТЕЛЬ (aim servo)
        // ----------------------
        guideServo = getDevice(hw, telemetry, Servo.class, "guideServo");
        if (guideServo != null) guideServo.setPosition(0.5);

        // ----------------------
        // ИНТЕЙК
        // ----------------------
        intakeMotor = getDevice(hw, telemetry, DcMotor.class, "intakeMotor");
        if (intakeMotor != null) {
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ----------------------
        // ШАССИ MECANUM (ДОБАВЛЕНО!)
        // ----------------------
        LF  = getDevice(hw, telemetry, DcMotorEx.class, "LF");
        RF = getDevice(hw, telemetry, DcMotorEx.class, "RF");
        LB  = getDevice(hw, telemetry, DcMotorEx.class, "LB");
        RB  = getDevice(hw, telemetry, DcMotorEx.class, "RB");

        if (LF != null)
            LF.setDirection(DcMotorSimple.Direction.FORWARD);

        if (RF != null)
            RF.setDirection(DcMotorSimple.Direction.REVERSE);

        if (LB != null)
            LB.setDirection(DcMotorSimple.Direction.FORWARD);

        if (RB != null)
            RB.setDirection(DcMotorSimple.Direction.REVERSE);
        try {
            pinpoint = hw.get(GoBildaPinpointDriver.class, "pinpoint");
            telemetry.addLine("✅ PinPoint найден");
        } catch (Exception e) {
            telemetry.addLine("❌ PinPoint не найден");
            pinpoint = null;
        }
    }

    // ----------------------------------------------------
    // Безопасное получение устройства
    // ----------------------------------------------------
    private <T> T getDevice(HardwareMap hardwareMap, Telemetry telemetry,
                            Class<T> deviceClass, String name) {
        try {
            return hardwareMap.get(deviceClass, name);
        } catch (Exception e) {
            telemetry.addLine("❌ Устройство не найдено: " + name);
            return null;
        }
    }
}