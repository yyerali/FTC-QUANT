//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//public class save {
//
//    private static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(10, 0, 0, 12);
//    public DcMotorEx leftShooter, rightShooter;
//    public Servo guideServo, elevatorServo;
//    public DcMotor intakeMotor;
//    public DcMotorEx LF, RF, LB, RB;
//    public GoBildaPinpointDriver pinpoint;
//
//    private final HardwareMap hw;
//    private final Telemetry telemetry;
//
//    public d(HardwareMap hardwareMap, Telemetry telemetry) {
//        this.hw = hardwareMap;
//        this.telemetry = telemetry;
//    }
//
//    public void init() {
//
//        elevatorServo = get(Servo.class, "elevatorServo");
//        if (elevatorServo != null) {
//            elevatorServo.setDirection(Servo.Direction.REVERSE);
//            elevatorServo.setPosition(0.5);
//        }
//
//        leftShooter = get(DcMotorEx.class, "shooterLeft");
//        rightShooter = get(DcMotorEx.class, "shooterRight");
//
//        if (leftShooter != null) {
//            leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
//            leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            leftShooter.setVelocityPIDFCoefficients(SHOOTER_PIDF.p, SHOOTER_PIDF.i, SHOOTER_PIDF.d, SHOOTER_PIDF.f);
//        }
//
//        if (rightShooter != null) {
//            rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
//            rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            rightShooter.setVelocityPIDFCoefficients(SHOOTER_PIDF.p, SHOOTER_PIDF.i, SHOOTER_PIDF.d, SHOOTER_PIDF.f);
//        }
//
//        // ВАЖНО: подняли AIM выше чтобы докидывал (исправление)
//        guideServo = get(Servo.class, "guideServo");
//        if (guideServo != null) guideServo.setPosition(0.73);
//
//        intakeMotor = get(DcMotor.class, "intakeMotor");
//        if (intakeMotor != null) intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        LF = get(DcMotorEx.class, "LF");
//        RF = get(DcMotorEx.class, "RF");
//        LB = get(DcMotorEx.class, "LB");
//        RB = get(DcMotorEx.class, "RB");
//
//        if (LF != null) LF.setDirection(DcMotorSimple.Direction.FORWARD);
//        if (RF != null) RF.setDirection(DcMotorSimple.Direction.REVERSE);
//        if (LB != null) LB.setDirection(DcMotorSimple.Direction.FORWARD);
//        if (RB != null) RB.setDirection(DcMotorSimple.Direction.REVERSE);
//    }
//
//    private <T> T get(Class<T> type, String name) {
//        try {
//            return hw.get(type, name);
//        } catch (Exception e) {
//            telemetry.addLine("❌ Not found: " + name);
//            return null;
//        }
//    }
//}


//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//public class ShooterSystem {
//
//    private final RobotHardware robot;
//    private boolean enabled = false;
//
//    public ShooterSystem(RobotHardware robot) {
//        this.robot = robot;
//
//        if (robot.leftShooter != null) {
//            robot.leftShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            robot.leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            robot.leftShooter.setDirection(DcMotor.Direction.REVERSE);
//        }
//
//        if (robot.rightShooter != null) {
//            robot.rightShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            robot.rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            robot.rightShooter.setDirection(DcMotor.Direction.FORWARD);
//        }
//    }
//
//    // ------------ NEW: toggle ------------
//    public void toggle() {
//        enabled = !enabled;
//        setPower(enabled ? 1.0 : 0.0);
//    }
//
//    // ------------ NEW: isShooting ------------
//    public boolean isShooting() {
//        return enabled;
//    }
//
//    // ------------ setPower ------------
//    public void setPower(double power) {
//        if (robot.leftShooter != null) robot.leftShooter.setPower(power);
//        if (robot.rightShooter != null) robot.rightShooter.setPower(power);
//        enabled = power > 0;
//    }
//
//    // ------------ stop ------------
//    public void stop() {
//        setPower(0);
//    }
//}
