package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class ElevatorSystem {

    private final Servo elevatorServo;
    private final double MIN_POS = 0.40;// начальная позиция
    private final double MAX_POS = 0.680; // верхняя позиция

    public ElevatorSystem(Servo servo) {
        this.elevatorServo = servo;
        elevatorServo.setPosition(MIN_POS); // стартовая позиция
    }

    public void goToBottom() {
        elevatorServo.setPosition(MIN_POS);
    }


    public void goToTop() {
        elevatorServo.setPosition(MAX_POS);
    }

    public void manualControl(double delta) {
        double pos = elevatorServo.getPosition() + delta;
        pos = clamp(pos, MIN_POS, MAX_POS);
        elevatorServo.setPosition(pos);
    }

    public void stop() {
        // Для серво нет активного остановки — просто держит текущую позицию
    }

    public double getPosition() {
        return elevatorServo.getPosition();
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}