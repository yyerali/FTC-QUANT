package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class AimerSystem {

    private final Servo servo;

    private final double DOWN_POS = 1.0;  // ближний
    private final double UP_POS   = 0.69;  // дальний

    public AimerSystem(Servo servo) {
        this.servo = servo;
        servo.setPosition(DOWN_POS);
    }

    public void goUp() {
        servo.setPosition(UP_POS);
    }

    public void goDown() {
        servo.setPosition(DOWN_POS);
    }

    public boolean isUp() {
        return Math.abs(servo.getPosition() - UP_POS) < 0.05;
    }

    public void toggle() {
        if (isUp()) {
            goDown();
        } else {
            goUp();
        }
    }

    public double getPosition() {
        return servo.getPosition();
    }
}

