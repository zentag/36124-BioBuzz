package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerArm {
    private final Servo leftArmServo;
    private final Servo rightArmServo;

    public enum State {
        STOWED,
        DEPLOYED
    }

    public FlowerArm(HardwareMap hardwareMap) {
        leftArmServo = hardwareMap.get(Servo.class, "left_arm_servo");
        rightArmServo = hardwareMap.get(Servo.class, "right_arm_servo");

        /*
         * The arm servos face each other, so reverse the left one so the same position
         * moves both sides of the arm the same way.
         */
        leftArmServo.setDirection(Servo.Direction.REVERSE);
    }

    public Command setState(State state) {
        return Command.build().setStart(() -> {
            double position = 0;
            if (state == State.DEPLOYED) position = 0.3;

            leftArmServo.setPosition(position);
            rightArmServo.setPosition(position);

        }).setDone(() -> true).requiring(this);
    }
}
