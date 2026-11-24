package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="Main Teleop - RED",group="StarterBot")
public class RedTeleop extends MainTeleop {
    public RedTeleop() {
        robotSide = Robot.RobotSide.RED;
    }
}
