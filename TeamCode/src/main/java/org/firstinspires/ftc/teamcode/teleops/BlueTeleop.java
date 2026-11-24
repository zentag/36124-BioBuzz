package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="Main Teleop - BLUE",group="StarterBot")
public class BlueTeleop extends MainTeleop {
    public BlueTeleop() {
        robotSide = Robot.RobotSide.BLUE;
    }
}
