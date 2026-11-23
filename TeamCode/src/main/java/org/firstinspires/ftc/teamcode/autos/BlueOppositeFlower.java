package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Opposite Flower - BLUE")
public class BlueOppositeFlower extends MainAuto {
    public Robot.RobotSide robotSide = Robot.RobotSide.BLUE;
    public StartPosition startPosition = StartPosition.OPPOSITE_FLOWER;

}
