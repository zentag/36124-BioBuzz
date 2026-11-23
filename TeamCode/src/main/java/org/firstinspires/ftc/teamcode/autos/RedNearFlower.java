package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Near Flower - RED")
public class RedNearFlower extends MainAuto {
    public Robot.RobotSide robotSide = Robot.RobotSide.RED;
    public StartPosition startPosition = StartPosition.NEAR_FLOWER;

}
