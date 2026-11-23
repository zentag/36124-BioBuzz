package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Near Flower - RED")
public class RedNearFlower extends MainAuto {
    public RedNearFlower() {
        robotSide = Robot.RobotSide.RED;
        startPosition = StartPosition.NEAR_FLOWER;
    }

}
