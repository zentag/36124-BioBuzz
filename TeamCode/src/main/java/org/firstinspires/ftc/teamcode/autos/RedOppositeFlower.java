package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Opposite Flower - RED")
public class RedOppositeFlower extends MainAuto {
    public RedOppositeFlower() {
        robotSide = Robot.RobotSide.RED;
        startPosition = StartPosition.OPPOSITE_FLOWER;
    }

}
