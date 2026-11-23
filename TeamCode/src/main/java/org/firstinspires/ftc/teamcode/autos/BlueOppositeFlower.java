package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Opposite Flower - BLUE")
public class BlueOppositeFlower extends MainAuto {
    public BlueOppositeFlower() {
        robotSide = Robot.RobotSide.BLUE;
        startPosition = StartPosition.OPPOSITE_FLOWER;
    }

}
