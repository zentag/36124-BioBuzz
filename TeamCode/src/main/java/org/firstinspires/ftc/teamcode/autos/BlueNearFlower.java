package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Near Flower - BLUE")
public class BlueNearFlower extends MainAuto {
    public BlueNearFlower() {
        robotSide = Robot.RobotSide.BLUE;
        startPosition = StartPosition.NEAR_FLOWER;
    }

}
