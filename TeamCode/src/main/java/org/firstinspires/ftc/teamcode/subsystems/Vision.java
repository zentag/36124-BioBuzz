package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.OptionalDouble;

public class Vision {
    private final Telemetry telemetry;
    private final Drivetrain drivetrain;
    private final HuskyLens huskyLens;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry, Drivetrain drivetrain) {
        this.telemetry = telemetry;
        this.drivetrain = drivetrain;
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
    }

    public OptionalDouble getBiggestPollenDirection() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks.length == 0) return OptionalDouble.empty();
        int bestScore = 0;
        HuskyLens.Block primaryBlock = blocks[0];
        for (int i = 0; i < blocks.length; i++) {
            int area = blocks[i].width * blocks[i].height;
            int score = area;
            if (score > bestScore) {
                primaryBlock = blocks[i];
                bestScore = score;
            }
            /*
             * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
             * - blocks[i].width and blocks[i].height   (size of box, in pixels)
             * - blocks[i].left and blocks[i].top       (edges of box)
             * - blocks[i].x and blocks[i].y            (center location)
             * - blocks[i].id                           (Color ID)
             *
             * These values have Java type int (integer).
             */
        }
        telemetry.addData("block", primaryBlock);
        int screenWidth = 240;
        int distanceFromCenter = primaryBlock.y - (screenWidth / 2);
        double halfOfCameraFOVRadians = Math.toRadians(60 / 2);
        double thetaRadians = Math.atan(
                (2 * distanceFromCenter * Math.tan(halfOfCameraFOVRadians))
                        /
                        screenWidth
        );
        double thetaDegrees = Math.toDegrees(thetaRadians);
        telemetry.addData("relative angle", thetaDegrees);
        return OptionalDouble.of(thetaRadians + drivetrain.getPose().heading());
    }
}
