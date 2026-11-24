package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Field;
import org.firstinspires.ftc.teamcode.subsystems.FlowerArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.OptionalDouble;

public class Robot {
    public final Field field;
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Launcher launcher;
    public final FlowerArm flowerArm;
    public final Vision vision;
    public final Lights lights;

    public double targetDirection = 0;
    public double targetingStartDirection = 0;
    public int targetingLoopsDone = 0;

    public enum RobotSide {
        RED,
        BLUE
    }

    private final Telemetry telemetry;
    private final RobotSide robotSide;

    public enum HiveTippedUp {
        TOWARDS_FLOWER,
        AWAY_FROM_FLOWER
    }

    public HiveTippedUp placeToShoot = HiveTippedUp.TOWARDS_FLOWER;


    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide) {
        Scheduler.reset();
        this.telemetry = telemetry;
        this.robotSide = robotSide;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        field = new Field(robotSide);
        drivetrain = new Drivetrain(hardwareMap, telemetry, field);
        intake = new Intake(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap, telemetry, intake);
        flowerArm = new FlowerArm(hardwareMap);
        vision = new Vision(hardwareMap, telemetry, drivetrain);
        lights = new Lights(hardwareMap);

        updateHiveDirection();

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    public void loop() {
        drivetrain.update();
        if (intake.shouldCountPollen) intake.checkForPollen();
        lights.update(intake.pollenInRobot > 3);
        intake.reportColors();
    }

    public Command intakeOnePollen() {
        double previousPollenInRobot = intake.pollenInRobot;
        return sequential(
                rotateUntilPollenDetected(),
                race(parallel(
                                drivetrain.driveInDirectionCommand(targetDirection),
                                intake.intakeCommand(() -> 1))
                                .until(() -> intake.pollenInRobot > previousPollenInRobot)
                                .then(race(intake.intakeCommand(() -> 1), waitMs(500))),
                        waitMs(2000)));
    }

    public Command rotateUntilPollenDetected() {
        Follower follower = drivetrain.getFollower();
        return Command.build()
                .setStart(() -> {
                    targetingStartDirection = follower.pose().heading();
                    targetingLoopsDone = 0;
                })
                .setExecute(() -> {
                    follower.manual(0, 0, .2);
                    telemetry.addData("yo", "rotating");
                    targetingLoopsDone += 1;
                }).setDone(() -> {
                    // if the camera sees a pollen, exit the command
                    OptionalDouble direction = vision.getBiggestPollenDirection();
                    if (direction.isPresent()) {
                        targetDirection = direction.getAsDouble();
                        return true;
                    }

                    // if we've done a full rotation, exit command
                    double currentHeading = follower.pose().heading();
                    if (Math.abs(targetingStartDirection - currentHeading) < .05 && targetingLoopsDone > 20) {
                        targetDirection = currentHeading;
                        return true;
                    }

                    // if neither end condition is met, we are not done
                    return false;
                }).setEnd((c) -> follower.manual(0, 0, 0))
                .requiring(follower)
                .setPriority(7);
    }


    private void updateHiveDirection() {
        // TODO: implement
    }

}
