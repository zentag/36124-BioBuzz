package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.DoubleSupplier;

public class Drivetrain {
    private final Follower follower;
    private final Telemetry telemetry;
    private final Field field;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry, Field field) {
        this.telemetry = telemetry;
        this.field = field;
        follower = Constants.create(hardwareMap);
    }

    public void update() {
        follower.update();
    }

    public Follower getFollower() {
        return follower;
    }

    public void resetHeading() {
        follower.setPose(follower.pose().withHeading(0));
        follower.manual(0, 0, 0);
    }

    public void driveFieldCentric(double forward, double strafe, double omega) {
        // Pedro Pathing driving
        DrivePowers powers = ManualDrive.fieldCentric(
                forward, strafe, omega,
                follower.pose().heading()
        );

        ManualDrive.driveOrHold(follower, powers);
        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

    }

    public Command driveToPoint(Pose target) {
        return sequential(
                repeat(lazy(() -> driveToBestSafePoint(target)), field.safePoints.length)
                        .until(() -> field.isPathClear(straightPath(follower.pose(), target))),
                lazy(() -> {
                    Path finalPath = straightPath(follower.pose(), target);
                    if (!field.isPathClear(finalPath)) return Command.NOOP;
                    return follow(follower, finalPath);
                })
        );
    }

    public Path straightPath(Pose from, Pose to) {
        return line(from, to).linear(from, to);
    }

    public Command driveToBestSafePoint(Pose target) {
        Pose here = follower.pose();

        Path bestPath = null;
        double bestDistance = Double.MAX_VALUE;

        Pose nearestPoint = null;
        double nearestDistance = Double.MAX_VALUE;

        for (Pose point : field.safePoints) {
            double fromRobot = Math.hypot(point.x() - here.x(), point.y() - here.y());
            if (fromRobot < 3) continue;

            if (fromRobot < nearestDistance) {
                nearestPoint = point;
                nearestDistance = fromRobot;
            }

            Path proposedPath = straightPath(here, point);
            if (!field.isPathClear(proposedPath)) continue;
            double toTarget = Math.hypot(point.x() - target.x(), point.y() - target.y());
            if (toTarget < bestDistance) {
                bestPath = proposedPath;
                bestDistance = toTarget;
            }
        }

        if (bestPath == null && nearestPoint != null) bestPath = straightPath(here, nearestPoint);
        if (bestPath == null) return Command.NOOP;
        return follow(follower, bestPath);
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void driveInDirection(double direction) {
        DrivePowers powers = ManualDrive.headingLock(follower, Constants.foresightConfig.headingFeedback.get(), new DrivePowers(1, 0, 0), direction);
        follower.manual(
                powers
        );
    }

    public Command driveFieldCentricCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier omega) {
        return Command.build().setExecute(() -> driveFieldCentric(
                        forward.getAsDouble(),
                        strafe.getAsDouble(),
                        omega.getAsDouble()
                ))
                .requiring(follower)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setPriority(5);
    }

    public Command driveInDirectionCommand(double direction) {
        return Command.build().setExecute(() -> {
                    driveInDirection(direction);
                }).setEnd((c) -> follower.manual(0, 0, 0))
                .requiring(follower)
                .setPriority(10);
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Command followPath(Path path) {
        return follow(follower, path);
    }
}
