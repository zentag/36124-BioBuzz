package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.graphics.RectF;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class Robot {
    private Follower follower;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private Servo leftArmServo = null;
    private Servo rightArmServo = null;
    private CRServo windmillServo = null;

    private HuskyLens huskyLens;
    private DistanceSensor intakeSensorLeft;
    private DistanceSensor intakeSensorRight;

    GoBildaPrismDriver prism;

    PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.GREEN);

    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. We use the min velocity
     * in the launch() function to only run the windmill servo when the motor is spinning fast
     * enough to make a successful throw.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    public double targetDirection = 0;
    public double targetingStartDirection = 0;
    public int targetingLoopsDone = 0;

    public boolean shouldCountPollen = true;
    public int pollenInRobot = 0;

    private boolean intakeBlocked = false;
    private int peakPollen = 0;
    private final ElapsedTime clearTimer = new ElapsedTime();

    private boolean showingFull = false;

    public enum RobotSide {
        RED,
        BLUE
    }

    private final Telemetry telemetry;
    private final RobotSide robotSide;

    public Pose[] safePoints;
    public RectF[] obstacles;

    public static final float FIELD_SIZE = 141.5F;
    public static final float TILE = 24;
    public static final float EDGE_TILE = 22.75F;
    public static final float FLOWER_SIZE = 6;
    public static final float ROBOT_DIAGONAL = 26;
    public static final float OBSTACLE_CLEARANCE = ROBOT_DIAGONAL / 2;

    private static RectF square(float centerX, float centerY, float size) {
        float half = size / 2;
        return new RectF(centerX - half, centerY - half, centerX + half, centerY + half);
    }

    public enum HiveTipped {
        TOWARDS_FLOWER,
        AWAY_FROM_FLOWER
    }

    public HiveTipped hiveDirection = HiveTipped.TOWARDS_FLOWER;


    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide) {
        Scheduler.reset();
        this.telemetry = telemetry;
        this.robotSide = robotSide;
        follower = Constants.create(hardwareMap);

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftArmServo = hardwareMap.get(Servo.class, "left_arm_servo");
        rightArmServo = hardwareMap.get(Servo.class, "right_arm_servo");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        intakeSensorLeft = hardwareMap.get(DistanceSensor.class, "intake_sensor_left");
        intakeSensorRight = hardwareMap.get(DistanceSensor.class, "intake_sensor_right");

        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        launcher.setZeroPowerBehavior(BRAKE);
        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        windmillServo.setPower(0);

        /*
         * The arm servos face each other, so reverse the left one so the same position
         * moves both sides of the arm the same way.
         */
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        PoseFactory p = PoseFactory.degrees();

        if (robotSide == RobotSide.RED) {
            double center = FIELD_SIZE / 2;
            p = p.mirrorAroundPoint(center, center);
        }

        safePoints = new Pose[]{
                p.of(82, 22, 0),
                p.of(120, 124, 0),
                p.of(115, 33, 0)
        };


        // flower 1 is against the left wall (x = 0), centered two tiles up.
        // each next flower is the one before it rotated 90 degrees around the field center,
        // which puts them against the bottom, right and top walls.
        // this layout is the same after a 180 degree turn, so red and blue don't need different values
        float wallOffset = FLOWER_SIZE / 2;   // flower center is half a flower away from its wall
        float twoTiles = EDGE_TILE + TILE;    // the seam two tiles from the wall: 46.75

        obstacles = new RectF[]{
                // middle of the field (RectF wants left, top, right, bottom with top < bottom)
                new RectF(42.75F, 51.25F, 98.75F, 90.25F),

                // flowers
                square(wallOffset, twoTiles, FLOWER_SIZE),                           // left wall
                square(FIELD_SIZE - twoTiles, wallOffset, FLOWER_SIZE),              // bottom wall
                square(FIELD_SIZE - wallOffset, FIELD_SIZE - twoTiles, FLOWER_SIZE), // right wall
                square(twoTiles, FIELD_SIZE - wallOffset, FLOWER_SIZE),              // top wall
        };

        // grow every obstacle so the robot's corners can't clip it (a negative inset grows the rectangle)
        for (RectF obstacle : obstacles) {
            obstacle.inset(-OBSTACLE_CLEARANCE, -OBSTACLE_CLEARANCE);
        }

        updateHiveDirection();

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    public void loop() {
        follower.update();
        if (shouldCountPollen) checkForPollen();
        boolean full = pollenInRobot > 3;
        if (full != showingFull) {
            solid.setPrimaryColor(full ? Color.RED : Color.GREEN);
            prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_0);
            showingFull = full;
        }
    }

    public void checkForPollen() {
        if (intake.getPowerFloat()) return;

        double NEAR_CM = 14;
        double EMPTY_CM = 22;

        double left = intakeSensorLeft.getDistance(DistanceUnit.CM);
        double right = intakeSensorRight.getDistance(DistanceUnit.CM);

        telemetry.addData("left", left);
        telemetry.addData("right", right);

        int rawPollen;
        if (left < NEAR_CM && right < NEAR_CM) {
            rawPollen = 2;          // each sensor has a pollen right in front of it
        } else if (left < EMPTY_CM - 2 || right < EMPTY_CM - 2) {
            rawPollen = 1;          // something is in there
        } else {
            rawPollen = 0;
        }

        if (rawPollen > 0) {
            intakeBlocked = true;
            peakPollen = Math.max(peakPollen, rawPollen);
            clearTimer.reset();
        } else if (intakeBlocked && clearTimer.milliseconds() > 150) {
            // pass is over: count it once, using the most we saw during it
            pollenInRobot += peakPollen;
            intakeBlocked = false;
            peakPollen = 0;
            if (pollenInRobot > 3) schedule(preventIntaking());
        }
    }


    public Command preventIntaking() {
        return Command.build().setStart(() -> {
                    intake.setPower(0);
                }).setDone(() -> pollenInRobot < 4)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setPriority(7)
                .requiring(intake);
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

    void launch() {
        /*
         * Calling gamepad1.right_bumper returns a boolean which will be true if the bumper is
         * held down, and false if it is not. Notably, this will continue to be true for every
         * cycle of our code that the driver holds down that bumper.
         * The first step of our launch() function is checking to see if the user is currently
         * holding down the right gamepad. If they are, then we want to start spinning up the launcher.
         * Otherwise, we start spinning the launcher down.
         */
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

        /*
         * Here we ask if the driver is currently pressing the right bumper, AND the launcher is
         * spinning fast enough to make a successful shot. If it is, then we will turn on the
         * windmill servo to start feeding the elements into the launcher motor. We also
         * add some power to the intake power. This can sometimes help dislodge stuck elements from
         * inside the hopper.
         */
        telemetry.addData("launcher velo", launcher.getVelocity());
        if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);
            intake.setPower(.5);
        } else {
            windmillServo.setPower(0);
        }
        telemetry.addData("servo power", windmillServo.getPower());
    }

    public Command launchCommand() {
        return Command.build().setExecute(this::launch).setEnd((c) -> {
                    windmillServo.setPower(0);
                    intake.setPower(0);
                    launcher.setVelocity(0);
                }).requiring(windmillServo, launcher, intake)
                .setPriority(10);
    }

    public Command intakeCommand(DoubleSupplier intakePower) {
        return Command.build().setExecute(() -> {
                    intake.setPower(intakePower.getAsDouble());
                }).setEnd((c) -> {
                    intake.setPower(0);
                }).requiring(intake)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setPriority(5);
    }

    public enum FlowerArmState {
        STOWED,
        DEPLOYED
    }

    public Command setFlowerArmState(FlowerArmState state) {
        return Command.build().setStart(() -> {
            double position = 0;
            if (state == FlowerArmState.DEPLOYED) position = 0.3;

            leftArmServo.setPosition(position);
            rightArmServo.setPosition(position);

        }).setDone(() -> true).requiring(leftArmServo, rightArmServo);
    }

    public Command driveToPoint(Pose target) {
        return sequential(
                repeat(lazy(() -> driveToBestSafePoint(target)), safePoints.length)
                        .until(() -> isPathClear(straightPath(follower.pose(), target))),
                lazy(() -> {
                    Path finalPath = straightPath(follower.pose(), target);
                    if (!isPathClear(finalPath)) return Command.NOOP;
                    return follow(follower, finalPath);
                })
        );
    }

    private Path straightPath(Pose from, Pose to) {
        return line(from, to).linear(from, to);
    }

    public Command driveToBestSafePoint(Pose target) {
        Pose here = follower.pose();

        Path bestPath = null;
        double bestDistance = Double.MAX_VALUE;

        Pose nearestPoint = null;
        double nearestDistance = Double.MAX_VALUE;

        for (Pose point : safePoints) {
            double fromRobot = Math.hypot(point.x() - here.x(), point.y() - here.y());
            if (fromRobot < 3) continue;

            if (fromRobot < nearestDistance) {
                nearestPoint = point;
                nearestDistance = fromRobot;
            }

            Path proposedPath = straightPath(here, point);
            if (!isPathClear(proposedPath)) continue;
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

    public boolean isPathClear(Path path) {
        // the area the robot's center can be in without its corners hitting a wall
        RectF fullUsableField = new RectF(0, 0, FIELD_SIZE, FIELD_SIZE);
        fullUsableField.inset(OBSTACLE_CLEARANCE, OBSTACLE_CLEARANCE);

        int samples = 50;
        for (int i = 0; i <= samples; i++) {
            Pose pose = path.get(i / (double) samples);
            float x = (float) pose.x();
            float y = (float) pose.y();

            // the start and end are known good spots (like the shooting position against the wall),
            // so only check the walls in the middle of the path
            boolean nearEnds = i < samples / 5 || i > samples - samples / 5;
            if (!nearEnds && !fullUsableField.contains(x, y)) return false;

            for (RectF r : obstacles) {
                if (r.contains(x, y)) return false;
            }
        }
        return true;
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

    public Command intakeOnePollen() {
        double previousPollenInRobot = pollenInRobot;
        return sequential(
                rotateUntilPollenDetected(),
                race(parallel(
                                driveInDirectionCommand(targetDirection),
                                intakeCommand(() -> 1))
                                .until(() -> pollenInRobot > previousPollenInRobot)
                                .then(race(intakeCommand(() -> 1), waitMs(500))),
                        waitMs(2000)));
    }

    public Command rotateUntilPollenDetected() {
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
                    OptionalDouble direction = getBiggestPollenDirection();
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

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Command followPath(Path path) {
        return follow(follower, path);
    }


    private void updateHiveDirection() {
        // TODO: implement
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
        return OptionalDouble.of(thetaRadians + follower.pose().heading());
    }

}
