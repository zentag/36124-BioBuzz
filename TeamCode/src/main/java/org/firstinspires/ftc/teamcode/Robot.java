package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.DoubleSupplier;

public class Robot {
    private Follower follower;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    private HuskyLens huskyLens;
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


    public enum RobotSide {
        RED,
        BLUE
    }

    private final Telemetry telemetry;
    private final RobotSide robotSide;


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
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

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
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the left intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    public void loop() {
        follower.update();
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

    public void intake(double intakePower) {
        /*
         * Here we set our intake motor and servos to their intake power. The order of operations
         * here is important though. The gamepad triggers define the starting point for the intake
         * power variable in each loop of our code, but inside our launch function we also sometimes
         * change the intake power. So we need to give our launch function a chance to modify the
         * variable before we write it to our motor and servos.
         */
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
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
            intake(.5);
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
        }).requiring(windmillServo, launcher, intake);
    }

    public Command intakeCommand(DoubleSupplier intakePower) {
        return Command.build().setExecute(() -> intake(intakePower.getAsDouble())).setEnd((c) -> {
            intake.setPower(0);
            leftIntakeServo.setPower(0);
            rightIntakeServo.setPower(0);
        }).requiring(intake, leftIntakeServo, rightIntakeServo);
    }

    public void driveInDirection(double direction) {
        DrivePowers powers = ManualDrive.headingLock(follower, Constants.foresightConfig.headingFeedback.get(), new DrivePowers(0, 0, 0), direction);
        follower.manual(
                powers
        );
    }

    public Command driveFieldCentricCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier omega) {
        return Command.build().setExecute(() -> driveFieldCentric(
                forward.getAsDouble(),
                strafe.getAsDouble(),
                omega.getAsDouble()
        )).requiring(follower);
    }

    public Command driveInDirectionCommand(double direction) {
        return Command.build().setExecute(() -> {
            driveInDirection(direction);
        }).requiring(follower);
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Command followPath(Path path) {
        return follow(follower, path);
    }

    public double getBiggestPollenDirection() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks.length == 0) return follower.pose().heading();
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
        return thetaRadians + follower.pose().heading();
    }

}
