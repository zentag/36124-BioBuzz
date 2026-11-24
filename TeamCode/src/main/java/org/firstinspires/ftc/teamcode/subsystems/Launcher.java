package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {
    private final Telemetry telemetry;
    private final Intake intake;

    private final DcMotorEx launcher;
    private final CRServo windmillServo;

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

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry, Intake intake) {
        this.telemetry = telemetry;
        this.intake = intake;

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");

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

        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
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
// TODO: make this finish at some point
    public Command launchCommand() {
        return Command.build().setExecute(this::launch).setEnd((c) -> {
                    windmillServo.setPower(0);
                    intake.setPower(0);
                    launcher.setVelocity(0);
                }).requiring(this, intake)
                .setPriority(10);
    }
}
