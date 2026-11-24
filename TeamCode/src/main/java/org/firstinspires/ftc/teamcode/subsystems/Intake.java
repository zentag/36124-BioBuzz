package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.DoubleSupplier;

public class Intake {
    private final Telemetry telemetry;

    private final DcMotor motor;
    private final DistanceSensor intakeSensorLeft;
    private final DistanceSensor intakeSensorRight;
    private final ColorRangeSensor intakeColorLeft;
    private final ColorRangeSensor intakeColorRight;

    // pollen yellow is matched by hue (the "which color" part of HSV), which stays about the same
    // no matter how close or bright the pollen is. pure yellow is 60. tune these from telemetry
    public static final float YELLOW_MIN_HUE = 40;
    public static final float YELLOW_MAX_HUE = 90;
    // below this saturation the reading is grey/white (nothing in front of the sensor), not yellow
    public static final float YELLOW_MIN_SATURATION = 0.4F;
    // boosts the raw readings so dim colors still give a usable hue
    public static final float COLOR_SENSOR_GAIN = 4;

    public boolean shouldCountPollen = true;
    public int pollenInRobot = 0;

    private boolean intakeBlocked = false;
    private int peakPollen = 0;
    private final ElapsedTime clearTimer = new ElapsedTime();

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        motor = hardwareMap.get(DcMotor.class, "intake");
        intakeSensorLeft = hardwareMap.get(DistanceSensor.class, "intake_sensor_left");
        intakeSensorRight = hardwareMap.get(DistanceSensor.class, "intake_sensor_right");
        intakeColorLeft = hardwareMap.get(ColorRangeSensor.class,"intake_color_left");
        intakeColorRight = hardwareMap.get(ColorRangeSensor.class,"intake_color_right");
        intakeColorLeft.setGain(COLOR_SENSOR_GAIN);
        intakeColorRight.setGain(COLOR_SENSOR_GAIN);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        motor.setZeroPowerBehavior(BRAKE);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void reportColors() {
        float[] leftHsv = readHsv(intakeColorLeft);
        float[] rightHsv = readHsv(intakeColorRight);
        telemetry.addData("left color", "hue %.0f  sat %.2f  yellow? %b", leftHsv[0], leftHsv[1], isPollenYellow(leftHsv));
        telemetry.addData("right color", "hue %.0f  sat %.2f  yellow? %b", rightHsv[0], rightHsv[1], isPollenYellow(rightHsv));
    }

    // the sensor's current color as {hue 0-360, saturation 0-1, value 0-1}
    private float[] readHsv(ColorRangeSensor sensor) {
        float[] hsv = new float[3];
        android.graphics.Color.colorToHSV(sensor.getNormalizedColors().toColor(), hsv);
        return hsv;
    }

    private boolean isPollenYellow(float[] hsv) {
        return hsv[0] >= YELLOW_MIN_HUE && hsv[0] <= YELLOW_MAX_HUE && hsv[1] >= YELLOW_MIN_SATURATION;
    }

    public boolean isPollenYellow(ColorRangeSensor sensor) {
        return isPollenYellow(readHsv(sensor));
    }

    public void checkForPollen() {
        if (motor.getPowerFloat()) return;

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
                    motor.setPower(0);
                }).setDone(() -> pollenInRobot < 4)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setPriority(7)
                .requiring(this);
    }

    public Command intakeCommand(DoubleSupplier intakePower) {
        return Command.build().setExecute(() -> {
                    motor.setPower(intakePower.getAsDouble());
                }).setEnd((c) -> {
                    motor.setPower(0);
                }).requiring(this)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setPriority(5);
    }
}
