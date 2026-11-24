package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TESTING Teleop - BLUE",group="StarterBot")
public class TestTeleop extends MainTeleop{
    @Override
    public void init(){
        super.init();
        robot.intake.shouldCountPollen = false;
    }
}
