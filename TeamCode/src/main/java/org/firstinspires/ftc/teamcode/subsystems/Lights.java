package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class Lights {
    private final GoBildaPrismDriver prism;
    private final PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.GREEN);
    private boolean showingFull = false;

    public Lights(HardwareMap hardwareMap) {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
    }

    public void update(boolean full) {
        if (full != showingFull) {
            solid.setPrimaryColor(full ? Color.RED : Color.GREEN);
            prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_0);
            showingFull = full;
        }
    }
}
