package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.RectF;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Robot;

public class Field {
    public static final float FIELD_SIZE = 141.5F;
    public static final float TILE = 24;
    public static final float EDGE_TILE = 22.75F;
    public static final float FLOWER_SIZE = 6;
    public static final float ROBOT_DIAGONAL = 26;
    public static final float OBSTACLE_CLEARANCE = ROBOT_DIAGONAL / 2;

    public Pose[] safePoints;
    public RectF[] obstacles;

    private static RectF square(float centerX, float centerY, float size) {
        float half = size / 2;
        return new RectF(centerX - half, centerY - half, centerX + half, centerY + half);
    }

    public Field(Robot.RobotSide robotSide) {
        PoseFactory p = PoseFactory.degrees();

        if (robotSide == Robot.RobotSide.RED) {
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
}
