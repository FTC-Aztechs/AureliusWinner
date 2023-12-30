package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class AuraPose2d {
    public double x;
    public double y;
    public double heading;
    Pose2d myPose2D;
    Vector2d myVec2D;

    AuraPose2d(double inX, double inY, double inHeading) { x = inX; y = inY; heading = inHeading;
        myPose2D = new Pose2d(x, y, Math.toRadians(heading));
        myVec2D = new Vector2d(x, y);
    }
    public Pose2d pose2d() { return myPose2D; }
    public Vector2d vec2d() { return myVec2D; }
}
