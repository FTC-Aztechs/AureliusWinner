package com.example.mymeepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MyMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d StartPos = new Pose2d(-36,62,Math.toRadians(-90));
        Pose2d rStartPos = new Pose2d(-36,-62,Math.toRadians(90));

        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                                .lineTo(new Vector2d(0,24))
                                .strafeTo(new Vector2d(24,24))
                                .lineTo(new Vector2d (24,0))
                                .strafeTo(new Vector2d(0,0))
                                .splineTo(new Vector2d(24,24),Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(0,0),Math.toRadians(0))
                                .build()
//                        //Blue Long Pos 3
//                        drive.trajectorySequenceBuilder(new Pose2d(0 + StartPos.getX(), 0 + StartPos.getY(), Math.toRadians(0) + StartPos.getHeading()))
//                                //purple
//                                .setReversed(true)
//                                .splineToLinearHeading(new Pose2d(2 + StartPos.getX(), -28 + StartPos.getY(), Math.toRadians(90) + StartPos.getHeading()), Math.toRadians(-90))
//                                //strafe to in front of gate
//                                .setReversed(false)
//                                .strafeTo(new Vector2d(2 + StartPos.getX(), -50  + StartPos.getY()))
//                                //strafe through gate
//                                .lineToLinearHeading(new Pose2d(50 + StartPos.getX(), -50  + StartPos.getY(), Math.toRadians(90) + StartPos.getHeading()))
//                                //spline to board and turn around
//                                .splineToLinearHeading(new Pose2d(82 + StartPos.getX(), -22 + StartPos.getY(), Math.toRadians(-90) + StartPos.getHeading()), Math.toRadians(90))
//                                //park
//                                .strafeTo(new Vector2d(82 + StartPos.getX(), -50 + StartPos.getY()))
//                                .build()
                );

        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        //Blue Long Pos 3
                        drive.trajectorySequenceBuilder(new Pose2d(0 + rStartPos.getX(), 0 + rStartPos.getY(), Math.toRadians(0) + rStartPos.getHeading()))
                                //purple
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(2 + rStartPos.getX(), 28 + rStartPos.getY(), Math.toRadians(-90) + rStartPos.getHeading()), Math.toRadians(90))
                                //strafe to in front of gate
                                .setReversed(false)
                                .strafeTo(new Vector2d(2 + rStartPos.getX(), 50  + rStartPos.getY()))
                                //strafe through gate
                                .lineToLinearHeading(new Pose2d(50 + rStartPos.getX(), 50  + rStartPos.getY(), Math.toRadians(-90) + rStartPos.getHeading()))
                                //spline to board and turn around
                                .splineToLinearHeading(new Pose2d(82 + rStartPos.getX(), 22 + rStartPos.getY(), Math.toRadians(90) + rStartPos.getHeading()), Math.toRadians(-90))
                                //park
                                .strafeTo(new Vector2d(82 + rStartPos.getX(), 50 + rStartPos.getY()))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myFirstBot)
                .addEntity(mySecondBot)
                .start();
    }
}