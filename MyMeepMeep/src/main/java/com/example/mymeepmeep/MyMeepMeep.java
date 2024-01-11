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

        //red short
        Pose2d redStartPos = new Pose2d(12,-61.5,Math.toRadians(90));//0,0,0

        Pose2d redPurple3Pos = new Pose2d(31, -34.5, Math.toRadians(180));//28,2,90
        Pose2d redPurple2Pos = new Pose2d(24, -24.5, Math.toRadians(180));//36,-12,90
        Pose2d redPurple1Pos = new Pose2d(12, -34.5, Math.toRadians(180));//28,-19,90

        Pose2d redYellow3Pos = new Pose2d(49, -34.5, Math.toRadians(180));//20,-37,90
        Pose2d redYellow2Pos = new Pose2d(49, -35.5, Math.toRadians(180));//28,-37,90
        Pose2d redYellow1Pos = new Pose2d(49,-28.5, Math.toRadians(180));//33,-37,90

        Vector2d redParkPos = new Vector2d(49, -54.5);//7,-37

        //blue short
        Pose2d blueStartPos = new Pose2d(12,61.5,Math.toRadians(-90));//0,0,0

        Pose2d bluePurple1Pos = new Pose2d(31, 34.5 , Math.toRadians(-180)); //27,19,-90
        Pose2d bluePurple2Pos = new Pose2d(24, 24.5 , Math.toRadians(-180));  //37,12,-90
        Pose2d bluePurple3Pos = new Pose2d(12, 34.5, Math.toRadians(-180));  //27,0,-90

        Pose2d blueYellow1Pos = new Pose2d(49, 34.5, Math.toRadians(-180));  //27,37,-90
        Pose2d blueYellow2Pos = new Pose2d(49, 35.5,Math.toRadians(-180));   //26,37,-90
        Pose2d blueYellow3Pos = new Pose2d(49,28.5 ,Math.toRadians(-180));    //33,37,-90

        Vector2d blueParkPos = new Vector2d(49, 54.5);  //7, 37

        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redStartPos)
                                .setTangent(Math.toRadians(80))
                                .splineToLinearHeading(redPurple1Pos, Math.toRadians(130))
                                .setReversed(true)
                                .splineToLinearHeading(redYellow1Pos, Math.toRadians(-90))
                                .strafeTo(redParkPos)
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
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(blueStartPos)
                                        .setTangent(Math.toRadians(-80))
                                        .splineToLinearHeading(bluePurple3Pos, Math.toRadians(-130))
                                        .setReversed(true)
                                        .splineToLinearHeading(blueYellow3Pos, Math.toRadians(90)).setReversed(true)
                                        .strafeTo(blueParkPos)
                                        .build()
//                        //Blue Long Pos 3
//                        drive.trajectorySequenceBuilder(new Pose2d(0 + rStartPos.getX(), 0 + rStartPos.getY(), Math.toRadians(0) + rStartPos.getHeading()))
//                                //purple
//                                .setReversed(true)
//                                .splineToLinearHeading(new Pose2d(2 + rStartPos.getX(), 28 + rStartPos.getY(), Math.toRadians(-90) + rStartPos.getHeading()), Math.toRadians(90))
//                                //strafe to in front of gate
//                                .setReversed(false)
//                                .strafeTo(new Vector2d(2 + rStartPos.getX(), 50  + rStartPos.getY()))
//                                //strafe through gate
//                                .lineToLinearHeading(new Pose2d(50 + rStartPos.getX(), 50  + rStartPos.getY(), Math.toRadians(-90) + rStartPos.getHeading()))
//                                //spline to board and turn around
//                                .splineToLinearHeading(new Pose2d(82 + rStartPos.getX(), 22 + rStartPos.getY(), Math.toRadians(90) + rStartPos.getHeading()), Math.toRadians(-90))
//                                //park
//                                .strafeTo(new Vector2d(82 + rStartPos.getX(), 50 + rStartPos.getY()))
//                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myFirstBot)
                .addEntity(mySecondBot)
                .start();
    }
}