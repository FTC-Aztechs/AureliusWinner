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

        Pose2d blueCycleLongStartPos = new Pose2d(-39,61.25,Math.toRadians(-90));//0,0,0

        Pose2d blueCycleLongPurple1Pos = new Pose2d(-32.5, 34.5, Math.toRadians(0));  //27,0,-90
        Pose2d blueCycleLongPurple2Pos = new Pose2d(-39, 34, Math.toRadians(-90));  //37,12,-90
        Pose2d blueCycleLongPurple3Pos = new Pose2d(-47, 41.5, Math.toRadians(-90)); //27,19,-90

        Vector2d blueCycleLongBeforeGatePos = new Vector2d(-38, 10);//50,-19
        Vector2d blueCycleLongAfterGateTagPos = new Vector2d(36, 10);//50,51.25


        Vector2d blueCycleLongPreStackPos = new Vector2d(-55.5,17);
        Vector2d blueCycleLongStackPos = new Vector2d(-59,17);

        Vector2d blueCycleLongStackPos2 = new Vector2d(-61,20);

        Vector2d blueCycleLongPostStackPos = new Vector2d(-59,21.5);

        Vector2d blueCycleLongPostStackPos2 = new Vector2d(-61,29.5);


        Pose2d blueCycleLongYellow1Pos = new Pose2d(51.5, 41.75, Math.toRadians(0));  //27,37,-90
        Pose2d blueCycleLongYellow2Pos = new Pose2d(51, 37.25, Math.toRadians(0));   //26,37,-90
        Pose2d blueCycleLongYellow3Pos = new Pose2d(51,28.5, Math.toRadians(0));    //33,37,-90

        Vector2d blueCycleLongWhite1Pos = new Vector2d(51, 35);  //27,37,-90
        Vector2d blueCycleLongWhite2Pos = new Vector2d(51, 34);   //26,37,-90
        Vector2d blueCycleLongWhite3Pos = new Vector2d(51,28);

        Vector2d blueCycleLongParkPos = new Vector2d(45, 11.5);//50, 82

        Vector2d blueCycleLongCycleBoardPos = new Vector2d(45,13);

        //red long
        Pose2d redStartPos = new Pose2d(-36,-61.5,Math.toRadians(90));//0,0,0

        Pose2d redPurple1Pos = new Pose2d(-37, -34.5 , Math.toRadians(180)); //27,19,-90
        Pose2d redPurple2Pos = new Pose2d(-31,- 33, Math.toRadians(90));  //37,12,-90
        Pose2d redPurple3Pos = new Pose2d(-34, -34.5, Math.toRadians(0));  //27,0,-90

        Vector2d redBeforeGatePos3 = new Vector2d(-38,-11.5);//50,2
        Vector2d redBeforeGatePos2 = new Vector2d(-50,-11.5);//50,-14
        Vector2d redBeforeGatePos1 = new Vector2d(-34,-11.5);//50,-19
        Vector2d redAfterGateTagPos = new Vector2d(15.25, -11.5);//50,51.25
        Vector2d redAfterGatePos = new Vector2d(32, -11.5);//50,68

        Vector2d redYellow3Pos = new Vector2d(50, -42);  //27,37,-90
        Vector2d redYellow2Pos = new Vector2d(50, -35.5);   //26,37,-90
        Pose2d redYellow1Pos = new Pose2d(50,-28.5,Math.toRadians(0));    //33,37,-90


        Vector2d redParkPos = new Vector2d(47.5, -11.5);//50, 82

        //blue long
        Pose2d blueStartPos = new Pose2d(-39,62.25,Math.toRadians(-90));//0,0,0

        Pose2d bluePurple3Pos = new Pose2d(-39, 34.5 , Math.toRadians(-180)); //27,19,-90
        Pose2d bluePurple2Pos = new Pose2d(-31, 33, Math.toRadians(-90));  //37,12,-90
        Pose2d bluePurple1Pos = new Pose2d(-34, 34.5, Math.toRadians(0));  //27,0,-90

        Vector2d blueBeforeGatePos1 = new Vector2d(-38,11.5);//50,2
        Vector2d blueBeforeGatePos2 = new Vector2d(-50, 11.5);//50,-14
        Vector2d blueBeforeGatePos3 = new Vector2d(-34, 11.5);//50,-19
        Vector2d blueAfterGateTagPos = new Vector2d(15.25, 11.5);//50,51.25
        Vector2d blueAfterGatePos = new Vector2d(32, 11.5);//50,68

        Vector2d blueYellow1Pos = new Vector2d(50, 42);  //27,37,-90
        Vector2d blueYellow2Pos = new Vector2d(50, 35.5);   //26,37,-90
        Pose2d blueYellow3Pos = new Pose2d(49.5,28.5,Math.toRadians(0));    //33,37,-90


        Vector2d blueParkPos = new Vector2d(47.5, 11.5);//50, 82

        //blue short
//        Pose2d blueStartPos = new Pose2d(15,61.5,Math.toRadians(-90));//0,0,0
//
//        Pose2d bluePurple3Pos = new Pose2d(10, 34.5 , Math.toRadians(-180)); //27,19,-90
//        Pose2d bluePurple2Pos = new Pose2d(17, 33, Math.toRadians(-90));  //37,12,-90
//        Pose2d bluePurple1Pos = new Pose2d(14, 34.5, Math.toRadians(0));  //27,0,-90
//
//        Pose2d blueYellow1Pos = new Pose2d(49.5, 42, Math.toRadians(0));  //27,37,-90
//        Vector2d blueYellow2Pos = new Vector2d(49.5, 35.5);   //26,37,-90
//        Vector2d blueYellow3Pos = new Vector2d(49.5,28.5);    //33,37,-90
//
//        Vector2d blueParkPos = new Vector2d(47.5, 54.5);  //7, 37

        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(bluePurple1Pos)
                                .setTangent(60)
                                .splineToConstantHeading(blueCycleLongPreStackPos, 60)
                                .splineToConstantHeading(blueCycleLongStackPos, 0)
                                .setTangent(-120)
                                .splineToConstantHeading(blueCycleLongPostStackPos, 0)
                                .build()
//                        drive.trajectorySequenceBuilder(redStartPos)
//                                .setTangent(Math.toRadians(110))
//                                .splineToLinearHeading(redPurple1Pos, Math.toRadians(90))
//                                .setTangent(Math.toRadians(-90))
//                                .lineToLinearHeading(new Pose2d(-34,-34.5, Math.toRadians(-180)))
//                                .strafeTo(redBeforeGatePos1)
//                                .turn(Math.toRadians(-180))
////                                        .stopAndAdd(rectifyHeadingError)
//                                .strafeTo(redAfterGateTagPos)
////                                        .stopAndAdd(updateAfterGatePos)
//                                .splineToLinearHeading(redYellow1Pos,Math.toRadians(0))
////                                        .stopAndAdd(rectifyHeadingError)
//
//                                .strafeTo(redParkPos)
//                                .build()
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
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(blueStartPos)
                                        .setTangent(Math.toRadians(-90))
                                        .splineToLinearHeading(bluePurple2Pos, Math.toRadians(-30))
                                        .waitSeconds(1)
                                        .setTangent(Math.toRadians(90))
                                        .splineToLinearHeading(new Pose2d(-50,45, Math.toRadians(0)),Math.toRadians(180))
                                        .strafeTo(blueBeforeGatePos2)
                                        .strafeTo(blueAfterGateTagPos)
                                        //.stopAndAdd(updateAfterGatePos)
                                        .splineToLinearHeading(blueYellow3Pos,Math.toRadians(0))
                                        .strafeTo(blueYellow2Pos)
                                        .strafeTo(blueParkPos)
                                        .build()//

                        // drive.trajectorySequenceBuilder(new Pose2d(0 + rStartPos.getX(), 0 + rStartPos.getY(), Math.toRadians(0) + rStartPos.getHeading()))
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