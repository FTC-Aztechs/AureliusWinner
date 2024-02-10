package com.example.newmeepmeeper;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class newMeepMeeper {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);


        //***************************************************************************************
        //TODO: CHOOSE RANDOMIZATION HERE
        int Randomization = 3;

        //***************************************************************************************
        //TODO: Declare Poses here

        //Todo: BlueShort
        Pose2d blueShortStartPos = new Pose2d(15,61.5,Math.toRadians(-90));//0,0,0

        Pose2d blueShortPurple1Pos = new Pose2d(14, 33, Math.toRadians(0));  //27,0,-90
        Pose2d blueShortPurple2Pos = new Pose2d(16, 32, Math.toRadians(-90));  //37,12,-90
        Pose2d blueShortPurple3Pos = new Pose2d(8.5, 33 , Math.toRadians(-180)); //27,19,-90


        Pose2d blueShortTagPos = new Pose2d(36,45, Math.toRadians(0));

        Vector2d blueShortYellow1Pos = new Vector2d(51.5, 42);  //27,37,-90
        Vector2d blueShortYellow2Pos = new Vector2d(51.5, 36);   //26,37,-90
        Vector2d blueShortYellow3Pos = new Vector2d(51.5,27.5);    //33,37,-90

        Vector2d blueShortParkPos = new Vector2d(47.5, 54.5);  //7, 37

        //Todo: RedShort
        Pose2d redShortStartPos = new Pose2d(15,-61.5,Math.toRadians(90));//0,0,0

        Pose2d redShortPurple3Pos = new Pose2d(14, -33, Math.toRadians(0)); //27,19,-90
        Pose2d redShortPurple2Pos = new Pose2d(16, -32, Math.toRadians(90));  //37,12,-90
        Pose2d redShortPurple1Pos = new Pose2d(8.5, -33, Math.toRadians(180));  //27,0,-90


        Pose2d redShortTagPos = new Pose2d(36,-45, Math.toRadians(0));

        Vector2d redShortYellow3Pos = new Vector2d(51.5, -42);  //27,37,-90
        Vector2d redShortYellow2Pos = new Vector2d(51.5, -36);   //26,37,-90
        Vector2d redShortYellow1Pos = new Vector2d(51.5,-27.5);    //33,37,-90

        Vector2d redShortParkPos = new Vector2d(45, -54.5);  //7, 37

        //Todo: BlueLong
        Pose2d blueLongStartPos = new Pose2d(-39,61.25,Math.toRadians(-90));//0,0,0

        Pose2d blueLongPurple1Pos = new Pose2d(-34, 34.5, Math.toRadians(0));  //27,0,-90
        Pose2d blueLongPurple2Pos = new Pose2d(-31, 34, Math.toRadians(-90));  //37,12,-90
        Pose2d blueLongPurple3Pos = new Pose2d(-39, 34.5 , Math.toRadians(-180)); //27,19,-90


        Vector2d blueLongBeforeGatePos1 = new Vector2d(-38,11.5);//50,2
        Vector2d blueLongBeforeGatePos2 = new Vector2d(-50, 11.5);//50,-14
        Vector2d blueLongBeforeGatePos3 = new Vector2d(-34, 11.5);//50,-19
        Vector2d blueLongAfterGateTagPos = new Vector2d(15.25, 11.5);//50,51.25
        Vector2d blueLongAfterGatePos = new Vector2d(32, 11.5);//50,68


        Vector2d blueLongYellow1Pos = new Vector2d(51.5, 42);  //27,37,-90
        Vector2d blueLongYellow2Pos = new Vector2d(51.5, 36);   //26,37,-90
        Pose2d blueLongYellow3Pos = new Pose2d(51.5,27.5, Math.toRadians(0));    //33,37,-90


        Vector2d blueLongParkPos = new Vector2d(47.5, 11.5);//50, 82

        //Todo:RedLong
        Pose2d redLongStartPos = new Pose2d(-39,-61.25,Math.toRadians(90));//0,0,0

        Pose2d redLongPurple3Pos = new Pose2d(-34, -34.5, Math.toRadians(0));  //27,0,-90
        Pose2d redLongPurple2Pos = new Pose2d(-31,- 34, Math.toRadians(90));  //37,12,-90
        Pose2d redLongPurple1Pos = new Pose2d(-39, -34.5 , Math.toRadians(180)); //27,19,-90

        Vector2d redLongBeforeGatePos3 = new Vector2d(-38,-11.5);//50,2
        Vector2d redLongBeforeGatePos2 = new Vector2d(-50,-11.5);//50,-14
        Vector2d redLongBeforeGatePos1 = new Vector2d(-34,-11.5);//50,-19
        Vector2d redLongAfterGateTagPos = new Vector2d(15.25, -11.5);//50,51.25
        Vector2d redLongAfterGatePos = new Vector2d(32, -11.5);//50,68

        Vector2d redLongYellow3Pos = new Vector2d(51.5, -42);  //27,37,-90
        Vector2d redLongYellow2Pos = new Vector2d(51.5, -36);   //26,37,-90
        Pose2d redLongYellow1Pos = new Pose2d(51.5,-27.5, Math.toRadians(0));    //33,37,-90


        Vector2d redLongParkPos = new Vector2d(47.5, -11.5);//50, 82


        //***************************************************************************************
        RoadRunnerBotEntity BlueShort = new DefaultBotBuilder(meepMeep)
                //TODO: This is BLUE_SHORT
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        switch (Randomization){

            case 1:
                BlueShort.runAction(BlueShort.getDrive().actionBuilder(blueShortStartPos)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(blueShortPurple1Pos, Math.toRadians(-90))
                        .strafeTo(new Vector2d(10,38.5))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(blueShortTagPos, Math.toRadians(-90))
                        .strafeTo(blueShortYellow1Pos)
                        .strafeTo(blueShortParkPos)
                        .build());
                break;

            case 2:
                BlueShort.runAction(BlueShort.getDrive().actionBuilder(blueShortStartPos)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(blueShortPurple2Pos, Math.toRadians(-90))
                        .lineToY(44.5)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(blueShortTagPos, Math.toRadians(-90))
                        .strafeTo(blueShortYellow1Pos)
                        .strafeTo(blueShortYellow2Pos)
                        .strafeTo(blueShortParkPos)
                        .build());
                break;

            case 3:
                BlueShort.runAction(BlueShort.getDrive().actionBuilder(blueShortStartPos)
                        .setTangent(Math.toRadians(-80))
                        .splineToLinearHeading(blueShortPurple3Pos, Math.toRadians(-130))
                        .lineToX(14)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(blueShortTagPos, Math.toRadians(-90))
                        .strafeTo(blueShortYellow1Pos)
                        .strafeTo(blueShortYellow3Pos)
                        .strafeTo(blueShortParkPos)
                        .build());
                break;
        }

        //***************************************************************************************
        RoadRunnerBotEntity BlueLong = new DefaultBotBuilder(meepMeep)
                //TODO: This is BLUE_LONG
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        switch (Randomization){

            case 1:
                BlueLong.runAction(BlueLong.getDrive().actionBuilder(blueLongStartPos)
                        .setTangent(Math.toRadians(-135))
                        .splineToLinearHeading(blueLongPurple1Pos, Math.toRadians(-30))
                        .setReversed(false)
                        .lineToX(-38)
                        .strafeTo(blueLongBeforeGatePos1)
                        .strafeTo(blueLongAfterGateTagPos)
                        .splineToLinearHeading(blueLongYellow3Pos,Math.toRadians(0))
                        .strafeTo(blueLongYellow1Pos)
                        .strafeTo(blueLongParkPos)
                        .build());
                break;

            case 2:
                BlueLong.runAction(BlueLong.getDrive().actionBuilder(blueLongStartPos)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(blueLongPurple2Pos, Math.toRadians(-70))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-50,45, Math.toRadians(0)),Math.toRadians(180))
                        .strafeTo(blueLongBeforeGatePos2)
                        .strafeTo(blueLongAfterGateTagPos)
                        .splineToLinearHeading(blueLongYellow3Pos,Math.toRadians(0))
                        .strafeTo(blueLongYellow2Pos)
                        .strafeTo(blueLongParkPos)
                        .build());
                break;

            case 3:
                BlueLong.runAction(BlueLong.getDrive().actionBuilder(blueLongStartPos)
                        .setTangent(Math.toRadians(-110))
                        .splineToLinearHeading(blueLongPurple3Pos, Math.toRadians(-90))
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(-37,34.5))
                        .strafeTo(blueLongBeforeGatePos3)
                        .turn(Math.toRadians(180))
                        .strafeTo(blueLongAfterGateTagPos)
                        .splineToLinearHeading(blueLongYellow3Pos,Math.toRadians(0))
                        .strafeTo(blueLongParkPos)
                        .build());
                break;
        }


        //***************************************************************************************
        RoadRunnerBotEntity RedShort = new DefaultBotBuilder(meepMeep)
                //TODO: This is RED_SHORT
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        switch (Randomization){

            case 1:
                RedShort.runAction(RedShort.getDrive().actionBuilder(redShortStartPos)
                        .setTangent(Math.toRadians(80))
                        .splineToLinearHeading(redShortPurple1Pos, Math.toRadians(130))
                        .lineToX(14)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(redShortTagPos, Math.toRadians(90))
                        .strafeTo(redShortYellow3Pos)
                        .strafeTo(redShortYellow1Pos)
                        .strafeTo(redShortParkPos)
                        .build());
                break;

            case 2:
                RedShort.runAction(RedShort.getDrive().actionBuilder(redShortStartPos)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redShortPurple2Pos, Math.toRadians(90))
                        .lineToY(-44.5)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(redShortTagPos, Math.toRadians(90))
                        .strafeTo(redShortYellow3Pos)
                        .strafeTo(redShortYellow2Pos)
                        .strafeTo(redShortParkPos)
                        .build());
                break;

            case 3:
                RedShort.runAction(RedShort.getDrive().actionBuilder(redShortStartPos)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redShortPurple3Pos, Math.toRadians(90))
                        .strafeTo(new Vector2d(10,-38.5))
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(redShortTagPos, Math.toRadians(90))
                        .strafeTo(redShortYellow3Pos)
                        .strafeTo(redShortParkPos)
                        .build());
                break;
        }


        //***************************************************************************************
        RoadRunnerBotEntity RedLong = new DefaultBotBuilder(meepMeep)
                //TODO: This is RED_LONG
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        switch (Randomization){

            case 1:
                RedLong.runAction(RedLong.getDrive().actionBuilder(redLongStartPos)
                        .setTangent(Math.toRadians(110))
                        .splineToLinearHeading(redLongPurple1Pos, Math.toRadians(90))
                        .setTangent(Math.toRadians(-90))
                        .strafeTo(new Vector2d(-37,-34.5))
                        .strafeTo(redLongBeforeGatePos1)
                        .turn(Math.toRadians(-180))
                        .strafeTo(redLongAfterGateTagPos)
                        .splineToLinearHeading(redLongYellow1Pos,Math.toRadians(0))
                        .strafeTo(redLongParkPos)
                        .build());
                break;

            case 2:
                RedLong.runAction(RedLong.getDrive().actionBuilder(redLongStartPos)
                        .setTangent(Math.toRadians(30))
                        .splineToLinearHeading(redLongPurple2Pos, Math.toRadians(180))
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(-50,-45, Math.toRadians(0)),Math.toRadians(-180))
                        .strafeTo(redLongBeforeGatePos2)
                        .strafeTo(redLongAfterGateTagPos)
                        .splineToLinearHeading(redLongYellow1Pos,Math.toRadians(0))
                        .strafeTo(redLongYellow2Pos)
                        .strafeTo(redLongParkPos)
                        .build());
                break;

            case 3:
                RedLong.runAction(RedLong.getDrive().actionBuilder(redLongStartPos)
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(redLongPurple3Pos, Math.toRadians(30))
                        .setReversed(false)
                        .lineToX(-38)
                        .strafeTo(redLongBeforeGatePos3)
                        .strafeTo(redLongAfterGateTagPos)
                        .splineToLinearHeading(redLongYellow1Pos,Math.toRadians(0))
                        .strafeTo(redLongYellow3Pos)
                        .strafeTo(redLongParkPos)
                        .build());
                break;
        }


        //***************************************************************************************
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                //TODO: Set background and robots here
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(BlueShort)
                .addEntity(BlueLong)
                .addEntity(RedShort)
                .addEntity(RedLong)
                .start();
    }
}