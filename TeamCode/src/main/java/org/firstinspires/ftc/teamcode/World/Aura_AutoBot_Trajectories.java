package org.firstinspires.ftc.teamcode.World;

import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_FOR_OUTTAKE;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_FOR_STACK_INTAKE;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_FOR_YELLOW_DROP;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_RETURN_TO_INTAKE;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Roadrunner.roadrunnerbasics.MecanumDrive;

public class Aura_AutoBot_Trajectories {
}

class AutoBot_BlueLong {
    Aura_AutoBot_Actions Aurelius;
    MecanumDrive blueLong;

    //Auto Blue Long
    //Poses
    Pose2d blueCycleLongStartPos = new Pose2d(-39,61.25,Math.toRadians(-90));//0,0,0

    Pose2d blueCycleLongPurple1Pos = new Pose2d(-32.5, 34.5, Math.toRadians(0));  //27,0,-90
    Pose2d blueCycleLongPurple2Pos = new Pose2d(-39, 34, Math.toRadians(-90));  //37,12,-90
    Pose2d blueCycleLongPurple3Pos = new Pose2d(-47, 41.5, Math.toRadians(-90)); //27,19,-90

    Vector2d blueCycleLongBeforeGatePos = new Vector2d(-38, 10);//50,-19
    Vector2d blueCycleLongAfterGateTagPos = new Vector2d(36, 10);//50,51.25

    Vector2d blueCycleLongPreStackPos = new Vector2d(-55.5,17);
    Vector2d blueCycleLongStackPos = new Vector2d(-59,17);
    Vector2d blueCycleLongPostStackPos = new Vector2d(-59,21.5);

    Pose2d blueCycleLongYellow1Pos = new Pose2d(51.5, 41.75, Math.toRadians(0));  //27,37,-90
    Pose2d blueCycleLongYellow2Pos = new Pose2d(51, 37.25, Math.toRadians(0));   //26,37,-90
    Pose2d blueCycleLongYellow3Pos = new Pose2d(51,28.5, Math.toRadians(0));    //33,37,-90

    Vector2d blueCycleLongWhite1Pos = new Vector2d(51, 35);  //27,37,-90
    Vector2d blueCycleLongWhite2Pos = new Vector2d(51, 34);   //26,37,-90
    Vector2d blueCycleLongWhite3Pos = new Vector2d(51,28);

    Vector2d blueCycleLongParkPos = new Vector2d(45, 11.5);//50, 82

    //Trajectories
    // Purple Trajectories
    private Action dropOffPurpleAtPos1;
    private Action dropOffPurpleAtPos2;
    private Action dropOffPurpleAtPos3;

    // Yellow Trajectories
    private Action dropOffYellowAtPos1;
    private Action dropOffYellowAtPos2;
    private Action dropOffYellowAtPos3;

    public AutoBot_BlueLong(Aura_AutoBot_Actions aurelius, MecanumDrive MecanumDrive) {
        Aurelius = aurelius;
        blueLong = MecanumDrive;
    }
    void buildAutoBlueLongTrajectories(){
        buildAutoBlueLongPurpleTrajectories();
    }
    void buildAutoBlueLongPurpleTrajectories() {
        dropOffPurpleAtPos1 = blueLong.actionBuilder(blueCycleLongStartPos)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(blueCycleLongPurple1Pos, Math.toRadians(-30))
                .stopAndAdd(ejectPurple)
                .build();

        dropOffPurpleAtPos2 = blueLong.actionBuilder(blueCycleLongStartPos)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(blueCycleLongPurple2Pos, Math.toRadians(-70))
                .stopAndAdd(ejectPurple)
                .build();

        dropOffPurpleAtPos3 = blueLong.actionBuilder(blueCycleLongStartPos)
                .setTangent(Math.toRadians(-110))
                .splineToLinearHeading(blueCycleLongPurple3Pos, Math.toRadians(-90))
                .stopAndAdd(ejectPurple)
                .build();
    }

    void buildYellowTrajectories()
    {
        dropOffYellowAtPos1 = mecanumDrive.actionBuilder(blueCycleLongPurple1Pos)
                .waitSeconds(5)
                .setReversed(false)
                .setTangent(Math.toRadians(0))
                .lineToX(-38)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(blueCycleLongPreStackPos)
                .stopAndAdd(getReadyForIntake)
                .afterDisp(0,deployStackIntake) // Make sure to turn on bottom roller
                .strafeTo(blueCycleLongStackPos)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(blueCycleLongPostStackPos)
                .stopAndAdd(intakeFromStack) // Make sure to flip box and lock fingers
                .waitSeconds(AUTO_WAIT_FOR_STACK_INTAKE)
                .stopAndAdd(securePixels)
                .strafeTo(blueCycleLongBeforeGatePos)
                .afterDisp(0, retractStackIntake)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(blueCycleLongAfterGateTagPos)
                .afterDisp(50, getReadyForOutTake)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(new Vector2d(36,36))
                .waitSeconds(1)
                .stopAndAdd(updatePosFromAprilTagEyeball)
                .splineToLinearHeading(blueCycleLongYellow1Pos,Math.toRadians(0))
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
                .stopAndAdd(depositYellow)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .strafeTo(blueCycleLongWhite1Pos)
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
                .stopAndAdd(depositWhite)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .afterDisp(0,getReadyForIntake)
                .strafeTo(blueCycleLongParkPos)
                .waitSeconds(AUTO_WAIT_RETURN_TO_INTAKE)
                .build();

        dropOffYellowAtPos2 = mecanumDrive.actionBuilder(blueCycleLongPurple2Pos)
                .waitSeconds(5)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-50,45, Math.toRadians(0)),Math.toRadians(180))
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(blueCycleLongPreStackPos)
                .stopAndAdd(getReadyForIntake)
                .afterDisp(0, deployStackIntake)
                .strafeTo(blueCycleLongStackPos)
                .strafeTo(blueCycleLongPostStackPos)
                .stopAndAdd(intakeFromStack) // Make sure to flip box and lock fingers
                .waitSeconds(AUTO_WAIT_FOR_STACK_INTAKE)
                .stopAndAdd(securePixels)
                .strafeTo(blueCycleLongBeforeGatePos)
                .afterDisp(0, retractStackIntake)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(blueCycleLongAfterGateTagPos)
                .afterDisp(50, getReadyForOutTake)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(new Vector2d(36,36))
                .waitSeconds(1)
                .stopAndAdd(updatePosFromAprilTagEyeball)
                .splineToLinearHeading(blueCycleLongYellow2Pos,Math.toRadians(0))
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
                .stopAndAdd(depositYellow)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .strafeTo(blueCycleLongWhite2Pos)
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
                .stopAndAdd(depositWhite)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .afterDisp(0,getReadyForIntake)
                .strafeTo(blueCycleLongParkPos)
                .waitSeconds(AUTO_WAIT_RETURN_TO_INTAKE)
                .build();

        dropOffYellowAtPos3 = mecanumDrive.actionBuilder(blueCycleLongPurple3Pos)
                .waitSeconds(5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57.5, 48,Math.toRadians(0)), Math.toRadians(-180))
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(blueCycleLongPreStackPos)
                .stopAndAdd(getReadyForIntake)
                .afterDisp(0, deployStackIntake)
                .strafeTo(blueCycleLongStackPos)
                .strafeTo(blueCycleLongPostStackPos)
                .stopAndAdd(intakeFromStack) // Make sure to flip box and lock fingers
                .waitSeconds(AUTO_WAIT_FOR_STACK_INTAKE)
                .stopAndAdd(securePixels)
                .strafeTo(blueCycleLongBeforeGatePos)
                .afterDisp(0, retractStackIntake)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(blueCycleLongAfterGateTagPos)
                .afterDisp(50, getReadyForOutTake)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(new Vector2d(36,36))
                .waitSeconds(1)
                .stopAndAdd(updatePosFromAprilTagEyeball)
                .splineToLinearHeading(blueCycleLongYellow3Pos,Math.toRadians(0))
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
                .stopAndAdd(depositYellow)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .strafeTo(blueCycleLongWhite2Pos)
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
                .stopAndAdd(depositWhite)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .afterDisp(0,getReadyForIntake)
                .strafeTo(blueCycleLongParkPos)
                .waitSeconds(AUTO_WAIT_RETURN_TO_INTAKE)
                .build();
    }