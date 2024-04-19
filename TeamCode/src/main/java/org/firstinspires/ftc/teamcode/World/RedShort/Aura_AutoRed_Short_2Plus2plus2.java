/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.World.RedShort;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.ioState.STATE_1_RFI;
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.ioState.STATE_3_PS;
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.ioState.STATE_5_RFO_LOW;
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.ioState.STATE_6_PR_BOTH;
import static org.firstinspires.ftc.teamcode.AuraRobot.APRILTAG_TIMEOUT;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_ACCEL_CONSTRAINT_1;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_ACCEL_CONSTRAINT_2;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_FOR_OUTTAKE;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_FOR_PURPLE_DROP;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_FOR_STACK_INTAKE;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_FOR_YELLOW_DROP;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_RETURN_TO_INTAKE;
import static org.firstinspires.ftc.teamcode.AuraRobot.LEFT_FINGER_UNLOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.PURPLE_LOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.PURPLE_UNLOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.RIGHT_FINGER_UNLOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.Ramp_Down_Pos;
import static org.firstinspires.ftc.teamcode.AuraRobot.Ramp_Up_Pos;
import static org.firstinspires.ftc.teamcode.AuraRobot.left2LinkageOpen;
import static org.firstinspires.ftc.teamcode.AuraRobot.leftLinkageClose;
import static org.firstinspires.ftc.teamcode.AuraRobot.redShort_leftLinkageClose;
import static org.firstinspires.ftc.teamcode.AuraRobot.redShort_leftLinkageOpen;
import static org.firstinspires.ftc.teamcode.AuraRobot.redShort_rightLinkageClose;
import static org.firstinspires.ftc.teamcode.AuraRobot.redShort_rightLinkageOpen;
import static org.firstinspires.ftc.teamcode.AuraRobot.right2LinkageOpen;
import static org.firstinspires.ftc.teamcode.AuraRobot.rightLinkageClose;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.AuraHeadingEstimator;
import org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController;
import org.firstinspires.ftc.teamcode.AuraRobot;
import org.firstinspires.ftc.teamcode.Roadrunner.roadrunnerbasics.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Config
@Autonomous(name="RedShort_2+2+2", group="World: Red_Short")

public class Aura_AutoRed_Short_2Plus2plus2 extends LinearOpMode {

    //**** Roadrunner Pose2ds ****

    //Todo: switch to field coordinates:
    //to find x: add robotcentric Y pos to field centric start pos X
    //to find y: subtract robotcentric X pos to field centric start pos y
    //to find heading: add -90 degrees to field centric start pos heading


    // RObot Width = 15; Length = 15.5
    Pose2d redShortStartPos = new Pose2d(15,-61.5,Math.toRadians(90));//0,0,0

    Pose2d redShortPurple3Pos = new Pose2d(14, -33, Math.toRadians(0));  //27,0,-90
    Pose2d redShortPurple2Pos = new Pose2d(16, -32.5, Math.toRadians(90));  //37,12,-90
    Pose2d redShortPurple1Pos = new Pose2d(8, -33 , Math.toRadians(180)); //27,19,-90

    Pose2d redShortTagPos = new Pose2d(36,-45, Math.toRadians(0));
    Vector2d redShortCycleTagPos = new Vector2d(36, -36);//50,51.25

    Vector2d redShortYellow3Pos = new Vector2d(50.5, -41.5);  //27,37,-90
    Vector2d redShortYellow2Pos = new Vector2d(50.5, -35);   //26,37,-90
    Vector2d redShortYellow1Pos = new Vector2d(51.5,-28);    //33,37,-90

    Vector2d redShortWhite3Pos = new Vector2d(50.5, -42);  //27,37,-90
    Vector2d redShortWhite2Pos = new Vector2d(50.5, -34.5);   //26,37,-90
    Vector2d redShortWhite1Pos = new Vector2d(50.5,-28.5);    //33,37,-90

    Pose2d redShortBeforeGateCyclePos = new Pose2d(12,-57, Math.toRadians(0));

    Vector2d redShortEntryPos = new Vector2d(12,-58);

    Vector2d redShortReturnPos = new Vector2d(-40,-58);
    Vector2d redShortWingPos = new Vector2d(-60.5,-58);
    Vector2d redShortStackPos = new Vector2d(-60.5,-34);

    Vector2d redShortCycleLongBeforeGatePos = new Vector2d(-38, -10);//50,-19
    Vector2d redShortCycleLongAfterGatePos = new Vector2d(36, -10);//50,51.25

    Vector2d redShortCycleLongPreStackPos2 = new Vector2d(-62,-21);
    Vector2d redShortCycleLongPostStackPos2 = new Vector2d(-62,-12.5);

    Vector2d redShortParkPos = new Vector2d(45, -54.5);  //7, 37
    boolean bProceedToYellow = false;

    //Roadrunner field-centric coordinates quick guide brought to you by Lavanya

    //y+ robot drives from centerfield towards the blue side
    //y- robot drives from centerfield towards the red side
    //x- robot drives from centerfield towards stacks
    //x+ robot drives from centerfield towards backdrops

    //0° robot from centerfield faces backdrop
    //90° robot from centerfield faces blue
    //180° robot from centerfield faces stacks
    //-90° robot from centerfield faces red
    //tangent parameter in splines = changing angle changes the shape of the path

    //setTangent() = changes the direction in which the robot initially starts to drive
    //90 = to the left
    //180 = to the back
    //-90 = to the right
    //0 = forward

    //************

    public class PurpleDumper implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {
            Aurelius.PurpleDumper.setPosition(PURPLE_UNLOCK);
            return false;
        }
    }
    public Action ejectPurple = new PurpleDumper();

    public class YellowDropper implements Action {
        @Override
        public boolean run (TelemetryPacket tPkt) {
            MyIntakeOuttakeController.setTargetState(STATE_6_PR_BOTH);
            return false;
        }
    }
    public Action depositYellow = new YellowDropper();

    public class GotoOuttakeAction implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {
            MyIntakeOuttakeController.setTargetState(STATE_5_RFO_LOW);
            return false;
        }
    }
    public Action getReadyForOutTake = new GotoOuttakeAction();

    public class WhiteDumper implements Action {
        @Override
        public boolean run (TelemetryPacket tPkt) {
            MyIntakeOuttakeController.LeftFinger.setPosition(LEFT_FINGER_UNLOCK);
            MyIntakeOuttakeController.RightFinger.setPosition(RIGHT_FINGER_UNLOCK);
            return false;
        }
    }
    public Action dumpWhite = new WhiteDumper();

    public class GotoIntakeAction implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {
            MyIntakeOuttakeController.setTargetState(STATE_1_RFI);
            return false;
        }
    }
    public Action getReadyForIntake = new GotoIntakeAction();

    public class IntakeOuttakeControllerAction implements Action {
        public boolean run(TelemetryPacket tPkt) {
            MyIntakeOuttakeController.update();
            return bRunningTrajectory;
        }
    }
    public Action updateIOController = new IntakeOuttakeControllerAction();

    public class beginTrajectory implements Action {
        public boolean run(TelemetryPacket tPkt) {
            bRunningTrajectory=true;
            return false;
        }
    }
    public Action beginTrajectoryMarker = new beginTrajectory();

    public class endTrajectory implements Action {
        public boolean run(TelemetryPacket tPkt) {
            bRunningTrajectory=false;
            return false;
        }
    }
    public Action endTrajectoryMarker = new endTrajectory();

    public class backwallAprilTagControllerEyeball implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {
            updateposFromAprilTagEyeball();
            return false;
        }
    }

    public class backwallAprilTagControllerKemera implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {
            updateposFromAprilTagKemera();
            return false;
        }
    }


    public Action updatePosFromAprilTagEyeball = new backwallAprilTagControllerEyeball();
    public Action updatePosFromAprilTagKemera = new backwallAprilTagControllerKemera();


    public class initAprilTag implements Action {
        public boolean run(TelemetryPacket tPkt){
//            initAprilTagEyeball();
            initAprilTagKemera();
            return false;
        }
    }

    public Action initApril = new initAprilTag();


    public class lowerStackLinkage implements Action {
        public boolean run(TelemetryPacket tPkt){
            lowerStackIntake();
            return false;
        }
    }
    public Action deployStackIntake = new lowerStackLinkage();

    public class lower2StackLinkage implements Action {
        public boolean run(TelemetryPacket tPkt){
            lowerStackIntake2();
            return false;
        }
    }
    public Action deploy2StackIntake = new lower2StackLinkage();


    public class raiseStackLinkage implements Action {
        public boolean run(TelemetryPacket tPkt){
            raiseStackIntake();
            return false;
        }
    }
    public Action retractStackIntake = new raiseStackLinkage();

    public class stackIntake implements Action {
        public boolean run(TelemetryPacket tPkt){
            stackIntakePixels();
            return false;
        }
    }
    public Action intakeFromStack = new stackIntake();

    public class ReverseIntake implements Action {
        public boolean run(TelemetryPacket tPkt){
            reverseIntake();
            return false;
        }
    }
    public Action reverseInt = new ReverseIntake();


    public class raiseBox implements Action {
        public boolean run(TelemetryPacket tPkt){
            secureStackIntakePixels();
            return false;
        }
    }
    public Action securePixels = new raiseBox();


    private static final double LEFT_SPIKEMARK_BOUNDARY_X = 300;
    private static final double RIGHT_SPIKEMARK_BOUNDARY_X = 130;

    public static int PurpleDropOffPos = 0;
    public static double SplineAngle = 0;
    public static double TangentAngle = -70;

    boolean bRunningTrajectory = false;

    AuraRobot Aurelius = new AuraRobot();
    AuraIntakeOuttakeController MyIntakeOuttakeController;
    MecanumDrive RedShort;


    private static FtcDashboard auraBoard;

    //TODO: imu
    public class IMUController implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {

            double oldHeading = RedShort.pose.heading.log();
            telemetry.addData("Old heading", Math.toDegrees(oldHeading));
            double yaw = Aurelius.myHeadingEstimator.getYaw();
            telemetry.addData("IMU Heading correction: ", Math.toDegrees(yaw - oldHeading));
            telemetry.addData("Corrected heading:", Math.toDegrees(yaw));
            telemetry.update();

            RedShort.pose = new Pose2d(RedShort.pose.position.x, RedShort.pose.position.y, yaw);

            return false;
        }
    }

    public Action rectifyHeadingError = new IMUController();

    //TODO: declare April Tag stuff
    OpenCvWebcam Sauron = null;
    AprilTagDetectionPipeline pipeline;

    private static int iTeleCt = 1;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.04;


//TODO: Declare AprilTag IDS below
//Example: last year's custom sleeve tags

//    // Tag ID 2,9,16 from 36h11 family
//    int LEFT = 2;
//    int MIDDLE = 9;
//    int RIGHT = 20;

    public AprilTagDetection tagOfInterest = null;

//TODO: Declare Tensorflow thing below
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "myRedpy.tflite";

    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
//    private static final String TFOD_MODEL_FILE = "C:\\Sashank\\FTC CenterStage\\Aurelius\\Aurelius\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\myRedpy.tflite";

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
            "Bloopy",
            "Redpy"
    };

    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortalEyeball;
    private VisionPortal visionPortalKemera;
    private VisionPortal visionPortalTfod;

    private static final boolean USE_WEBCAM = true;
    public static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagProcessor aprilTagEyeball;              // Used for managing the AprilTag detection process.
    private AprilTagProcessor aprilTagKemera;
    private org.firstinspires.ftc.vision.apriltag.AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected


    // TODO: define trajectory variables here
    // Purple Trajectories
    private Action dropOffPurpleAtPos1;
    private Action dropOffPurpleAtPos2;
    private Action dropOffPurpleAtPos3;

    // Yellow Trajectories
    private Action dropOffYellowAtPos1;
    private Action dropOffYellowAtPos2;
    private Action dropOffYellowAtPos3;

    // Park Trajectories
    private Action dropOffYellowAtPark;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        auraBoard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize...
        Aurelius.init(hardwareMap);
        Aurelius.PurpleDumper.setPosition(PURPLE_LOCK);
        Aurelius.boeing747.init();
        Aurelius.hanger.init();
        Aurelius.LeftLink.setPosition(leftLinkageClose);
        Aurelius.RightLink.setPosition(rightLinkageClose);
        Aurelius.hanger.update();
        telemetry.addLine(String.format("%d. Aura Initialized!", iTeleCt++));
        telemetry.update();


        double volts = getBatteryVoltage();
        telemetry.addLine(String.format("%d. Battery voltage: %.1f volts", iTeleCt++, volts));
        telemetry.update();

        RedShort = new MecanumDrive(Aurelius.hwMap, redShortStartPos);
        Aurelius.myHeadingEstimator = new AuraHeadingEstimator(Aurelius.hwMap, redShortStartPos);
        telemetry.addLine(String.format("%d. myHeadingEstimator Initialized!", iTeleCt++));
        telemetry.update();

        ElapsedTime trajectoryTimer = new ElapsedTime(MILLISECONDS);
        MyIntakeOuttakeController = new AuraIntakeOuttakeController(hardwareMap, false);
        MyIntakeOuttakeController.setTargetState(STATE_3_PS);

        // Build trajectories here ...
        telemetry.addData("Status: ", "Building Trajectories......");
        telemetry.update();
        buildPurpleTrajectories();
        buildYellowTrajectories();
        telemetry.addData("Status: ", "Building Trajectories......done");
        telemetry.update();

        // Initialize TFOD and report what's detected until start is pushed
        telemetry.addData("Status: ", "Initializing Tensor Flow ......");
        telemetry.update();
        initTfod();
        //initAprilTagEyeball();
        //initAprilTagKemera();

        runtime.reset();
        while(runtime.seconds() < 3)
            idle();
        telemetry.addData("Status: ", "Tensor flow ready!");
        telemetry.update();


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (!isStarted()) {
            telemetryTfod();
            MyIntakeOuttakeController.update();
        }

        // Set zero direction on IMU
        Aurelius.myHeadingEstimator.resetYaw();

        runtime.reset();
        if (opModeIsActive()) {
            DetectPurpleDropoffPos();
            visionPortalKemera.close();
            telemetry.addData("Going to position:", "PurpleDropOffPos");
            telemetry.update();

            //TODO: Run Trajectories
            switch (PurpleDropOffPos) {
                case 1:
                    Actions.runBlocking(
                            new ParallelAction(
                                    new SequentialAction(
                                            beginTrajectoryMarker,
                                            dropOffPurpleAtPos1,
                                            dropOffYellowAtPos1,
                                            endTrajectoryMarker),
                                    new ParallelAction(
                                            updateIOController,
                                            initApril)
                            ));
                    break;
                case 2:
                    Actions.runBlocking(
                            new ParallelAction(
                                    new SequentialAction(
                                            beginTrajectoryMarker,
                                            dropOffPurpleAtPos2,
                                            dropOffYellowAtPos2,
                                            endTrajectoryMarker),
                                    new ParallelAction(
                                            updateIOController,
                                            initApril)
                            ));
                    break;
                case 3:
                default:
                    Actions.runBlocking(
                            new ParallelAction(
                                    new SequentialAction(
                                            beginTrajectoryMarker,
                                            dropOffPurpleAtPos3,
                                            dropOffYellowAtPos3,
                                            endTrajectoryMarker),
                                    new ParallelAction(
                                            updateIOController,
                                            initApril)
                            ));
                    break;
            }
        }
    }

    void buildPurpleTrajectories()
    {
        dropOffPurpleAtPos3 = RedShort.actionBuilder(redShortStartPos)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redShortPurple3Pos, Math.toRadians(90), null, new ProfileAccelConstraint(AUTO_ACCEL_CONSTRAINT_1,AUTO_ACCEL_CONSTRAINT_2))
                .stopAndAdd(ejectPurple)
                .waitSeconds(AUTO_WAIT_FOR_PURPLE_DROP)
                .build();

        dropOffPurpleAtPos2 = RedShort.actionBuilder(redShortStartPos)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redShortPurple2Pos, Math.toRadians(90), null, new ProfileAccelConstraint(AUTO_ACCEL_CONSTRAINT_1,AUTO_ACCEL_CONSTRAINT_2))
                .stopAndAdd(ejectPurple)
                .waitSeconds(AUTO_WAIT_FOR_PURPLE_DROP)
                .build();

        dropOffPurpleAtPos1 = RedShort.actionBuilder(redShortStartPos)
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(redShortPurple1Pos, Math.toRadians(130), null, new ProfileAccelConstraint(AUTO_ACCEL_CONSTRAINT_1,AUTO_ACCEL_CONSTRAINT_2))
                .stopAndAdd(ejectPurple)
                .waitSeconds(AUTO_WAIT_FOR_PURPLE_DROP)
                .build();
    }

    void buildYellowTrajectories()
    {
        dropOffYellowAtPos3 = RedShort.actionBuilder(redShortPurple3Pos)
            //dropoff yellow
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(new Vector2d(10,-38.5))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(redShortTagPos, Math.toRadians(90))
                .afterDisp(0, getReadyForOutTake)
                .stopAndAdd(updatePosFromAprilTagKemera)
                .strafeTo(redShortYellow3Pos)
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
                .stopAndAdd(depositYellow)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .afterDisp(0, getReadyForIntake)
            //cycle 2 white
                .strafeTo(redShortEntryPos)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(redShortWingPos, new TranslationalVelConstraint(65))
                .strafeTo(redShortStackPos, new TranslationalVelConstraint(25))
                .afterDisp(0,deployStackIntake) // Make sure to turn on bottom roller
                .stopAndAdd(intakeFromStack) // Make sure to flip box and lock fingers
                .waitSeconds(AUTO_WAIT_FOR_STACK_INTAKE)
                .stopAndAdd(securePixels)
                .strafeTo(redShortReturnPos)
                .afterDisp(0, retractStackIntake)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(redShortEntryPos)
                .afterDisp(50, getReadyForOutTake)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(redShortTagPos.position)
                .stopAndAdd(updatePosFromAprilTagKemera)
                .strafeTo(redShortWhite2Pos)
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
                .stopAndAdd(depositYellow)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .afterDisp(0,getReadyForIntake)
            //drop 2nd cycle in back
//            .splineToConstantHeading(blueShortCycleLongAfterGatePos, Math.toRadians(180))
                .strafeTo(redShortCycleLongAfterGatePos)
                .stopAndAdd(rectifyHeadingError)
                .setReversed(true)
                .lineToXConstantHeading(-55.5)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(redShortCycleLongPreStackPos2, Math.toRadians(180))
//            .strafeTo(blueShortCycleLongPreStackPos2)
                .stopAndAdd(deploy2StackIntake)
//            .afterDisp(0, deployStackIntake)
                .strafeTo(redShortCycleLongPostStackPos2) //, Math.toRadians(-90), new TranslationalVelConstraint(25))
                .stopAndAdd(intakeFromStack) // Make sure to flip box and lock fingers
                .waitSeconds(AUTO_WAIT_FOR_STACK_INTAKE)
                .stopAndAdd(securePixels)
                .setReversed(false)
                .splineToConstantHeading(redShortCycleLongBeforeGatePos, Math.toRadians(0))
                .afterDisp(0, retractStackIntake)
                .stopAndAdd(rectifyHeadingError)
                .lineToXConstantHeading(36)
                .afterDisp(50, getReadyForOutTake)
                .lineToXConstantHeading(55)
                .stopAndAdd(dumpWhite)
                .stopAndAdd(getReadyForIntake)
                .waitSeconds(AUTO_WAIT_RETURN_TO_INTAKE)
                .build();

        dropOffYellowAtPos2 = RedShort.actionBuilder(redShortPurple2Pos)
            //dropoff yellow
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(new Vector2d(10,-38.5))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(redShortTagPos, Math.toRadians(90))
                .afterDisp(0, getReadyForOutTake)
                .stopAndAdd(updatePosFromAprilTagKemera)
                .strafeTo(redShortYellow2Pos)
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
                .stopAndAdd(depositYellow)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .afterDisp(0, getReadyForIntake)
            //cycle 2 white
                .strafeTo(redShortEntryPos)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(redShortWingPos, new TranslationalVelConstraint(65))
                .strafeTo(redShortStackPos, new TranslationalVelConstraint(25))
                .afterDisp(0,deployStackIntake) // Make sure to turn on bottom roller
                .stopAndAdd(intakeFromStack) // Make sure to flip box and lock fingers
                .waitSeconds(AUTO_WAIT_FOR_STACK_INTAKE)
                .stopAndAdd(securePixels)
                .strafeTo(redShortReturnPos)
                .afterDisp(0, retractStackIntake)
                .strafeTo(redShortEntryPos)
                .afterDisp(50, getReadyForOutTake)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(redShortTagPos.position)
                .stopAndAdd(updatePosFromAprilTagKemera)
                .strafeTo(redShortWhite3Pos)
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
                .stopAndAdd(depositYellow)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .afterDisp(0,getReadyForIntake)
            //drop 2nd cycle in back
//            .splineToConstantHeading(blueShortCycleLongAfterGatePos, Math.toRadians(180))
                .strafeTo(redShortCycleLongAfterGatePos)
                .stopAndAdd(rectifyHeadingError)
                .setReversed(true)
                .lineToXConstantHeading(-55.5)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(redShortCycleLongPreStackPos2, Math.toRadians(180))
//            .strafeTo(blueShortCycleLongPreStackPos2)
                .stopAndAdd(deploy2StackIntake)
//            .afterDisp(0, deployStackIntake)
                .strafeTo(redShortCycleLongPostStackPos2) //, Math.toRadians(-90), new TranslationalVelConstraint(25))
                .stopAndAdd(intakeFromStack) // Make sure to flip box and lock fingers
                .waitSeconds(AUTO_WAIT_FOR_STACK_INTAKE)
                .stopAndAdd(securePixels)
                .setReversed(false)
                .splineToConstantHeading(redShortCycleLongBeforeGatePos, Math.toRadians(0))
                .afterDisp(0, retractStackIntake)
                .stopAndAdd(rectifyHeadingError)
                .lineToXConstantHeading(36)
                .afterDisp(50, getReadyForOutTake)
                .lineToXConstantHeading(55)
                .stopAndAdd(dumpWhite)
                .stopAndAdd(getReadyForIntake)
                .waitSeconds(AUTO_WAIT_RETURN_TO_INTAKE)
                .build();


        dropOffYellowAtPos1 = RedShort.actionBuilder(redShortPurple1Pos)
            //dropoff yellow
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(new Vector2d(10,-38.5))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(redShortTagPos, Math.toRadians(90))
                .afterDisp(0, getReadyForOutTake)
                .stopAndAdd(updatePosFromAprilTagKemera)
                .strafeTo(redShortYellow1Pos)
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
                .stopAndAdd(depositYellow)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .afterDisp(0, getReadyForIntake)
            //cycle 2 white
                .strafeTo(redShortEntryPos)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(redShortWingPos, new TranslationalVelConstraint(65))
                .strafeTo(redShortStackPos, new TranslationalVelConstraint(25))
                .afterDisp(0,deployStackIntake) // Make sure to turn on bottom roller
                .stopAndAdd(intakeFromStack) // Make sure to flip box and lock fingers
                .waitSeconds(AUTO_WAIT_FOR_STACK_INTAKE)
                .stopAndAdd(securePixels)
                .strafeTo(redShortWingPos)
                .afterDisp(0, retractStackIntake)
                .strafeTo(redShortEntryPos)
                .afterDisp(50, getReadyForOutTake)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(redShortTagPos.position)
                .stopAndAdd(updatePosFromAprilTagKemera)
                .strafeTo(redShortWhite2Pos)
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
                .stopAndAdd(depositYellow)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .afterDisp(0,getReadyForIntake)
            //drop 2nd cycle in back
//            .splineToConstantHeading(blueShortCycleLongAfterGatePos, Math.toRadians(180))
                .strafeTo(redShortCycleLongAfterGatePos)
                .stopAndAdd(rectifyHeadingError)
                .setReversed(true)
                .lineToXConstantHeading(-55.5)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(redShortCycleLongPreStackPos2, Math.toRadians(180))
//            .strafeTo(blueShortCycleLongPreStackPos2)
                .stopAndAdd(deploy2StackIntake)
//            .afterDisp(0, deployStackIntake)
                .strafeTo(redShortCycleLongPostStackPos2) //, Math.toRadians(-90), new TranslationalVelConstraint(25))
                .stopAndAdd(intakeFromStack) // Make sure to flip box and lock fingers
                .waitSeconds(AUTO_WAIT_FOR_STACK_INTAKE)
                .stopAndAdd(securePixels)
                .setReversed(false)
                .splineToConstantHeading(redShortCycleLongBeforeGatePos, Math.toRadians(0))
                .afterDisp(0, retractStackIntake)
                .stopAndAdd(rectifyHeadingError)
                .lineToXConstantHeading(36)
                .afterDisp(50, getReadyForOutTake)
                .lineToXConstantHeading(55)
                .stopAndAdd(dumpWhite)
                .stopAndAdd(getReadyForIntake)
                .waitSeconds(AUTO_WAIT_RETURN_TO_INTAKE)
                .build();
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        // Create the vision portal the easy way.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Eyeball"));


        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortalKemera = builder.build();

    } // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        telemetry.update();
    }   // end method telemetryTfod()

    void DetectPurpleDropoffPos()
    {
        double x = 0, y = 0;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        for(Recognition recognition : currentRecognitions ) {
            if (recognition.getLabel() == "Pixel") {
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                y = (recognition.getTop() + recognition.getBottom()) / 2;
            }
            break;
        }
        if( x > 0 && x < LEFT_SPIKEMARK_BOUNDARY_X )
            PurpleDropOffPos = 2;
        else if (x > RIGHT_SPIKEMARK_BOUNDARY_X)
            PurpleDropOffPos = 3;
        else
            PurpleDropOffPos = 1;

        telemetry.addData("Detected Spike Mark X = ", x);
        telemetry.addData("Detected Drop off Position = ", PurpleDropOffPos);

        // Push telemetry to the Driver Station.
        telemetry.update();

    }

    boolean updateposFromAprilTagEyeball()
    {
//        initAprilTag(); // initializing the april tag processor
        setManualExposureEyeball(6, 250); // accounting for motion blur
        targetFound = false;
        desiredTag  = null;

        ElapsedTime AprilTagTimer = new ElapsedTime();
        AprilTagTimer.reset();
        while(!targetFound && AprilTagTimer.seconds() < APRILTAG_TIMEOUT) {
            List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTagEyeball.getDetections();
            for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    } else {
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        telemetry.update();
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    telemetry.update();
                }
            }
        }

        if(targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.fieldPosition);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            telemetry.update();

            // TODO: 1. Calibrate camera
            //       2. Offset currX and currY from camera to robot center

            double range = desiredTag.ftcPose.range;
            double bearing = desiredTag.ftcPose.bearing;
            double yaw = desiredTag.ftcPose.yaw;

            double robotOffsetX = -7;
            double robotOffsetY = +5.5;

            double offsetX = (range * Math.cos(Math.toRadians(bearing)));

            double offsetY = (range * Math.sin(Math.toRadians(bearing)));

            double currHeading = -Math.toRadians(yaw);

            double rotateX = (robotOffsetX * Math.cos(currHeading)) + (robotOffsetY * -Math.sin(currHeading));
            double rotateY = (robotOffsetX * Math.sin(currHeading)) + (robotOffsetY * Math.cos(currHeading));

            double currX = rotateX + (desiredTag.metadata.fieldPosition.getData()[0] -
                    offsetX);

            double currY = rotateY + (desiredTag.metadata.fieldPosition.getData()[1] -
                    offsetY);

            telemetry.addData("Current pos:", "X: %5.1f Y: %5.1f Heading: %5.1f degrees", RedShort.pose.position.x, RedShort.pose.position.y, Math.toDegrees(RedShort.pose.heading.log()));
            telemetry.update();

            RedShort.pose = new Pose2d(currX, currY, currHeading);
            telemetry.addData("Updated pos:", "X: %5.1f Y: %5.1f Heading %5.1f degrees", RedShort.pose.position.x, RedShort.pose.position.y, Math.toDegrees(RedShort.pose.heading.log()));
            telemetry.update();
            return true;
        }
        telemetry.addLine("Not Found: Desired Tag not found");
        telemetry.update();
        return false;
    }

    private void initAprilTagEyeball() {
        // Create the AprilTag processor by using a builder.
        aprilTagEyeball = new AprilTagProcessor.Builder()
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagEyeball.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortalEyeball = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Eyeball"))
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(aprilTagEyeball)
                    .build();
        } else {
            visionPortalEyeball = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTagEyeball)
                    .build();
        }
    }

    private void setManualExposureEyeball(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortalEyeball == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortalEyeball.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortalEyeball.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortalEyeball.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortalEyeball.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    boolean updateposFromAprilTagKemera()
    {
//        initAprilTag(); // initializing the april tag processor
        setManualExposureKemera(6, 250); // accounting for motion blur
        targetFound = false;
        desiredTag  = null;

        ElapsedTime AprilTagTimer = new ElapsedTime();
        AprilTagTimer.reset();
        while(!targetFound && AprilTagTimer.seconds() < APRILTAG_TIMEOUT) {
            List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTagKemera.getDetections();
            for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    } else {
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        telemetry.update();
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    telemetry.update();
                }
            }
        }

        if(targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.fieldPosition);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            telemetry.update();

            // TODO: 1. Calibrate camera
            //       2. Offset currX and currY from camera to robot center

            double range = desiredTag.ftcPose.range;
            double bearing = desiredTag.ftcPose.bearing;
            double yaw = desiredTag.ftcPose.yaw;

            double robotOffsetX = -7;
            double robotOffsetY = -3.75;

            double offsetX = (range * Math.cos(Math.toRadians(bearing)));

            double offsetY = (range * Math.sin(Math.toRadians(bearing)));

            double currHeading = -Math.toRadians(yaw);

            double rotateX = (robotOffsetX * Math.cos(currHeading)) + (robotOffsetY * -Math.sin(currHeading));
            double rotateY = (robotOffsetX * Math.sin(currHeading)) + (robotOffsetY * Math.cos(currHeading));

            double currX = rotateX + (desiredTag.metadata.fieldPosition.getData()[0] -
                    offsetX);

            double currY = rotateY + (desiredTag.metadata.fieldPosition.getData()[1] -
                    offsetY);

            telemetry.addData("Current pos:", "X: %5.1f Y: %5.1f Heading: %5.1f degrees", RedShort.pose.position.x, RedShort.pose.position.y, Math.toDegrees(RedShort.pose.heading.log()));
            telemetry.update();

            RedShort.pose = new Pose2d(currX, currY, currHeading);
            telemetry.addData("Updated pos:", "X: %5.1f Y: %5.1f Heading %5.1f degrees", RedShort.pose.position.x, RedShort.pose.position.y, Math.toDegrees(RedShort.pose.heading.log()));
            telemetry.update();
            return true;
        }
        telemetry.addLine("Not Found: Desired Tag not found");
        telemetry.update();
        return false;
    }

    private void initAprilTagKemera() {
        // Create the AprilTag processor by using a builder.
        aprilTagKemera = new AprilTagProcessor.Builder()
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagKemera.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortalKemera = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Kemera"))
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(aprilTagKemera)
                    .build();
        } else {
            visionPortalKemera = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTagKemera)
                    .build();
        }
    }

    private void setManualExposureKemera(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortalKemera == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortalKemera.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortalKemera.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortalKemera.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortalKemera.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public void reverseIntake()
    {
        Aurelius.setPower(AuraRobot.AuraMotors.INTAKE, -.7);
    }

    public void lowerStackIntake()
    {
        Aurelius.LeftLink.setPosition(redShort_leftLinkageOpen);
        Aurelius.RightLink.setPosition(redShort_rightLinkageOpen);
    }

    public void lowerStackIntake2()
    {
        Aurelius.LeftLink.setPosition(left2LinkageOpen);
        Aurelius.RightLink.setPosition(right2LinkageOpen);
    }

    public void raiseStackIntake()
    {
        Aurelius.LeftLink.setPosition(redShort_leftLinkageClose);
        Aurelius.RightLink.setPosition(redShort_rightLinkageClose);
    }

    public void stackIntakePixels()
    {
        Aurelius.setPower(AuraRobot.AuraMotors.INTAKE, 0.7);
        Aurelius.Ramp.setPosition(Ramp_Down_Pos);
    }

    public void secureStackIntakePixels()
    {
        Aurelius.setPower(AuraRobot.AuraMotors.INTAKE, 0);
        Aurelius.Ramp.setPosition(Ramp_Up_Pos);
        MyIntakeOuttakeController.setTargetState(STATE_3_PS);
    }


}