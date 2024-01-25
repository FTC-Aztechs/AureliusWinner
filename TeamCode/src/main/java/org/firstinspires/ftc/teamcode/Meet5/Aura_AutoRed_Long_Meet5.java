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

package org.firstinspires.ftc.teamcode.Meet5;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.ioState.STATE_1_RFI;
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.ioState.STATE_3_PS;
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.ioState.STATE_5_RFO_MANUAL;
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.ioState.STATE_6_PR_BOTH;
import static org.firstinspires.ftc.teamcode.AuraRobot.APRILTAG_TIMEOUT;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_FOR_OUTTAKE;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_FOR_YELLOW_DROP;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_RETURN_TO_INTAKE;
import static org.firstinspires.ftc.teamcode.AuraRobot.AuraMotors.INTAKE;
import static org.firstinspires.ftc.teamcode.AuraRobot.PURPLE_LOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.PURPLE_UNLOCK;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
import org.firstinspires.ftc.teamcode.AuraHangController;
import org.firstinspires.ftc.teamcode.AuraHeadingEstimator;
import org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController;
import org.firstinspires.ftc.teamcode.AuraRobot;
import org.firstinspires.ftc.teamcode.roadrunnerbasics.MecanumDrive;
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
@Autonomous(name="Red_Long", group="Linear OpMode")

public class Aura_AutoRed_Long_Meet5 extends LinearOpMode {

    //**** Roadrunner Pose2ds ****

    //Todo: switch to field coordinates:
    //to find x: add robotcentric Y pos to field centric start pos X
    //to find y: subtract robotcentric X pos to field centric start pos y
    //to find heading: add -90 degrees to field centric start pos heading

    Pose2d redStartPos = new Pose2d(-36,-61.5,Math.toRadians(90));//0,0,0

    Pose2d redPurple1Pos = new Pose2d(-38, -34.5 , Math.toRadians(180)); //27,19,-90
    Pose2d redPurple2Pos = new Pose2d(-31,- 33, Math.toRadians(90));  //37,12,-90
    Pose2d redPurple3Pos = new Pose2d(-34, -34.5, Math.toRadians(0));  //27,0,-90

    Vector2d redBeforeGatePos3 = new Vector2d(-38,-11.5);//50,2
    Vector2d redBeforeGatePos2 = new Vector2d(-50,-11.5);//50,-14
    Vector2d redBeforeGatePos1 = new Vector2d(-34,-11.5);//50,-19
    Vector2d redAfterGateTagPos = new Vector2d(15.25, -11.5);//50,51.25
    Vector2d redAfterGatePos = new Vector2d(32, -11.5);//50,68

    Vector2d redYellow3Pos = new Vector2d(45, -42);  //27,37,-90
    Vector2d redYellow2Pos = new Vector2d(45, -35.5);   //26,37,-90
    Pose2d redYellow1Pos = new Pose2d(45,-28.5,Math.toRadians(0));    //33,37,-90


    Vector2d redParkPos = new Vector2d(51.5, -11.5);//50, 82
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
            MyIntakeOuttakeController.setTargetState(STATE_5_RFO_MANUAL);
            return false;
        }
    }
    public Action getReadyForOutTake = new GotoOuttakeAction();

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

    public class backwallAprilTagController implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {
            updatePosfromBackwallAprilTag();
            return false;
        }
    }

    public Action updateAfterGatePos = new backwallAprilTagController();

    private static final double LEFT_SPIKEMARK_BOUNDARY_X = 250;
    private static final double RIGHT_SPIKEMARK_BOUNDARY_X = 260;

    public static int PurpleDropOffPos = 0;
    public static double SplineAngle = 0;
    public static double TangentAngle = -70;

    boolean bRunningTrajectory = false;

    AuraRobot Aurelius = new AuraRobot();
    AuraIntakeOuttakeController MyIntakeOuttakeController;
    MecanumDrive RedLong;
    


    private static FtcDashboard auraBoard;


    //TODO: imu
    public class IMUController implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {

            double oldHeading = RedLong.pose.heading.log();
            telemetry.addData("Old heading", Math.toDegrees(oldHeading));
            double yaw = Aurelius.myHeadingEstimator.getYaw();
            telemetry.addData("IMU Heading correction: ", Math.toDegrees(yaw - oldHeading));
            telemetry.addData("Corrected heading:", Math.toDegrees(yaw));
            telemetry.update();

            RedLong.pose = new Pose2d(RedLong.pose.position.x, RedLong.pose.position.y, yaw);

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
    private VisionPortal visionPortal;

    private static final boolean USE_WEBCAM = true;
    public static final int DESIRED_TAG_ID = 4;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
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
        Aurelius.hanger.update();
        telemetry.addLine(String.format("%d. Aura Initialized!", iTeleCt++));
        telemetry.update();

        double volts = getBatteryVoltage();
        telemetry.addLine(String.format("%d. Battery voltage: %.1f volts", iTeleCt++, volts));
        telemetry.update();

        RedLong = new MecanumDrive(Aurelius.hwMap, redStartPos);
        Aurelius.myHeadingEstimator = new AuraHeadingEstimator(Aurelius.hwMap, redStartPos);
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
            visionPortal.close();
            telemetry.addData("Going to position:", "PurpleDropOffPos");
            telemetry.update();

            // Wait 5 seconds to let other robot finish
            runtime.reset();
            while (runtime.seconds() < 5) {
                MyIntakeOuttakeController.update();
            }

            switch (PurpleDropOffPos) {
                case 1:
                    Actions.runBlocking(
                        new ParallelAction(
                            new SequentialAction(
                                beginTrajectoryMarker,
                                dropOffPurpleAtPos1,
                                dropOffYellowAtPos1,
                                endTrajectoryMarker),
                            updateIOController
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
                        updateIOController
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
                        updateIOController
                    ));
                    break;
            }
        }
    }

    void buildPurpleTrajectories()
    {
        dropOffPurpleAtPos3 = RedLong.actionBuilder(redStartPos)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redPurple3Pos, Math.toRadians(90))
                .stopAndAdd(ejectPurple)
                .waitSeconds(1)
                .build();

        dropOffPurpleAtPos2 = RedLong.actionBuilder(redStartPos)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redPurple2Pos, Math.toRadians(90))
                .stopAndAdd(ejectPurple)
                .waitSeconds(1)
                .build();

        dropOffPurpleAtPos1 = RedLong.actionBuilder(redStartPos)
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(redPurple1Pos, Math.toRadians(130))
                .stopAndAdd(ejectPurple)
                .waitSeconds(1)
                .build();
    }

    void buildYellowTrajectories()
    {
        dropOffYellowAtPos3 = RedLong.actionBuilder(redPurple3Pos)
                .setReversed(false)
                .lineToX(-38)
                .strafeTo(redBeforeGatePos3)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(redAfterGateTagPos)
                .afterDisp(0, getReadyForOutTake)
                .stopAndAdd(updateAfterGatePos)
                .splineToLinearHeading(redYellow1Pos,Math.toRadians(0))
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(redYellow3Pos)
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
				.stopAndAdd(depositYellow)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .strafeTo(redParkPos)
                .afterDisp(0, getReadyForIntake)
                .waitSeconds(AUTO_WAIT_RETURN_TO_INTAKE)
                .build();

        dropOffYellowAtPos2 = RedLong.actionBuilder(redPurple2Pos)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-50,-45, Math.toRadians(0)),Math.toRadians(-180))
                .strafeTo(redBeforeGatePos2)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(redAfterGateTagPos)
                .afterDisp(0, getReadyForOutTake)
                .stopAndAdd(updateAfterGatePos)
                .splineToLinearHeading(redYellow1Pos,Math.toRadians(0))
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(redYellow2Pos)
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
				.stopAndAdd(depositYellow)
                .strafeTo(redParkPos)
                .afterDisp(0, getReadyForIntake)
                .waitSeconds(AUTO_WAIT_RETURN_TO_INTAKE)
                .build();

        dropOffYellowAtPos1 = RedLong.actionBuilder(redPurple1Pos)
                .setTangent(Math.toRadians(-90))
                .lineToX(-34)
                .strafeTo(redBeforeGatePos1)
                .turn(Math.toRadians(-180))
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(redAfterGateTagPos)
                .afterDisp(0, getReadyForOutTake)
                .stopAndAdd(updateAfterGatePos)
                .splineToLinearHeading(redYellow1Pos,Math.toRadians(0))
                .stopAndAdd(rectifyHeadingError)
                .waitSeconds(AUTO_WAIT_FOR_OUTTAKE)
				.stopAndAdd(depositYellow)
                .waitSeconds(AUTO_WAIT_FOR_YELLOW_DROP)
                .strafeTo(redParkPos)
                .afterDisp(0, getReadyForIntake)
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
        builder.setCamera(hardwareMap.get(WebcamName.class, "Kemera"));


        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

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
            PurpleDropOffPos = 1;
        else if (x > RIGHT_SPIKEMARK_BOUNDARY_X)
            PurpleDropOffPos = 2;
        else
            PurpleDropOffPos = 3;

        telemetry.addData("Detected Spike Mark X = ", x);
        telemetry.addData("Detected Drop off Position = ", PurpleDropOffPos);

        // Push telemetry to the Driver Station.
        telemetry.update();

    }

    boolean updatePosfromBackwallAprilTag()
    {
        initAprilTag(); // initializing the april tag processor
        setManualExposure(6, 250); // accounting for motion blur
        targetFound = false;
        desiredTag  = null;

        ElapsedTime AprilTagTimer = new ElapsedTime();
        AprilTagTimer.reset();
        while(!targetFound && AprilTagTimer.seconds() < APRILTAG_TIMEOUT) {
            List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
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

            double currX = desiredTag.metadata.fieldPosition.getData()[0] -
                    (desiredTag.ftcPose.range * Math.sin(Math.toRadians(desiredTag.ftcPose.bearing)));

            double currY = desiredTag.metadata.fieldPosition.getData()[1] -
                    (desiredTag.ftcPose.range * Math.cos(Math.toRadians(desiredTag.ftcPose.bearing)));


            double deltaHeading = -desiredTag.ftcPose.yaw;

            telemetry.addData("Current pos:", "X: %5.1f Y: %5.1f Heading: %5.1f degrees", currX, currY, Math.toDegrees(RedLong.pose.heading.log()));
//            telemetry.addData("Deltas", "X: %5.1f Y: %5.1f Heading: %5.1f degrees", deltaX, deltaY, deltaHeading);
            telemetry.update();

            RedLong.pose = new Pose2d(currX, currY,Math.toRadians(-90) - Math.toRadians(deltaHeading));
            telemetry.addData("Updated pos:", "X: %5.1f Y: %5.1f Heading %5.1f degrees", RedLong.pose.position.x, RedLong.pose.position.y, Math.toDegrees(RedLong.pose.heading.log()));
            telemetry.update();
            return true;
        }
        telemetry.addLine("Not Found: Desired Tag not found");
        telemetry.update();
        return false;
    }


    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Eyeball"))
                        .addProcessor(aprilTag)
                        .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }


}
