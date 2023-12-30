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

package org.firstinspires.ftc.teamcode.Meet4;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.AuraRobot.APRILTAG_TIMEOUT;
import static org.firstinspires.ftc.teamcode.AuraRobot.AuraMotors.INTAKE;
import static org.firstinspires.ftc.teamcode.Aura_DepositController.DepositState.Down;
import static org.firstinspires.ftc.teamcode.Aura_DepositController.DepositState.Open;
import static org.firstinspires.ftc.teamcode.Aura_DepositController.DepositState.Up;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
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
@Autonomous(name="Coach_Blue_Long", group="Linear OpMode")
@Disabled


public class Aura_AutoBlue_Long_Coach extends LinearOpMode {

    //**** Roadrunner Pose2ds ****

    Pose2d StartPos = new Pose2d(0,0,0);

    Pose2d Purple1Pos = new Pose2d(28, 2, Math.toRadians(90));
    Pose2d Purple2Pos = new Pose2d(36, -14, Math.toRadians(90));
    Pose2d Purple3Pos = new Pose2d(28, -19, Math.toRadians(90));

    Pose2d Yellow1Pos = new Pose2d(15, 88, Math.toRadians(-90));
    Pose2d Yellow2Pos = new Pose2d(20, 87.25, Math.toRadians(-90));
    Pose2d Yellow3Pos = new Pose2d(30, 87.25, Math.toRadians(-90));

    Vector2d BeforeGatePos1 = new Vector2d(50,2);
    Vector2d BeforeGatePos2 = new Vector2d(50,-14);
    Vector2d BeforeGatePos3 = new Vector2d(50,-19);
    double BeforeGateHeading = 90;

    Vector2d AfterGatePos   = new Vector2d(45,58);
    double AfterGateHeading = 90;

    // Set these manually from the Robot once it is at AfterGatePos.
    double RangeCalibrated   = 0.0;
    double YawCalibrated     = 0.0;
    double BearingCalibrated = 0.0;
    boolean bProceedToYellow = false;

    Vector2d ParkPos = new Vector2d(42, 82);

    public static double purpleOffset1 = 1.0;
    public static double purpleOffset2 = 1.0;
    public static double purpleOffset3 = 1.0;
    public static double yellowOffset1 = 1.0;
    public static double yellowOffset2 = 1.0;
    public static double yellowOffset3 = 1.0;

    public static double CalibratedYaw = 0;

    public static double CalibratedBearing = 0;

    public static double CalibratedRange = 0;

    //Roadrunner quick guide brought to you by Lavanya

    //y+ robot drives toward backdroo
    //y- robot drives away from backdrop
    //x- robot drives closer to starting wall
    //x+ robot drives toward the center of the field

    //tangent parameter in splines = changing angle changes the shape of the path
    //setTangent() = changes the direction in which the robot initially starts to drive
    //90 = to the left
    //180 = to the back
    //-90 = to the right
    //0 = forward

    //************

    public class IntakeController implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {
            ElapsedTime actionTimer = new ElapsedTime();
            actionTimer.reset();
            while(actionTimer.seconds() < 0.8) {
                Aurelius.setPower(INTAKE, -0.2);
            }
            Aurelius.setPower(INTAKE, 0);
            return false;
        }
    }

    public Action ejectPurple = new IntakeController();

    public class DepositController implements Action {
        @Override
        public boolean run (TelemetryPacket tPkt) {
            sleep(500);
            Aurelius.depositFlipper.setTargetState(Up);
            Aurelius.depositFlipper.update();

            sleep(1500);
            Aurelius.depositFlipper.setTargetState(Open);
            Aurelius.depositFlipper.update();

            sleep(500);
            Aurelius.depositFlipper.setTargetState(Down);
            Aurelius.depositFlipper.update();

            sleep(500);

            return false;
        }
    }

    public Action depositYellow = new DepositController();

    public class IMUController implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {
            return false;

        }
    }

    public Action rectifyHeadingError = new IMUController();

    public class backwallAprilTagController implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {
            if(updatePosfromBackwallAprilTag()) {
                bProceedToYellow = true;
            } else {
                bProceedToYellow = false;
            }
            return false;
        }
    }

    public Action updateAfterGatePos = new backwallAprilTagController();

    private static final double LEFT_SPIKEMARK_BOUNDARY_X = 250;
    private static final double RIGHT_SPIKEMARK_BOUNDARY_X = 260;

    public static int PurpleDropOffPos = 0;
    public static double SplineAngle = 0;
    public static double TangentAngle = -70;


    AuraRobot Aurelius = new AuraRobot();
    MecanumDrive BlueLong;

    private static FtcDashboard auraBoard;

    //TODO: declare April Tag stuffi
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
    private static final String TFOD_MODEL_ASSET = "myBloopy.tflite";

    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
//    private static final String TFOD_MODEL_FILE = "C:\\Sashank\\FTC CenterStage\\Aurelius\\Aurelius\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\myBloopy.tflite";

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

    private static final boolean CAMERA_SIDE = false;
    private static final int DESIRED_TAG_ID = 1;     // Choose the tag you want to approach or set to -1 for ANY tag.
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

        Aurelius.init(hardwareMap);
        BlueLong = new MecanumDrive(Aurelius.hwMap, new Pose2d(0,0,0));
        ElapsedTime trajectoryTimer = new ElapsedTime(MILLISECONDS);

        auraBoard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine(String.format("%d. Aura Initialized!", iTeleCt++));

        double volts = getBatteryVoltage();
        telemetry.addLine(String.format("%d. Battery voltage: %.1f volts", iTeleCt++, volts));

        //TODO: Initialize any essential starting motor/servo positions here

        telemetry.addData("Status: ", "Building Trajectories......");
        telemetry.update();

        //TODO: Build trajectories here
        telemetry.update();

        // Initialize TFOD and report what's detected until start is pushed
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        buildPurpleTrajectories();
        buildYellowTrajectories();
        buildParkTrajectories();

        while (!isStarted()) {
            telemetryTfod();
        }

        runtime.reset();
        if (opModeIsActive()) {
            DetectPurpleDropoffPos();
            visionPortal.close();

            //TODO: Run Trajectories
            switch (PurpleDropOffPos) {
                case 1:
                    // Go to position 2
                    Actions.runBlocking( new SequentialAction(
                            dropOffPurpleAtPos1,
                            updateAfterGatePos
                    ));
                    if(bProceedToYellow)
                        Actions.runBlocking( new SequentialAction( dropOffYellowAtPos1 ));
                    else
                        Actions.runBlocking( new SequentialAction( dropOffYellowAtPark ));
                    break;
                case 2:
                    // Go to position 2
                    Actions.runBlocking( new SequentialAction(
                                dropOffPurpleAtPos2,
                            updateAfterGatePos
                            ));
                    if(bProceedToYellow)
                        Actions.runBlocking( new SequentialAction( dropOffYellowAtPos2 ));
                    else
                        Actions.runBlocking( new SequentialAction( dropOffYellowAtPark ));
                    break;
                case 3:
                default:
                    // Go to position 2
                    Actions.runBlocking( new SequentialAction(
                            dropOffPurpleAtPos3,
                            updateAfterGatePos
                    ));
                    if(bProceedToYellow)
                        Actions.runBlocking( new SequentialAction( dropOffYellowAtPos3 ));
                    else
                        Actions.runBlocking( new SequentialAction( dropOffYellowAtPark ));
                    break;
            }
        }
    }

    void buildPurpleTrajectories()
    {
        dropOffPurpleAtPos1 = BlueLong.actionBuilder(StartPos)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(Purple1Pos, Math.toRadians(30))
                .stopAndAdd(ejectPurple)
                .waitSeconds(0.8)
                .setReversed(false)
                .lineToY(-2)
                .strafeTo(BeforeGatePos1)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(AfterGatePos)
                .build();

        dropOffPurpleAtPos2 = BlueLong.actionBuilder(StartPos)
                .setTangent(Math.toRadians(-70))
                .splineToLinearHeading(Purple2Pos, Math.toRadians(0))
                .stopAndAdd(ejectPurple)
                .waitSeconds(0.8)
                .setReversed(false)
                .lineToY(-18)
                .strafeTo(BeforeGatePos2)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(AfterGatePos)
                .build();

        dropOffPurpleAtPos3 = BlueLong.actionBuilder(StartPos)
                .setTangent(Math.toRadians(-70))
                .splineToLinearHeading(Purple3Pos, Math.toRadians(0))
                .stopAndAdd(ejectPurple)
                .waitSeconds(0.8)
                .setReversed(false)
                .lineToY(-21)
                .strafeTo(BeforeGatePos3)
                .stopAndAdd(rectifyHeadingError)
                .strafeTo(AfterGatePos)
                .build();
    }

    void buildYellowTrajectories()
    {
        dropOffYellowAtPos1 = BlueLong.actionBuilder(new Pose2d(AfterGatePos.x,AfterGatePos.y, Math.toRadians(90)))
                .setReversed(false)
                .splineToLinearHeading(Yellow3Pos, Math.toRadians(90))
                .strafeTo(new Vector2d(Yellow1Pos.component1().x,Yellow1Pos.component1().y))
                .stopAndAdd(depositYellow)
                .waitSeconds(3)
                .strafeTo(ParkPos)
                .build();

        dropOffYellowAtPos2 = BlueLong.actionBuilder(new Pose2d(AfterGatePos.x,AfterGatePos.y, Math.toRadians(90)))
                .setReversed(false)
                .splineToLinearHeading(Yellow3Pos, Math.toRadians(90))
                .strafeTo(new Vector2d(Yellow2Pos.component1().x,Yellow2Pos.component1().y))
                .stopAndAdd(depositYellow)
                .waitSeconds(3)
                .strafeTo(ParkPos)
                .build();

        dropOffYellowAtPos3 = BlueLong.actionBuilder(new Pose2d(AfterGatePos.x,AfterGatePos.y, Math.toRadians(90)))
                .setReversed(false)
                .splineToLinearHeading(Yellow3Pos, Math.toRadians(90))
                .stopAndAdd(depositYellow)
                .waitSeconds(3)
                .strafeTo(ParkPos)
                .build();
    }

    void buildParkTrajectories()
    {
        dropOffYellowAtPark = BlueLong.actionBuilder(new Pose2d(AfterGatePos.x, AfterGatePos.y, Math.toRadians(90)))
                .strafeTo(ParkPos)
                .stopAndAdd(depositYellow)
                .build();
    }



//TODO: Use April Tags to get current pos
//TODO: write trajectories as different functions

    //TODO: add any motors/servos that initialized later
    void initMotorsAndServos(boolean run_to_position)
    {
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

    //TODO: April Tag detection function - might need updating
//TODO: TFOD functions here
    //TFOD ConceptTensorFlowObjectDetectionEasy functions
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

    }   // end method telemetryTfod()

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
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
        }
        if(targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.fieldPosition);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);

            double deltaY = (desiredTag.ftcPose.range * Math.cos(desiredTag.ftcPose.bearing) -
                    RangeCalibrated          * Math.cos(BearingCalibrated));

            double deltaX = (desiredTag.ftcPose.range * Math.sin(desiredTag.ftcPose.bearing)) -
                    (RangeCalibrated          * Math.sin(BearingCalibrated));

            double deltaHeading = desiredTag.ftcPose.yaw - YawCalibrated;

            BlueLong.pose = new Pose2d(AfterGatePos.x + deltaX, AfterGatePos.y + deltaY, AfterGateHeading + deltaHeading);

            return true;
        }
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
            if(CAMERA_SIDE) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Eyeball"))
                        .addProcessor(aprilTag)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Kemera"))
                        .addProcessor(aprilTag)
                        .build();
            }
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

//        //TODO REmove this override
//        PurpleDropOffPos = 1;

        telemetry.addData("Detected Spike Mark X = ", x);
        telemetry.addData("Detected Drop off Position = ", PurpleDropOffPos);

        // Push telemetry to the Driver Station.
        telemetry.update();

    }

}





