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
import static org.firstinspires.ftc.teamcode.AuraRobot.AuraMotors.INTAKE;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.AuraHeadingEstimator;
import org.firstinspires.ftc.teamcode.AuraRobot;
import org.firstinspires.ftc.teamcode.Roadrunner.roadrunnerbasics.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

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
@Autonomous(name="Red_Short4", group="Linear OpMode")
@Disabled

public class Aura_AutoRed_Short_Meet4 extends LinearOpMode {

    //**** Roadrunner Pose2ds ****

    Pose2d StartPos = new Pose2d(0,0,0);

    Pose2d Purple1Pos = new Pose2d(28, 2, Math.toRadians(90));
    Pose2d Purple2Pos = new Pose2d(36, -12, Math.toRadians(90));
    Pose2d Purple3Pos = new Pose2d(28, -19, Math.toRadians(90));

    Pose2d Yellow3Pos = new Pose2d(20, -37, Math.toRadians(90));
    Pose2d Yellow2Pos = new Pose2d(28, -37, Math.toRadians(90));
    Pose2d Yellow1Pos = new Pose2d(33,-37, Math.toRadians(90));

    Vector2d ParkPos = new Vector2d(7, -37);


    private static final double LEFT_SPIKEMARK_BOUNDARY_X = 250;
    private static final double RIGHT_SPIKEMARK_BOUNDARY_X = 260;

    public static double SplineAngle = 90;
    public static int PurpleDropOffPos = 0;

    AuraRobot Aurelius = new AuraRobot();
    MecanumDrive RedShort;
    public AuraHeadingEstimator myHeadingEstimator;


    private static FtcDashboard auraBoard;

    //TODO: imu
    public class IMUController implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {

            double oldHeading =RedShort.pose.heading.log();
            telemetry.addData("Old heading", Math.toDegrees(oldHeading));
            double yaw = myHeadingEstimator.getYaw();
            telemetry.addData("IMU Heading correction: ", Math.toDegrees(yaw - oldHeading));
            telemetry.addData("Corrected heading:", Math.toDegrees(yaw));
            telemetry.update();

           RedShort.pose = new Pose2d(RedShort.pose.position.x,RedShort.pose.position.y, yaw);

            return false;
        }
    }

    public Action rectifyHeadingError = new IMUController();

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
    private static final String TFOD_MODEL_ASSET = "myRedpy.tflite";

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

    // TODO: define trajectory variables here
    // Purple Trajectories
    private Action trajPos3Purple;
    private Action trajPos2Purple;
    private Action trajPos1Purple;

    // Yellow Trajectories
    private Action trajPos3Yellow;
    private Action trajPos2Yellow;
    private Action trajPos1Yellow;

    // Park Trajectories
    private Action trajPos3ToPark;
    private Action trajPos2ToPark;
    private Action trajPos1ToPark;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: Assume this will be our Auto. The pseudo code below is for camera detection
        //   Option 1: Use TFOD - in this case, we simply use the ConceptTFod detector and extend it with our trained model
        //   Option 2: Develop our own OpenCV based image processor
        //               1. Implement a new VisionProcessor (kemmaProcessor)
        //                  - Implement init, processFrame and onDrawFrame methods - look at the AprilTagProcessorImpl and TfodProcesorImpl for examples.
        //                  - ProcessFrame needs to have the algorithm to detect the black (color of team element) pixels in the rectangle
        //               2. Use the VisionPortal pattern to implement camera detection (see AprilTag and tFodProcessor examples)
        //               3. Register the kemmaProcessor with VisionPortal
        //               4. Implement a method on kemmaProcessor to return detected position based on which of the 3 rectangles returns most positive
        //   Option 3: Ditch the VisionProcessor and use EasyOpenCV directly

        Aurelius.init(hardwareMap);
        RedShort = new MecanumDrive(Aurelius.hwMap, new Pose2d(0,0,Math.toRadians(0)));
        myHeadingEstimator = new AuraHeadingEstimator(Aurelius.hwMap, StartPos);
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
                    Actions.runBlocking(
                            new SequentialAction(
                                    trajPos1Purple,
                                    new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket tPkt) {
                                            dropOffPurplePixel();
                                            return false;
                                        }
                                    },
                                    trajPos1Yellow,
                                    new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket tPkt) {
                                            dropOffYellowPixel();
                                            return false;
                                        }
                                    }
                                    , trajPos1ToPark
                            ));
                    break;
                case 2:
                    // Go to position 2
                    Actions.runBlocking(
                            new SequentialAction(
                                    trajPos2Purple,
                                    new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket tPkt) {
                                            dropOffPurplePixel();
                                            return false;
                                        }
                                    },
                                    trajPos2Yellow,
                                    new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket tPkt) {
                                            dropOffYellowPixel();
                                            return false;
                                        }
                                    },
                                    trajPos2ToPark
                            ));
                    break;
                case 3:
                default:
                    // Go to position 3
                    Actions.runBlocking(
                            new SequentialAction(
                                    trajPos3Purple,
                                    new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket tPkt) {
                                            dropOffPurplePixel();
                                            return false;
                                        }
                                    },
                                    trajPos3Yellow,
                                    new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket tPkt) {
                                            dropOffYellowPixel();
                                            return false;
                                        }
                                    },
                                    trajPos3ToPark
                            ));
                    break;
            }
        }
    }

    void buildPurpleTrajectories()
    {
        trajPos1Purple = RedShort.actionBuilder(StartPos)
                .setTangent(Math.toRadians(-80))
                .splineToLinearHeading(Purple1Pos, Math.toRadians(-130))
                .build();

        trajPos2Purple = RedShort.actionBuilder(StartPos)
                .setTangent(Math.toRadians(-70))
                .splineToLinearHeading(Purple2Pos, Math.toRadians(0))
                .build();

        trajPos3Purple = RedShort.actionBuilder(StartPos)
                .setTangent(Math.toRadians(-70))
                .splineToLinearHeading(Purple3Pos, Math.toRadians(0))
                .build();
    }

    void buildYellowTrajectories()
    {
        trajPos1Yellow = RedShort.actionBuilder(Purple1Pos)
                .stopAndAdd(rectifyHeadingError)
                .setReversed(true)
                .splineToLinearHeading(Yellow1Pos, Math.toRadians(-90))
                .build();

        trajPos2Yellow = RedShort.actionBuilder(Purple2Pos)
                .stopAndAdd(rectifyHeadingError)
                .setReversed(true)
                .splineToLinearHeading(Yellow2Pos, Math.toRadians(-90))
                .build();

        trajPos3Yellow = RedShort.actionBuilder(Purple3Pos)
                .stopAndAdd(rectifyHeadingError)
                .setReversed(true)
                .splineToLinearHeading(Yellow3Pos, Math.toRadians(-90))
                .build();
    }

    void buildParkTrajectories()
    {
        trajPos1ToPark = RedShort.actionBuilder(Yellow1Pos)
                .strafeTo(ParkPos)
                .build();

        trajPos2ToPark = RedShort.actionBuilder(Yellow2Pos)
                .strafeTo(ParkPos)
                .build();

        trajPos3ToPark = RedShort.actionBuilder(Yellow3Pos)
                .strafeTo(ParkPos)
                .build();
    }

    void dropOffPurplePixel()
    {
        runtime.reset();
        while(runtime.seconds() < 1.2) {
            Aurelius.setPower(INTAKE, -0.175);
        }
        Aurelius.setPower(INTAKE, 0);
    }

    void dropOffYellowPixel()
    {
 telemetry.addData("Deposit State", "down");
//        telemetry.update();
//
//        sleep(500);
//
//        Aurelius.depositFlipper.setTargetState(Up);
//        Aurelius.depositFlipper.update();
//        telemetry.addData("Deposit State", "up");
//        telemetry.update();
//
//        sleep(1500);
//
//        Aurelius.depositFlipper.setTargetState(Open);
//        Aurelius.depositFlipper.update();
//        telemetry.addData("Deposit State", "open");
//        telemetry.update();
//
//        sleep(500);
//
//        Aurelius.depositFlipper.setTargetState(Down);
//        Aurelius.depositFlipper.update();
//        telemetry.addData("Deposit State", "down");
//        telemetry.update();
//
//        sleep(500);
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






