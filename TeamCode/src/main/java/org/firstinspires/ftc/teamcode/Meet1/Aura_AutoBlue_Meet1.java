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

package org.firstinspires.ftc.teamcode.Meet1;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.AuraRobot;
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

@Autonomous(name="Aura_Auto", group="Linear OpMode")
@Disabled
public class Aura_AutoBlue_Meet1 extends LinearOpMode {

    private static final double LEFT_SPIKEMARK_BOUNDARY_X = 0;
    private static final double RIGHT_SPIKEMARK_BOUNDARY_X = 0;

    public static int PurpleDropOffPos = 0;

    AuraRobot Aurelius = new AuraRobot();

    private static FtcDashboard auraBoard;

    //TODO: declare April Tag stuff
    OpenCvWebcam Sauron = null;
    public AprilTagDetectionPipeline pipeline;

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

//TODO: Declare Trajectories Below

    private DcMotor Upper_Right = null;
    private DcMotor Upper_Left = null;
    private DcMotor Lower_Right = null;
    private DcMotor Lower_Left = null;
    //private DcMotor intakeMotor = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


//TODO: declare the variable that will store the outcome for detection (already made, just uncomment)

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

        telemetry.addData("Status: ", "Initializing camera......");
        telemetry.update();

        //TODO: Add TFOD detection here
        initTfod();

        Upper_Right = hardwareMap.get(DcMotor.class, "Upper_Right");
        Upper_Left = hardwareMap.get(DcMotor.class, "Upper_Left");
        Lower_Right = hardwareMap.get(DcMotor.class, "Lower_Right");
        Lower_Left = hardwareMap.get(DcMotor.class, "Lower_Left");

        Upper_Right.setDirection(DcMotor.Direction.FORWARD);
        Lower_Right.setDirection(DcMotor.Direction.FORWARD);
        Upper_Left.setDirection(DcMotor.Direction.REVERSE);
        Lower_Left.setDirection(DcMotor.Direction.REVERSE);

        Upper_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lower_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Upper_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lower_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (!isStarted()) {
            telemetryTfod();
        }

        if (opModeIsActive()) {
            DetectPurpleDropoffPos();
            visionPortal.close();
            //TODO: Run Trajectories

            switch (PurpleDropOffPos) {
                case 1:
                    encoderStraightDrive(DRIVE_SPEED, -36, -36, 5);
                    encoderStraightDrive(DRIVE_SPEED, 4, 4, 5);
                    //intakeMotor.setPower(0.4);
//                    encoderStraightDrive(DRIVE_SPEED, -24, -24, 5.0);  // forward 24, for 5s?
//                    encoderStraightDrive(TURN_SPEED, 3, -3, 4.0);  // turn left 45 degrees, for 4s?
//                    //spit out purple pixel
//                    encoderStraightDrive(TURN_SPEED, -3, 3, 4.0);  // turn right 45 degrees, for 4s?
//                    encoderStraightDrive(DRIVE_SPEED, 20, 20, 5.0);  // backward 20, for 5s?
//                    encoderStraightDrive(TURN_SPEED, 6, -6, 4.0);  // turn left 90 degrees, for 4s?
//                    encoderStraightDrive(DRIVE_SPEED, -48, -48, 5.0);  // forward 48, for 5s?
                    break;
                case 2:
                    encoderStraightDrive(DRIVE_SPEED, -36, -36, 5);
                    encoderStraightDrive(DRIVE_SPEED, 4, 4, 5);
                    //intakeMotor.setPower(0.4);
//                    encoderStraightDrive(DRIVE_SPEED, -30, -30, 5.0);  // forward 30, for 5s?
//                    //spit out purple pixel
//                    encoderStraightDrive(DRIVE_SPEED, 26, 26, 5.0);  // forward 30, for 5s?
//                    encoderStraightDrive(TURN_SPEED, 6, -6, 4.0);  // turn left 90 degrees, for 4s?
//                    encoderStraightDrive(DRIVE_SPEED, -48, -48, 5.0);  // forward 48, for 5s?
                    break;
                case 3:
                    encoderStraightDrive(DRIVE_SPEED, -36, -36, 5);
                    encoderStraightDrive(DRIVE_SPEED, 4, 4, 5);
                    //.setPower(0.4);
//                    encoderStraightDrive(DRIVE_SPEED, -24, -24, 5.0);  // forward 24, for 5s?
//                    encoderStraightDrive(TURN_SPEED, -3, 3, 4.0);  // S2: turn right 45 degrees, for 4s?
//                    //spit out purple pixel
//                    encoderStraightDrive(TURN_SPEED, 3, -3, 4.0);  // turn left 45 degrees, for 4s?
//                    encoderStraightDrive(DRIVE_SPEED, 20, 20, 5.0);  // backward 20, for 5s?
//                    encoderStraightDrive(TURN_SPEED, 6, -6, 4.0);  // turn left 90 degrees, for 4s?
//                    encoderStraightDrive(DRIVE_SPEED, -48, -48, 5.0);  // forward 48, for 5s?
                    break;
                default:
                    encoderStraightDrive(DRIVE_SPEED, -36, -36, 5);
                    encoderStraightDrive(DRIVE_SPEED, 4, 4, 5);
                    //intakeMotor.setPower(0.4);
//                    encoderStraightDrive(DRIVE_SPEED, -30, -30, 5.0);  // forward 30, for 5s?
//                    //spit out purple pixel
//                    encoderStraightDrive(DRIVE_SPEED, 26, 26, 5.0);  // forward 30, for 5s?
                    break;
            }
//            encoderStraightDrive(TURN_SPEED, 3, -3, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        }
    }


//TODO: Use April Tags to get current pos

//TODO: write trajectories as different functions

    //TODO: add any motors/servos that initialized later
    void initMotorsAndServos(boolean run_to_position)
    {
        // Reset Slides - current position becomes 0
//        Aura_Robot.setRunMode(CAT_MOUSE, STOP_AND_RESET_ENCODER);
//        Aura_Robot.setRunMode(CAT_MOUSE, RUN_WITHOUT_ENCODER);

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
//        .setModelFileName(TFOD_MODEL_FILE)
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
            if (recognition.getLabel() == "Bloopy") {
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                y = (recognition.getTop() + recognition.getBottom()) / 2;
            }
            break;
        }
        visionPortal.close();

        if( x < LEFT_SPIKEMARK_BOUNDARY_X )
            PurpleDropOffPos = 1;
        else if (x > RIGHT_SPIKEMARK_BOUNDARY_X)
            PurpleDropOffPos = 3;
        else
            PurpleDropOffPos = 2;

        telemetry.addData("Detected Drop off Position = ", PurpleDropOffPos);

        // Push telemetry to the Driver Station.
        telemetry.update();

    }

    public void encoderStraightDrive(double speed,
                                     double leftInches, double rightInches,
                                     double timeoutS) {
        int newUpperLeftTarget;
        int newUpperRightTarget;
        int newLowerLeftTarget;
        int newLowerRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newUpperLeftTarget = Upper_Left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newUpperRightTarget = Upper_Right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLowerLeftTarget = Lower_Left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLowerRightTarget = Lower_Right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            Upper_Left.setTargetPosition(newUpperLeftTarget);
            Upper_Right.setTargetPosition(newUpperRightTarget);
            Lower_Left.setTargetPosition(newLowerLeftTarget);
            Lower_Right.setTargetPosition(newLowerRightTarget);

            // Turn On RUN_TO_POSITION
            Upper_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Upper_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lower_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lower_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            Upper_Left.setPower(Math.abs(speed));
            Upper_Right.setPower(Math.abs(speed));
            Lower_Left.setPower(Math.abs(speed));
            Lower_Right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Upper_Left.isBusy() && Upper_Right.isBusy() && Lower_Left.isBusy() && Lower_Right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newUpperLeftTarget,  newUpperRightTarget, newLowerLeftTarget, newLowerRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        Upper_Left.getCurrentPosition(), Upper_Right.getCurrentPosition(), Lower_Left.getCurrentPosition(), Lower_Right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            Upper_Left.setPower(0);
            Upper_Right.setPower(0);
            Lower_Left.setPower(0);
            Lower_Right.setPower(0);

            // Turn off RUN_TO_POSITION
            Upper_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Upper_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lower_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lower_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move.
        }
    }

    public void encoderStrafeDrive(double speed,
                                     double leftDiagonalInches, double rightDiagonalInches,
                                     double timeoutS) {
//        guide for strafe
//                _________
//             L-| Forward |-R
//               |         |
//               |         |
//             R-|_________|-L

        int newUpperLeftTarget;
        int newUpperRightTarget;
        int newLowerLeftTarget;
        int newLowerRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newUpperLeftTarget = Upper_Left.getCurrentPosition() + (int)(leftDiagonalInches * COUNTS_PER_INCH);
            newUpperRightTarget = Upper_Right.getCurrentPosition() + (int)(rightDiagonalInches * COUNTS_PER_INCH);
            newLowerLeftTarget = Lower_Left.getCurrentPosition() + (int)(rightDiagonalInches * COUNTS_PER_INCH);
            newLowerRightTarget = Lower_Right.getCurrentPosition() + (int)(leftDiagonalInches * COUNTS_PER_INCH);
            Upper_Left.setTargetPosition(newUpperLeftTarget);
            Upper_Right.setTargetPosition(newUpperRightTarget);
            Lower_Left.setTargetPosition(newLowerLeftTarget);
            Lower_Right.setTargetPosition(newLowerRightTarget);

            // Turn On RUN_TO_POSITION
            Upper_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Upper_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lower_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lower_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            Upper_Left.setPower(Math.abs(speed));
            Upper_Right.setPower(Math.abs(speed));
            Lower_Left.setPower(Math.abs(speed));
            Lower_Right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Upper_Left.isBusy() && Upper_Right.isBusy() && Lower_Left.isBusy() && Lower_Right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newUpperLeftTarget,  newUpperRightTarget, newLowerLeftTarget, newLowerRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        Upper_Left.getCurrentPosition(), Upper_Right.getCurrentPosition(), Lower_Left.getCurrentPosition(), Lower_Right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            Upper_Left.setPower(0);
            Upper_Right.setPower(0);
            Lower_Left.setPower(0);
            Lower_Right.setPower(0);

            // Turn off RUN_TO_POSITION
            Upper_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Upper_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lower_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lower_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move.
        }
    }

}





