package org.firstinspires.ftc.teamcode.World;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.ioState.STATE_1_RFI;
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.ioState.STATE_3_PS;
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.ioState.STATE_5_RFO_LOW;
import static org.firstinspires.ftc.teamcode.AuraRobot.APRILTAG_TIMEOUT;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_FOR_OUTTAKE;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_FOR_STACK_INTAKE;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_FOR_YELLOW_DROP;
import static org.firstinspires.ftc.teamcode.AuraRobot.AUTO_WAIT_RETURN_TO_INTAKE;
import static org.firstinspires.ftc.teamcode.AuraRobot.LEFT_FINGER_UNLOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.PURPLE_UNLOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.RIGHT_FINGER_UNLOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.leftLinkageClose;
import static org.firstinspires.ftc.teamcode.AuraRobot.leftLinkageOpen;
import static org.firstinspires.ftc.teamcode.AuraRobot.rightLinkageClose;
import static org.firstinspires.ftc.teamcode.AuraRobot.rightLinkageOpen;

import android.util.Size;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController;
import org.firstinspires.ftc.teamcode.AuraRobot;
import org.firstinspires.ftc.teamcode.Roadrunner.roadrunnerbasics.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Aura_AutoBot_Actions {
    AuraRobot Aurelius;
    MecanumDrive mecanumDrive;
    AuraIntakeOuttakeController myIntakeOuttakeController;

    public AprilTagDetection tagOfInterest = null;
    private static final String TFOD_MODEL_ASSET = "myBloopy.tflite";
    private static final String[] LABELS = {
            "Pixel",
            "Bloopy",
            "Redpy"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortalEyeball;
    private VisionPortal visionPortalKemera;
    private static final boolean USE_WEBCAM = true;
    public static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagProcessor aprilTagEyeball;              // Used for managing the AprilTag detection process.
    private AprilTagProcessor aprilTagKemera;
    private org.firstinspires.ftc.vision.apriltag.AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    MultipleTelemetry telemetry;

    public Aura_AutoBot_Actions(AuraRobot aurelius, MecanumDrive MecanumDrive, MultipleTelemetry Telemetry) {
        mecanumDrive = MecanumDrive;
        Aurelius = aurelius;
        telemetry = Telemetry;
        myIntakeOuttakeController = new AuraIntakeOuttakeController(hardwareMap, false);
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
        visionPortalEyeball = builder.build();

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
            while (!isStopRequested() && (visionPortalKemera.getCameraState() != VisionPortal.CameraState.STREAMING))
                sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortalKemera.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortalKemera.getCameraControl(GainControl.class);
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

            telemetry.addData("Current pos:", "X: %5.1f Y: %5.1f Heading: %5.1f degrees", BlueLong.pose.position.x, BlueLong.pose.position.y, Math.toDegrees(BlueLong.pose.heading.log()));
            telemetry.update();

            mecanumDrive.pose = new Pose2d(currX, currY, currHeading);
            telemetry.addData("Updated pos:", "X: %5.1f Y: %5.1f Heading %5.1f degrees", BlueLong.pose.position.x, BlueLong.pose.position.y, Math.toDegrees(BlueLong.pose.heading.log()));
            telemetry.update();
            return true;
        }
        telemetry.addLine("Not Found: Desired Tag not found");
        telemetry.update();
        return false;
    }
    public class purpleDumper implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {
            Aurelius.PurpleDumper.setPosition(PURPLE_UNLOCK);
            return false;
        }
    }
    public class YellowDropper implements Action {
        @Override
        public boolean run (TelemetryPacket tPkt) {
            myIntakeOuttakeController.RightFinger.setPosition(RIGHT_FINGER_UNLOCK);
            return false;
        }
    }
    public class WhiteDropper implements Action {
        @Override
        public boolean run (TelemetryPacket tPkt) {
            myIntakeOuttakeController.LeftFinger.setPosition(LEFT_FINGER_UNLOCK);
            return false;
        }
    }
    public class GotoOuttakeAction implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {
            myIntakeOuttakeController.setTargetState(STATE_5_RFO_LOW);
            return false;
        }
    }
    public class GotoIntakeAction implements Action {
        @Override
        public boolean run(TelemetryPacket tPkt) {
            myIntakeOuttakeController.setTargetState(STATE_1_RFI);
            return false;
        }
    }
    public class IntakeOuttakeControllerAction implements Action {
        public boolean run(TelemetryPacket tPkt) {
            myIntakeOuttakeController.update();
            return bRunningTrajectory;
        }
    }
    public class beginTrajectory implements Action {
        public boolean run(TelemetryPacket tPkt) {
            bRunningTrajectory=true;
            return false;
        }
    }
    public class endTrajectory implements Action {
        public boolean run(TelemetryPacket tPkt) {
            bRunningTrajectory=false;
            return false;
        }
    }
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
    public class initAprilTagEyeball implements Action {
        public boolean run(TelemetryPacket tPkt){
            initAprilTagEyeball();
            return false;
        }
    }
    public class initAprilTagKemera implements Action {
        public boolean run(TelemetryPacket tPkt){
            initAprilTagKemera();
            return false;
        }
    }
    public class lowerStackLinkage implements Action {
        public boolean run(TelemetryPacket tPkt){
            Aurelius.LeftLink.setPosition(leftLinkageOpen);
            Aurelius.RightLink.setPosition(rightLinkageOpen);
            Aurelius.Roller.setPower(-0.7);
            return false;
        }
    }
    public class raiseStackLinkage implements Action {
        public boolean run(TelemetryPacket tPkt){
            Aurelius.LeftLink.setPosition(leftLinkageClose);
            Aurelius.RightLink.setPosition(rightLinkageClose);
            Aurelius.Roller.setPower(0);
            return false;
        }
    }
    public class stackIntake implements Action {
        public boolean run(TelemetryPacket tPkt){
            Aurelius.setPower(AuraRobot.AuraMotors.INTAKE, 0.7);
            return false;
        }
    }
    public class ReverseIntake implements Action {
        public boolean run(TelemetryPacket tPkt){
            Aurelius.setPower(AuraRobot.AuraMotors.INTAKE, -.7);
            return false;
        }
    }
    public class raiseBox implements Action {
        public boolean run(TelemetryPacket tPkt){
            Aurelius.setPower(AuraRobot.AuraMotors.INTAKE, 0);
            myIntakeOuttakeController.setTargetState(STATE_3_PS);
            return false;
        }
    }

}
