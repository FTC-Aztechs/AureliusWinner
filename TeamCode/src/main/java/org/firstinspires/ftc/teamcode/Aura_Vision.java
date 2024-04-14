package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp

public class Aura_Vision extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        int[] portalsList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        AprilTagProcessor tagProcessorEyeball = new AprilTagProcessor.Builder()
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortalEyeball = new VisionPortal.Builder()
                .addProcessors(tagProcessorEyeball)
                .setCamera(hardwareMap.get(WebcamName.class, "Eyeball"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[0])
                .build();
        AprilTagProcessor tagProcessorSauron = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1385.92f , 1385.92f, 951.982f , 534.084f)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortalSauron = new VisionPortal.Builder()
                .addProcessors(tagProcessorSauron)
                .setCamera(hardwareMap.get(WebcamName.class, "Sauron"))
                .setCameraResolution(new Size(1920, 1080))
                .enableLiveView(true)
                .setLiveViewContainerId(portalsList[1])
                .build();
        waitForStart();
//        FtcDashboard.getInstance().startCameraStream((CameraStreamSource) tagProcessor,0);
        visionPortalEyeball.resumeStreaming();
        visionPortalSauron.resumeStreaming();


        while(!isStopRequested() && opModeIsActive()) {
            //Eyeball
                if(tagProcessorEyeball.getDetections().size() > 0) {
                    AprilTagDetection tagEyeball = tagProcessorEyeball.getDetections().get(0);
                    telemetry.addData("id", tagEyeball.id);
                    double rangeEyeball = tagEyeball.ftcPose.range;
                    double bearingEyeball = tagEyeball.ftcPose.bearing;
                    double yawEyeball = tagEyeball.ftcPose.yaw;

                    double robotOffsetX = -7;
                    double robotOffsetY = +5.5;

                    double offsetXEyeball = (rangeEyeball * Math.cos(Math.toRadians(bearingEyeball)));

                    double offsetYEyeball = (rangeEyeball * Math.sin(Math.toRadians(bearingEyeball)));

                    double currHeadingEyeball = -Math.toRadians(yawEyeball);

                    double rotateXEyeball = (robotOffsetX * Math.cos(currHeadingEyeball)) + (robotOffsetY * -Math.sin(currHeadingEyeball));
                    double rotateYEyeball = (robotOffsetX * Math.sin(currHeadingEyeball)) + (robotOffsetY * Math.cos(currHeadingEyeball));

                    double currXEyeball = rotateXEyeball + (tagEyeball.metadata.fieldPosition.getData()[0] -
                            offsetXEyeball);

                    double currYEyeball = rotateYEyeball + (tagEyeball.metadata.fieldPosition.getData()[1] -
                            offsetYEyeball);

                    telemetry.addData("Current pos:", "X: %5.1f Y: %5.1f Heading: %5.1f degrees", currXEyeball, currYEyeball, Math.toDegrees(currHeadingEyeball));
                    telemetry.addData("tagPos", "X: %5.1f Y: %5.1f", tagEyeball.metadata.fieldPosition.getData()[0],tagEyeball.metadata.fieldPosition.getData()[1]);
                    telemetry.addData("offset", "X: %5.1f Y: %5.1f",offsetXEyeball,offsetYEyeball);
                    telemetry.addData("yaw", tagEyeball.ftcPose.yaw);
                    telemetry.addData("bearing", tagEyeball.ftcPose.bearing);
                    telemetry.addData("range", tagEyeball.ftcPose.range);
                    telemetry.addData("x", tagEyeball.ftcPose.x);
                    telemetry.addData("y", tagEyeball.ftcPose.y);
                    telemetry.addData("z", tagEyeball.ftcPose.z);
                    telemetry.addData("roll", tagEyeball.ftcPose.roll);
                    telemetry.addData("pitch", tagEyeball.ftcPose.pitch);
                    telemetry.addData("yaw", tagEyeball.ftcPose.yaw);
                    telemetry.update();
                }
            //Sauron
                if(tagProcessorSauron.getDetections().size() > 0) {
                    AprilTagDetection tagSauron = tagProcessorSauron.getDetections().get(0);
                    telemetry.addData("id", tagSauron.id);
                    double rangeSauron = tagSauron.ftcPose.range;
                    double bearingSauron = tagSauron.ftcPose.bearing;
                    double yawSauron = tagSauron.ftcPose.yaw;

                    double robotOffsetXSauron = 8.5;
                    double robotOffsetYSauron = 0;

                    double offsetX = (rangeSauron * Math.cos(Math.toRadians(bearingSauron)));

                    double offsetY = (rangeSauron * Math.sin(Math.toRadians(bearingSauron)));

                    double currHeadingSauron = -Math.toRadians(yawSauron);

                    double rotateXSauron = (robotOffsetXSauron * Math.cos(currHeadingSauron)) + (robotOffsetYSauron * -Math.sin(currHeadingSauron));
                    double rotateYSauron = (robotOffsetXSauron * Math.sin(currHeadingSauron)) + (robotOffsetYSauron * Math.cos(currHeadingSauron));

                    double currXSauron = rotateXSauron + (tagSauron.metadata.fieldPosition.getData()[0] -
                            offsetX);

                    double currYSauron = rotateYSauron + (tagSauron.metadata.fieldPosition.getData()[1] -
                            offsetY);

                    telemetry.addData("Current pos:", "X: %5.1f Y: %5.1f Heading: %5.1f degrees", currXSauron, currYSauron, Math.toDegrees(currHeadingSauron));
                    telemetry.addData("tagPos", "X: %5.1f Y: %5.1f", tagSauron.metadata.fieldPosition.getData()[0],tagSauron.metadata.fieldPosition.getData()[1]);
                    telemetry.addData("offset", "X: %5.1f Y: %5.1f",offsetX,offsetY);
                    telemetry.addData("yaw", tagSauron.ftcPose.yaw);
                    telemetry.addData("bearing", tagSauron.ftcPose.bearing);
                    telemetry.addData("range", tagSauron.ftcPose.range);
                    telemetry.addData("x", tagSauron.ftcPose.x);
                    telemetry.addData("y", tagSauron.ftcPose.y);
                    telemetry.addData("z", tagSauron.ftcPose.z);
                    telemetry.addData("roll", tagSauron.ftcPose.roll);
                    telemetry.addData("pitch", tagSauron.ftcPose.pitch);
                    telemetry.addData("yaw", tagSauron.ftcPose.yaw);
                    telemetry.update();
                }
            telemetry.update();
        }
    }

}
