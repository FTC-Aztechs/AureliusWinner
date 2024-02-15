package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp

public class Aura_Vision extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessors(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Kemera"))
                .setCameraResolution(new Size(640, 480))
                .build();
        waitForStart();
//        FtcDashboard.getInstance().startCameraStream((CameraStreamSource) tagProcessor,0);
        visionPortal.resumeStreaming();

        while(!isStopRequested() && opModeIsActive()) {
            if(tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("id", tag.id);
                double range = tag.ftcPose.range;
                double bearing = tag.ftcPose.bearing;
                double yaw = tag.ftcPose.yaw;

                double robotOffsetX = -7;
                double robotOffsetY = -2.25;

                double offsetX = (range * Math.cos(Math.toRadians(bearing)));

                double offsetY = (range * Math.sin(Math.toRadians(bearing)));

                double currHeading = -Math.toRadians(yaw);

                double rotateX = (robotOffsetX * Math.cos(currHeading)) + (robotOffsetY * -Math.sin(currHeading));
                double rotateY = (robotOffsetX * Math.sin(currHeading)) + (robotOffsetY * Math.cos(currHeading));

                double currX = rotateX + (tag.metadata.fieldPosition.getData()[0] -
                        offsetX);

                double currY = rotateY + (tag.metadata.fieldPosition.getData()[1] -
                        offsetY);

                telemetry.addData("Current pos:", "X: %5.1f Y: %5.1f Heading: %5.1f degrees", currX, currY, Math.toDegrees(currHeading));
                telemetry.addData("tagPos", "X: %5.1f Y: %5.1f", tag.metadata.fieldPosition.getData()[0],tag.metadata.fieldPosition.getData()[1]);
                telemetry.addData("offset", "X: %5.1f Y: %5.1f",offsetX,offsetY);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("bearing", tag.ftcPose.bearing);
                telemetry.addData("range", tag.ftcPose.range);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.update();
            }
            telemetry.update();
        }
    }
}
