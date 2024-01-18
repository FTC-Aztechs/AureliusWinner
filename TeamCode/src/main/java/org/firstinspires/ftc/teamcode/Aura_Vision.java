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
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessors(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Eyeball"))
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

                double currX = tag.metadata.fieldPosition.getData()[0] -
                        (tag.ftcPose.range * Math.sin(Math.toRadians(tag.ftcPose.bearing)));

                double currY = tag.metadata.fieldPosition.getData()[1] -
                        (tag.ftcPose.range * Math.cos(Math.toRadians(tag.ftcPose.bearing)));


                double currHeading = tag.ftcPose.yaw + tag.ftcPose.yaw;

                telemetry.addData("Current pos:", "X: %5.1f Y: %5.1f Heading: %5.1f degrees", currX, currY, currHeading);
                telemetry.update();
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("bearing", tag.ftcPose.bearing);
                telemetry.addData("range", tag.ftcPose.range);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
            }
            telemetry.update();
        }
    }
}
