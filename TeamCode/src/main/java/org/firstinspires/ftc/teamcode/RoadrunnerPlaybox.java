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

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunnerbasics.MecanumDrive;

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
@Autonomous(name="Playbox", group="Linear OpMode")

public class RoadrunnerPlaybox extends LinearOpMode {

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

public static double StartposX = 0;
public static double StartposY = 0;
public static double StartposHeading = 0;
public static int Distance = 12;

private static int iTeleCt = 1;



    AuraRobot Aurelius = new AuraRobot();
    MecanumDrive Playbox;

    private static FtcDashboard auraBoard;

    // TODO: define trajectory variables here
    // Purple Trajectories
    private Action squareTraj;
    private Action splineTraj;

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
        Playbox = new MecanumDrive(Aurelius.hwMap, new Pose2d(StartposX,StartposY,Math.toRadians(StartposHeading)));
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

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        buildTestTrajectory();
        waitForStart();

        runtime.reset();
        if (opModeIsActive()) {

    //TODO: Run Trajectories
            Actions.runBlocking(
                new SequentialAction(
                        squareTraj,
                        splineTraj
                ));

            }
        }

    void buildTestTrajectory()
    {
        squareTraj = Playbox.actionBuilder(new Pose2d(StartposX,StartposY,Math.toRadians(StartposHeading)))
                .strafeTo(new Vector2d(Distance + StartposX, 0 + StartposY))
                .strafeTo(new Vector2d(Distance + StartposX,Distance + StartposY))
                .setReversed(true)
                .strafeTo(new Vector2d(0 + StartposX, Distance + StartposY))
                .setReversed(false)
                .strafeTo(new Vector2d(0 + StartposX,0 + StartposY))
                .build();

        splineTraj = Playbox.actionBuilder(new Pose2d(StartposX,StartposY,Math.toRadians(StartposHeading)))
                .splineToLinearHeading(new Pose2d(Distance + StartposX,Distance + StartposY, Math.toRadians(90 + StartposHeading)),Math.toRadians(90 + StartposHeading))
                .splineToLinearHeading(new Pose2d(0 + StartposX,0 + StartposY, Math.toRadians(0 + StartposHeading)),Math.toRadians(90 + StartposHeading))
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

}






