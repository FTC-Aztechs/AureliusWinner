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
 * SERVICES; LOSS OF USE, Line, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.AuraHangController.HangState.Hang;
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.targetSlidePos;
import static org.firstinspires.ftc.teamcode.AuraRobot.AuraMotors.HANG;
import static org.firstinspires.ftc.teamcode.AuraRobot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.AuraRobot.LEFT_FINGER_LOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.LEFT_FINGER_UNLOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.RIGHT_FINGER_LOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.RIGHT_FINGER_UNLOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_INTAKE_POS;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_RAISE_HIGH;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_FLIP_HEIGHT;
import static org.firstinspires.ftc.teamcode.AuraRobot.bumperSpeedAdjust;
import static org.firstinspires.ftc.teamcode.AuraRobot.dPadSpeedAdjust;
import static org.firstinspires.ftc.teamcode.AuraRobot.slideTicks_stepSize;
import static org.firstinspires.ftc.teamcode.AuraRobot.speedAdjust;

import android.graphics.Color;
import android.transition.Slide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="Aura_Manual", group="Manual mode")

public class Aura_Manual extends LinearOpMode {

    // Declare OpMode members.
    AuraRobot Aurelius = new AuraRobot();

    private boolean changingWheelSpeed = false; //truer than false;
    private boolean changingIntakeSpeed = false;
    private boolean changingLauncherSpeed = false;

    private boolean changingState = false;

    public RevColorSensorV3 Left;

    public ColorRangeSensor Right;

    private int slide_currentPos = 0;
    private int slide_newPos = slide_currentPos;


    public static int Mode = 1;
//    public static int BUTTON_TRIGGER_TIMER_MS = 500;

    boolean PlaneLaunched = false;

    //    private static ElapsedTime timer_gp1_buttonA;
//    private static ElapsedTime timer_gp1_buttonX;
//    private static ElapsedTime timer_gp1_buttonY;
//    private static ElapsedTime timer_gp1_buttonB;
//    private static ElapsedTime timer_gp1_dpad_up;
//    private static ElapsedTime timer_gp1_dpad_down;
    private static ElapsedTime timer_gp2_buttonA = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonX = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonY = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonB = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp1_dpad_left = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp1_dpad_right = new ElapsedTime(MILLISECONDS);


    private static ElapsedTime timer_gp2_x = new ElapsedTime(MILLISECONDS);

    private static ElapsedTime timer_gp2_y = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_a = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_b = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_up = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_down = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_rb = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_rt = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_lt = new ElapsedTime(MILLISECONDS);
    //    private static ElapsedTime timer_gp2_buttonA = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonX = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonY = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonB = new ElapsedTime(MILLISECONDS);
//    ;
//    private static ElapsedTime timer_gp2_dpad_up = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_dpad_down = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_left = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_right = new ElapsedTime(MILLISECONDS);

    private static ElapsedTime timer_gp1_left_bumper = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp1_left_trigger = new ElapsedTime(MILLISECONDS);

    //slide button booleans
    private boolean assumingHighPosition = false;
    private boolean assumingMidPosition = false;
    private boolean assumingLowPosition = false;
    private boolean assumingFloorPosition = false;
    private boolean assumingTopMidCone = false;
    private boolean assumingMiddleCone = false;

    // Define Fingers
    private Servo LeftFinger = null;
    private Servo RightFinger = null;


    //drive booleans
    private boolean changing_drive_mode = false;
    private boolean fieldCentric = false;

    AuraIntakeOuttakeController myIntakeOuttakeController;

    FtcDashboard auraDashboard;


    @Override
    public void runOpMode() {
        // Initialize the drive system vriables
        Aurelius.init(hardwareMap);
        LeftFinger = hardwareMap.get(Servo.class, "lefty");
        RightFinger = hardwareMap.get(Servo.class, "righty");


        myIntakeOuttakeController = new AuraIntakeOuttakeController (hardwareMap, true);
        initAurelius();

        while (!isStarted()){
            myIntakeOuttakeController.update();
        }

        while (opModeIsActive()) {
            AuraIntakeRoller();
            AuraIntakeOuttake();
            AuraManualDrive();
            AuraLauncher();
            AuraFingers();
            AuraHang();
            //AuraHang();//            AuraColor();

        }
    }




    public void AuraFingers()
    {
        if(gamepad2.left_trigger == 1f) {
            if(!changingState) {
                timer_gp2_lt.reset();
                changingState = true;
            } else if (timer_gp2_lt.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                LeftFinger.setPosition(LEFT_FINGER_UNLOCK);
                changingState = false;
            }
        }
        if(gamepad2.right_trigger == 1f) {
            if(!changingState) {
                timer_gp2_rt.reset();
                changingState = true;
            } else if (timer_gp2_lt.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                RightFinger.setPosition(RIGHT_FINGER_UNLOCK);
                changingState = false;
            }
        }
    }

    public void initAurelius() {
        FtcDashboard Dash = auraDashboard;

        Aurelius.boeing747.init();

        Aurelius.hanger.init();
        Aurelius.hanger.update();

        myIntakeOuttakeController.init();
        myIntakeOuttakeController.setTargetState(AuraIntakeOuttakeController.ioState.STATE_1_RFI);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Status: Robot is ready to roll!");
        telemetry.update();
    }



    public void AuraManualDrive() {
        // changing the speed
        if (gamepad1.dpad_left) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_left.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (dPadSpeedAdjust <= 1) {
                    dPadSpeedAdjust = 1;
                } else {
                    dPadSpeedAdjust -= 1;
                }
                telemetry.addLine("Current speed: " + dPadSpeedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        //gamepad right -> increase wheel speed
        if (gamepad1.dpad_right) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_right.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (dPadSpeedAdjust >= 10) {
                    dPadSpeedAdjust = 10;
                } else {
                    dPadSpeedAdjust += 1;
                }
                telemetry.addLine("Current speed: " + dPadSpeedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        //bumper speed boost mode
        if (gamepad1.right_bumper) {
            speedAdjust = bumperSpeedAdjust;
        } else {
            speedAdjust = dPadSpeedAdjust;
        }

        // actually making the robot move
        float turnDir = gamepad1.right_stick_x;
        float moveDir = gamepad1.left_stick_y;
        float strafeDir = gamepad1.left_stick_x;

        if (turnDir > 1) {
            turnDir = 1;
        } else if (turnDir < -1) {
            turnDir = -1;
        }
        Aurelius.Lower_Left.setPower((moveDir + strafeDir - turnDir) * (-speedAdjust / 10)); // 1.0
        Aurelius.Lower_Right.setPower((moveDir - strafeDir + turnDir) * (-speedAdjust / 10)); // 1.0
        Aurelius.Upper_Left.setPower((moveDir - strafeDir - turnDir) * (-speedAdjust / 10)); // 0
        Aurelius.Upper_Right.setPower((moveDir + strafeDir + turnDir) * (-speedAdjust / 10)); // 0
    }

    public void AuraHang() {
        if(gamepad2.dpad_left) {
            if (!changingState) {
                timer_gp2_dpad_left.reset();
                changingState = true;
            } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                Aurelius.hanger.setTargetState(AuraHangController.HangState.Idle);
                telemetry.addData("State ", " Idle");
                changingState = false;
            }
        }
        if (gamepad2.dpad_up) {
            if (!changingState) {
                timer_gp2_dpad_up.reset();
                changingState = true;
            } else if (timer_gp2_dpad_up.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                Aurelius.hanger.setTargetState(AuraHangController.HangState.Up);
                telemetry.addData("State ", " Up");
                changingState = false;
            }
        }
        if (gamepad2.dpad_down) {
            if (!changingState) {
                timer_gp2_dpad_down.reset();
                changingState = true;
            } else if (timer_gp2_dpad_down.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                Aurelius.hanger.setTargetState(AuraHangController.HangState.Hang);
                telemetry.addData("State ", " Hang");
                changingState = false;
            }
        }
        Aurelius.hanger.update();
    }

    public void AuraIntakeRoller() {

        Aurelius.setPower(AuraRobot.AuraMotors.INTAKE,(-gamepad2.right_stick_y));
        Aurelius.setPower(AuraRobot.AuraMotors.ROLLER, (gamepad2.right_stick_y));

    }

//    @SuppressLint("SuspiciousIndentation")
public void AuraIntakeOuttake() {


    if (gamepad2.a) {
        if (!changingState) {
            timer_gp2_a.reset();
            changingState = true;
        } else if (timer_gp2_a.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
            myIntakeOuttakeController.setTargetState(AuraIntakeOuttakeController.ioState.STATE_5_RFO_MANUAL);
            telemetry.addData("State", "going to outtake");
            telemetry.update();
            changingState = false;
        }
    }

    if (gamepad2.b) {
        if (!changingState) {
            timer_gp2_b.reset();
            changingState = true;
        } else if (timer_gp2_b.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
            myIntakeOuttakeController.goingUp = false;
            myIntakeOuttakeController.setTargetState(AuraIntakeOuttakeController.ioState.STATE_1_RFI);
            telemetry.addData("State", "going to intake");
            telemetry.update();

        }
    }

    if(myIntakeOuttakeController.currState == AuraIntakeOuttakeController.ioState.STATE_5_RFO_MANUAL){
        double target = targetSlidePos + (int) (-gamepad2.left_stick_y * slideTicks_stepSize);
        if (target >= SLIDE_RAISE_HIGH) {
            target = SLIDE_RAISE_HIGH;
        } else if (target < SLIDE_INTAKE_POS) {
            target = SLIDE_INTAKE_POS;
            telemetry.addData("TargetSlidePos: ", targetSlidePos);
            telemetry.update();
        }
        myIntakeOuttakeController.setTargetPosition(target);
    }

    myIntakeOuttakeController.update();
}

    public void AuraLauncher(){
        if (gamepad1.left_trigger == 1f) {
            if (!changingLauncherSpeed) {
                timer_gp1_left_trigger.reset();
                changingLauncherSpeed = true;
            } else if (timer_gp1_left_trigger.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (PlaneLaunched == false) {
                    Aurelius.boeing747.setTargetState(AuraLaunchController.launchState.Launch);
                    Aurelius.boeing747.update();
                    PlaneLaunched = true;
                } else {
                    Aurelius.boeing747.setTargetState(AuraLaunchController.launchState.Set);
                    Aurelius.boeing747.update();
                    PlaneLaunched = false;
                }
                telemetry.addLine("Current State:" + Aurelius.boeing747.currState);
                telemetry.update();
                changingLauncherSpeed = false;
            }
        }

    }
}