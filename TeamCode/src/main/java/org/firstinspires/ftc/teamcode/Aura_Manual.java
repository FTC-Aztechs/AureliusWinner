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

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.AuraRobot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.AuraRobot.Launcher_Set_Pos;
import static org.firstinspires.ftc.teamcode.AuraRobot.Lid_Open_Pos;
import static org.firstinspires.ftc.teamcode.AuraRobot.bumperSpeedAdjust;
import static org.firstinspires.ftc.teamcode.AuraRobot.dPadIntakeAdjust;
import static org.firstinspires.ftc.teamcode.AuraRobot.dPadSpeedAdjust;
import static org.firstinspires.ftc.teamcode.AuraRobot.speedAdjust;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public RevColorSensorV3 Left = null;

    public ColorRangeSensor Right = null;



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
    private static ElapsedTime timer_gp2_dpad_up = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_down = new ElapsedTime(MILLISECONDS);
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


    //drive booleans
    private boolean changing_drive_mode = false;
    private boolean fieldCentric = false;


    FtcDashboard auraDashboard;


    @Override
    public void runOpMode() {
        // Initialize the drive system vriables
        Aurelius.init(hardwareMap);
        initAurelius();
        waitForStart();

        while (opModeIsActive()) {
//            AuraIntake();
//            AuraLauncher();
            AuraColor();
            AuraManualDrive();
//            AuraManualHang();
            //AuraDeposit();
            telemetry.addLine("Drive Mode: Forward Facing");
            telemetry.update();
        }
    }

    public void initAurelius() {
        FtcDashboard Dash = auraDashboard;
//        Aurelius.boeing747.launcher.setPosition(Launcher_Set_Pos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Left = hardwareMap.get(RevColorSensorV3.class, "Left");
        Right = hardwareMap.get(ColorRangeSensor.class,"Right");

        telemetry.addLine("Status: Robot is ready to roll!");
        telemetry.update();

        return;
    }

    public void AuraColor() {
        Left.initialize();
        Left.enableLed(true);
        Right.enableLed(true);
        Left.getLightDetected();


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

        return;
    }

    public void AuraManualHang() {
        if(gamepad2.x) {
            if (!changingState) {
                timer_gp2_x.reset();
                changingState = true;
            } else if (timer_gp2_x.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                Aurelius.hanger.setTargetState(AuraHangController.HangState.Idle);
                telemetry.addData("State ", " Idle");
                changingState = false;
            }
        }
        if (gamepad2.y) {
            if (!changingState) {
                timer_gp2_y.reset();
                changingState = true;
            } else if (timer_gp2_y.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                Aurelius.hanger.setTargetState(AuraHangController.HangState.Up);
                telemetry.addData("State ", " Up");
                changingState = false;
            }
        }
        if (gamepad2.a) {
            if (!changingState) {
                timer_gp2_a.reset();
                changingState = true;
            } else if (timer_gp2_a.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                Aurelius.hanger.setTargetState(AuraHangController.HangState.Hang);
                telemetry.addData("State ", " Hang");

                changingState = false;
            }
        }
        Aurelius.hanger.update();
    }

    public void AuraIntake() {
        if (gamepad2.dpad_left) {
            if (!changingIntakeSpeed) {
                timer_gp2_dpad_left.reset();
                changingIntakeSpeed = true;
            } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (dPadIntakeAdjust <= 1) {
                    dPadIntakeAdjust = 1;
                } else {
                    dPadIntakeAdjust -= 1;
                }
                telemetry.addLine("Current intake speed: " + dPadIntakeAdjust);
                telemetry.update();
                changingIntakeSpeed = false;
            }
        }

        //gamepad right -> increase wheel speed
        if (gamepad2.dpad_right) {
            if (!changingIntakeSpeed) {
                timer_gp2_dpad_right.reset();
                changingIntakeSpeed = true;
            } else if (timer_gp2_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (dPadIntakeAdjust >= 10) {
                    dPadIntakeAdjust = 10;
                } else {
                    dPadIntakeAdjust += 1;
                }
                telemetry.addLine("Current speed: " + dPadIntakeAdjust);
                telemetry.update();
                changingIntakeSpeed = false;
            }
        }

        Aurelius.setPower(AuraRobot.AuraMotors.INTAKE,(dPadIntakeAdjust/10)* gamepad2.right_stick_y);
        Aurelius.setPower(AuraRobot.AuraMotors.ROLLER, (dPadIntakeAdjust / 10) * gamepad2.right_stick_y);

    }

    @SuppressLint("SuspiciousIndentation")
    public void AuraDeposit()
    {
        ElapsedTime timer = new ElapsedTime();
        if(gamepad2.a)
            timer_gp2_buttonA.reset();
            if(timer_gp2_buttonA.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                Aurelius.depositFlipper.setTargetState(Aura_DepositController.DepositState.Up);
                Aurelius.depositFlipper.update();
            }
        if(timer_gp2_buttonY.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
            Aurelius.depositFlipper.Lid.setPosition(Lid_Open_Pos);
            Aurelius.depositFlipper.update();

        }
        if(timer_gp2_buttonB.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
            Aurelius.depositFlipper.setTargetState(Aura_DepositController.DepositState.Down);
            Aurelius.depositFlipper.update();

        }
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