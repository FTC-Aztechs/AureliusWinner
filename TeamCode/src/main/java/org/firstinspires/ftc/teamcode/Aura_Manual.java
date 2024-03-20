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
import static org.firstinspires.ftc.teamcode.AuraIntakeOuttakeController.targetSlidePos;
import static org.firstinspires.ftc.teamcode.AuraRobot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.AuraRobot.HANG_POWER;
import static org.firstinspires.ftc.teamcode.AuraRobot.LEFT_FINGER_UNLOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.Launcher_Fire_Pos;
import static org.firstinspires.ftc.teamcode.AuraRobot.Launcher_Set_Pos;
import static org.firstinspires.ftc.teamcode.AuraRobot.RIGHT_FINGER_UNLOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.Ramp_Down_Pos;
import static org.firstinspires.ftc.teamcode.AuraRobot.Ramp_Up_Pos;
import static org.firstinspires.ftc.teamcode.AuraRobot.rightLinkageOpen;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_INTAKE_POS;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_RAISE_HIGH;
import static org.firstinspires.ftc.teamcode.AuraRobot.bumperSpeedAdjust;
import static org.firstinspires.ftc.teamcode.AuraRobot.dPadSpeedAdjust;
import static org.firstinspires.ftc.teamcode.AuraRobot.leftLinkageClose;
import static org.firstinspires.ftc.teamcode.AuraRobot.leftLinkageOpen;
import static org.firstinspires.ftc.teamcode.AuraRobot.rightLinkageClose;
import static org.firstinspires.ftc.teamcode.AuraRobot.slideTicks_stepSize;
import static org.firstinspires.ftc.teamcode.AuraRobot.speedAdjust;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
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

    private double intakeSpeed;
    public static double intakeMaxSpeed = 1;

    private boolean changingState = false;

    public RevColorSensorV3 Left;

    public ColorRangeSensor Right;

    public RevBlinkinLedDriver BlinkinBoard;

    private int slide_currentPos = 0;
    private int slide_newPos = slide_currentPos;

    public  RevBlinkinLedDriver.BlinkinPattern[] colors = {WHITE_PATTERN, GREEN_PATTERN, PURPLE_PATTERN, YELLOW_PATTERN};
    public  int[][] rightRanges = {
            {1400, 1700, 1600, 1950, 1500, 1800}, // White order is RGB
            {264, 364, 468, 568, 237, 337},      // Green
            {500, 750, 550, 720, 710, 950},      // Purple
            {782, 882, 584, 684, 312, 412}       // Yellow
    };
    public  int[][] leftRanges = {
            {1365, 1465, 2382, 2482, 2244, 2344},// White
            {348, 448, 1065, 1165, 460, 560},    // Green
            {800, 1100, 1200, 1650, 1600, 2300},  // Purple
            {1100, 1300, 1400, 1770, 420, 590}   // Yellow
    };


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

    private static ElapsedTime timer_gp2_rt = new ElapsedTime(MILLISECONDS);


    private static ElapsedTime timer_gp2_x = new ElapsedTime(MILLISECONDS);

    private static ElapsedTime timer_gp2_y = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_a = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_b = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_up = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_down = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_rb = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp1_rt = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp1_lt = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_right_stick_button = new ElapsedTime(MILLISECONDS);

    private static ElapsedTime timer_gp1_lb = new ElapsedTime(MILLISECONDS);
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
    private static ElapsedTime timer_gp1_y = new ElapsedTime(MILLISECONDS);

    //slide button booleans
    private boolean assumingHighPosition = false;
    private boolean assumingMidPosition = false;
    private boolean assumingLowPosition = false;
    private boolean assumingFloorPosition = false;
    private boolean assumingTopMidCone = false;
    private boolean assumingMiddleCone = false;

    private boolean rightDetected = false;
    private boolean leftDetected = false;


    public ElapsedTime PatternTimer;

    private static final RevBlinkinLedDriver.BlinkinPattern WHITE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    private static final RevBlinkinLedDriver.BlinkinPattern GREEN_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    private static final RevBlinkinLedDriver.BlinkinPattern PURPLE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
    private static final RevBlinkinLedDriver.BlinkinPattern YELLOW_PATTERN = RevBlinkinLedDriver.BlinkinPattern.YELLOW;



    // Define Fingers
    private Servo LeftFinger = null;
    private Servo RightFinger = null;

    private Servo RightLink = null;
    private Servo LeftLink = null;


    private RevBlinkinLedDriver.BlinkinPattern rightDetectedColor = colors[1];
    private RevBlinkinLedDriver.BlinkinPattern leftDetectedColor = colors[1];

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
        BlinkinBoard = hardwareMap.get(RevBlinkinLedDriver.class, "Blink");
        Left = hardwareMap.get(RevColorSensorV3.class, "Left");
        Right = hardwareMap.get(ColorRangeSensor.class, "Right");
        LeftLink = hardwareMap.get(Servo.class, "LeftLink");
        RightLink = hardwareMap.get(Servo.class, "RightLink");

        PatternTimer = new ElapsedTime();
        PatternTimer.reset();

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
//            AuraColor();
            //AuraHang();

        }
    }




    public void AuraFingers()
    {
        if(gamepad1.left_trigger == 1f) {
            if(!changingState) {
                timer_gp1_lt.reset();
                changingState = true;
            } else if (timer_gp1_lt.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                LeftFinger.setPosition(LEFT_FINGER_UNLOCK);
                changingState = false;
            }
        }
        if(gamepad1.right_trigger == 1f) {
            if(!changingState) {
                timer_gp1_rt.reset();
                changingState = true;
            } else if (timer_gp1_lt.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                RightFinger.setPosition(RIGHT_FINGER_UNLOCK);
                changingState = false;
            }
        }

        if(gamepad1.left_bumper) {
            if(!changingState) {
                timer_gp1_lb.reset();
                changingState = true;
            } else if (timer_gp1_lb.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                RightFinger.setPosition(RIGHT_FINGER_UNLOCK);
                LeftFinger.setPosition(LEFT_FINGER_UNLOCK);
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
        LeftLink.setPosition(leftLinkageClose);
        RightLink.setPosition(rightLinkageClose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Status: Robot is ready to roll!");
        telemetry.update();
    }

    public void AuraColor() {
        rightDetectedColor =  RevBlinkinLedDriver.BlinkinPattern.BLACK;
        leftDetectedColor = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        telemetry.addData("Right Red: ", Right.red()); // color range
        telemetry.addData("Right Green: ", Right.green());
        telemetry.addData("Right Blue: ", Right.blue());
        telemetry.addData("Left Red: ", Left.red());
        telemetry.addData("Left Green: ", Left.green());
        telemetry.addData("Left Blue: ", Left.blue());

//         Check the color for Right sensor
        for (int i = 0; i < colors.length; i++) {
            if (Right.red() >= rightRanges[i][0] && Right.red() <= rightRanges[i][1] &&
                    Right.green() >= rightRanges[i][2] && Right.green() <= rightRanges[i][3] &&
                    Right.blue() >= rightRanges[i][4] && Right.blue() <= rightRanges[i][5]) {
                rightDetectedColor = colors[i];
            }

            if (Left.red() >= leftRanges[i][0] && Left.red() <= leftRanges[i][1] &&
                    Left.green() >= leftRanges[i][2] && Left.green() <= leftRanges[i][3] &&
                    Left.blue() >= leftRanges[i][4] && Left.blue() <= leftRanges[i][5]) {
                leftDetectedColor = colors[i];
            }

            if (rightDetectedColor != RevBlinkinLedDriver.BlinkinPattern.BLACK && leftDetectedColor != RevBlinkinLedDriver.BlinkinPattern.BLACK) {
                break;
            }
        }

        if(PatternTimer.seconds()<1) {
            BlinkinBoard.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        } else if (PatternTimer.seconds() < 2) {
            BlinkinBoard.setPattern(leftDetectedColor);
        } else if (PatternTimer.seconds() < 3) {
            BlinkinBoard.setPattern(rightDetectedColor);
        } else {
            PatternTimer.reset();
        }

        telemetry.update();
    }
//
//
//    private RevBlinkinLedDriver.BlinkinPattern getBlinkinPatternForColor(int red, int green, int blue, int[][] colorRanges, String[] colorNames) {
//        for (int i = 0; i < colorNames.length; i++) {
//            if (red >= colorRanges[i][0] && red <= colorRanges[i][1] && green >= colorRanges[i][2] && green <= colorRanges[i][3] && blue >= colorRanges[i][4] && blue <= colorRanges[i][5]) {
//                switch (colorNames[i]) {
//                    case "White":
//                        return WHITE_PATTERN;
//                    case "Green":
//                        return GREEN_PATTERN;
//                    case "Purple":
//                        return PURPLE_PATTERN;
//                    case "Yellow":
//                        return YELLOW_PATTERN;
//                }
//            }
//        }
//        return RevBlinkinLedDriver.BlinkinPattern.AQUA;
//    }


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
        if (gamepad2.x) {
            if (!changingState) {
                timer_gp2_x.reset();
                changingState = true;
            } else if (timer_gp2_x.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                Aurelius.hanger.setTargetState(AuraHangController.HangState.Idle);
                Aurelius.hanger.update();
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
                Aurelius.hanger.update();
                telemetry.addData("State ", " Up");
                changingState = false;
            }
        }
        if (gamepad2.dpad_down == true) {
            Aurelius.Hang.setPower(HANG_POWER);
        } else if (gamepad2.dpad_up == true) {
            Aurelius.Hang.setPower(-HANG_POWER);
        } else {
            Aurelius.Hang.setPower(0);
        }

    }

    public void AuraIntakeRoller() {

        intakeSpeed = -gamepad2.right_stick_y;
        if (intakeSpeed >= intakeMaxSpeed) {
            intakeSpeed = intakeMaxSpeed;
        }
        Aurelius.setPower(AuraRobot.AuraMotors.INTAKE,(intakeSpeed));
        if(gamepad2.right_stick_y != 0f) {
            Aurelius.Ramp.setPosition(Ramp_Down_Pos);
        } else if (gamepad2.right_stick_y == 0f){
            Aurelius.Ramp.setPosition(Ramp_Up_Pos);
        }

    }

public void AuraIntakeOuttake() {
    if(gamepad2.right_trigger == 1f) {
        if(!changingState) {
            timer_gp2_rt.reset();
            changingState = true;
        } else if (timer_gp2_rt.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
            LeftLink.setPosition(leftLinkageOpen);
            RightLink.setPosition(rightLinkageOpen);
            changingState = false;
        }
    } else if(gamepad2.right_trigger == 0f) {
        LeftLink.setPosition(leftLinkageClose);
        RightLink.setPosition(rightLinkageClose);
    }


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

    if(gamepad2.right_stick_button)
        if(!changingState) {
            timer_gp2_right_stick_button.reset();
            changingState = true;
        } else if(timer_gp2_right_stick_button.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS ){
            myIntakeOuttakeController.setTargetState(AuraIntakeOuttakeController.ioState.STATE_2_ITA);
            telemetry.addData("State", "going to Tuck Intake");
            telemetry.update();
            changingState = false;
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

    public void AuraLauncher() {
        if (gamepad1.y) {
            if (!changingLauncherSpeed) {
                timer_gp1_y.reset();
                changingLauncherSpeed = true;
            } else if (timer_gp1_y.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (PlaneLaunched == false) {
                    Aurelius.boeing747.launcher.setPosition(Launcher_Fire_Pos);
                    changingLauncherSpeed = false;
////                    Aurelius.boeing747.setTargetState(AuraLaunchController.launchState.Launch);
////                    Aurelius.boeing747.update();
////                    PlaneLaunched = true;
////                } else {
////                    Aurelius.boeing747.setTargetState(AuraLaunchController.launchState.Set);
////                    Aurelius.boeing747.update();
////                    PlaneLaunched = false;
////                }
////                telemetry.addLine("Current State:" + Aurelius.boeing747.currState);
////                telemetry.update();
            }
            }
        }
        else Aurelius.boeing747.launcher.setPosition(Launcher_Set_Pos);
    }

}