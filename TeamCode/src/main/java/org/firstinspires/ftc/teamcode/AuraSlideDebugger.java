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

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.AuraRobot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.AuraRobot.ELBOW_DOWN;
import static org.firstinspires.ftc.teamcode.AuraRobot.LowerLimit;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_RAISE_HIGH;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_FLIP_HEIGHT;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_RAISE_MED;
import static org.firstinspires.ftc.teamcode.AuraRobot.SlidePower_Down;
import static org.firstinspires.ftc.teamcode.AuraRobot.SlidePower_Up;
import static org.firstinspires.ftc.teamcode.AuraRobot.WRIST_TUCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.slideTicks_stepSize;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="AuraSlideDebugger" , group="Manual mode")
//@Disabled


public class AuraSlideDebugger extends LinearOpMode {
    // Declare OpMode members.
    AuraRobot Aura = new AuraRobot();

    private boolean changingSlideSpeed = false;
    boolean ServoTurn = false;
    double Claw_Position; // Start at halfway position

    // Dashboard exposed control variables
//    public static double UpAdjust = 10;
//    public static int SLIDE_RAISE_HIGH = 1000;
//    public static int SLIDE_RAISE_MED = 800;
//    public static int SLIDE_RAISE_LOW = 450;
//    public static int GroundJunction = 100;
//    public static int SLIDE_INTAKE_POS = 10;
//    public static int Cone1 = 20;
//    public static int Cone2 = 70;
//    public static int Cone3 = 110;
//    public static int Cone4 = 150;
//    public static int Cone5 = 190;
//    public static double SlidePower_Up= 1;
//    public static double SlidePower_Down = 0.5;
//    public static int ticks_stepSize = 100;
    public static int Mode = 3;
//    public static int BUTTON_TRIGGER_TIMER_MS = 500;


    //    private static ElapsedTime timer_gp1_buttonA;
//    private static ElapsedTime timer_gp1_buttonX;
//    private static ElapsedTime timer_gp1_buttonY;
//    private static ElapsedTime timer_gp1_buttonB;
//    private static ElapsedTime timer_gp1_dpad_up;
//    private static ElapsedTime timer_gp1_dpad_down;
//    private static final ElapsedTime timer_gp1_dpad_left = new ElapsedTime(MILLISECONDS);
//    private static final ElapsedTime timer_gp1_dpad_right = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonA = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonX = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonY = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonB = new ElapsedTime(MILLISECONDS);
    //    private static ElapsedTime timer_gp2_dpad_up = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_dpad_down = new ElapsedTime(MILLISECONDS);
    private static final ElapsedTime timer_gp2_dpad_left = new ElapsedTime(MILLISECONDS);
    private static final ElapsedTime timer_gp2_dpad_right = new ElapsedTime(MILLISECONDS);

    FtcDashboard mvrkDashboard;
    private boolean assumingHighPosition = false;
    private boolean assumingMidPosition = false;
    private boolean assumingLowPosition = false;
    private boolean assumingSLIDE_INTAKE_POS = false;

    private int currPos = 0;
    private int newPos = currPos;

    //    public enum TomJerryState {
    //        LIFT_FLOOR,
    //        LIFT_HIGH_POS,
    //        LIFT_MEDIUM_POS,
    //        LIFT_LOW_POS,
    //    }

    // AutoState Transitions:
    // FLOOR -> High = +pwr, position
    // FLOOR -> Med =  +pwr, position
    // Floor -> low =  +pwr, position
    // High -> Med =   -pwr, position
    // High -> Low =   -pwr, position
    // High -> Floor = -pwr, position
    // Med -> High = +pwr, position
    // Med -> Low = -pwr, position
    // Med -> Floor = -pwr, position
    // Low -> High = +pwr, position
    // Low -> Med = +pwr, position
    // Low -> Floor = -pwr, position
    // Floor -> Floor = No Op
    // Low -> Low = No Op
    // Med -> Med= No Op
    // High -> High = No Op

    @Override
    public void runOpMode() {

        // Initialize the drive system variables
        Aura.init(hardwareMap);
        initAura();

//        Aura.setPosition(AuraRobot.AuraServos.TWIN_TOWERS, Aura.Claw_Open_Pos);

        waitForStart();

        if( Mode == 1 || Mode == 2) {
            Aura.setRunMode(AuraRobot.AuraMotors.SLIDE, RUN_USING_ENCODER);
        }
        else
            Aura.setRunMode(AuraRobot.AuraMotors.SLIDE, STOP_AND_RESET_ENCODER);
        Aura.setRunMode(AuraRobot.AuraMotors.SLIDE, RUN_WITHOUT_ENCODER);

        telemetry.addData("Slide Debugger Running in mode: ", Mode);
        telemetry.update();
        Servo Wrist = hardwareMap.get(Servo.class, "tilt");
        Servo Elbow = hardwareMap.get(Servo.class, "flip");

        Aura.setRunMode(AuraRobot.AuraMotors.SLIDE, STOP_AND_RESET_ENCODER);
        telemetry.addData("rykUpSlide_pid: Current Slide Position: ", Aura.getCurrentPosition(AuraRobot.AuraMotors.SLIDE));
        telemetry.update();


        Wrist.setPosition(WRIST_TUCK);
        Elbow.setPosition(ELBOW_DOWN);


        while (opModeIsActive()) {
            if (Mode == 0) {
                mvrkUpSlide();
            } else if(Mode == 1){
                mvrkUpSlide_rue();
            } else if(Mode == 2) {
                mvrkSlideTester_rue();
            } else if(Mode == 3) {
                auraSlideTester_pid();
            }
            else if(Mode == 4) {
                mvrkUpSlide_rtp();
            }
            telemetry.addData("Ryk reporting voltage: ", Aura.getBatteryVoltage());

            //rykClaw();
        }
    }

    // Initialization

    public void initAura() {
        msStuckDetectStop = 2500;
        FtcDashboard Dash = mvrkDashboard;
        currPos = 0;
        Aura.Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Ryk reporting voltage: ", Aura.getBatteryVoltage());
        telemetry.addData("Status:", "Ryk is ready to roll!");
        telemetry.update();

        return;
    }

    //Slider speed control

    public void mvrkUpSlide() {

        //Gamepad2 left -> Decrease Speed
        if (gamepad2.dpad_left) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_left.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (AuraRobot.UpAdjust <= 1) {
                    AuraRobot.UpAdjust = 1;
                } else {
                    AuraRobot.UpAdjust -= 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", AuraRobot.UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        //Gamepad 2right -> Increase Speed
        if (gamepad2.dpad_right) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_right.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (AuraRobot.UpAdjust >= 10) {
                    AuraRobot.UpAdjust = 10;
                } else {
                    AuraRobot.UpAdjust += 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", AuraRobot.UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        Aura.setPower(AuraRobot.AuraMotors.SLIDE, -gamepad2.left_stick_y * (AuraRobot.UpAdjust / 10));


        telemetry.addData("gamepad power", Aura.Slide.getPower());
        telemetry.addData("rykUpSlide: Current Slide Position: ", Aura.getCurrentPosition(AuraRobot.AuraMotors.SLIDE));
        telemetry.update();

    }

    public void mvrkUpSlide_rue() {
        //Gamepad2 left -> Decrease Speed
        if (gamepad2.dpad_left) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_left.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (AuraRobot.UpAdjust <= 1) {
                    AuraRobot.UpAdjust = 1;
                } else {
                    AuraRobot.UpAdjust -= 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", AuraRobot.UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        //Gamepad 2right -> Increase Speed
        if (gamepad2.dpad_right) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_right.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (AuraRobot.UpAdjust >= 10) {
                    AuraRobot.UpAdjust = 10;
                } else {
                    AuraRobot.UpAdjust += 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", AuraRobot.UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        int newPos = currPos + (int) ( -gamepad2.left_stick_y * slideTicks_stepSize);
        if( newPos >= SLIDE_RAISE_HIGH)
            newPos = SLIDE_RAISE_HIGH;
        else if (newPos <= LowerLimit)
            newPos = LowerLimit;
        telemetry.addData("newPos calc from gamePad2.left_stick_y: ", newPos);
        telemetry.update();

        if (gamepad2.y) {
            if (!assumingHighPosition) {
                timer_gp2_buttonY.reset();
                assumingHighPosition = true;
            } else if (timer_gp2_buttonY.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_Y triggered. Set Tom&Jerry to High position");
                telemetry.update();

                newPos = SLIDE_RAISE_HIGH;
                assumingHighPosition = false;
            }
        }

        if (gamepad2.x) {
            if (!assumingMidPosition) {
                timer_gp2_buttonX.reset();
                assumingMidPosition = true;
            } else if (timer_gp2_buttonX.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_X triggered. Set Tom&Jerry to Mid position");
                telemetry.update();
                newPos = SLIDE_RAISE_MED;
                assumingMidPosition = false;
            }
        }

        if(gamepad2.a) {
            if (!assumingLowPosition) {
                timer_gp2_buttonY.reset();
                assumingLowPosition = true;
            } else if (timer_gp2_buttonA.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_A triggered. Set Tom&Jerry to Low position");
                telemetry.update();

                newPos = SLIDE_FLIP_HEIGHT;
                assumingLowPosition = false;
            }
        }

        if (gamepad2.b) {
            if (!assumingSLIDE_INTAKE_POS) {
                timer_gp2_buttonB.reset();
                assumingSLIDE_INTAKE_POS = true;
            } else if (timer_gp2_buttonB.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_B triggered. Set Tom&Jerry to Floor position");
                telemetry.update();

                newPos = LowerLimit;
                assumingSLIDE_INTAKE_POS = false;
            }
        }

        telemetry.addData("newPos from Any button triggers: ", newPos);
        telemetry.update();

        if( newPos != currPos && newPos >= LowerLimit && newPos <= SLIDE_RAISE_HIGH ) {
            Aura.setTargetPosition(AuraRobot.AuraMotors.SLIDE, newPos);
            Aura.setRunMode(AuraRobot.AuraMotors.SLIDE, RUN_TO_POSITION);
            if (newPos > currPos) {
                Aura.setPower(AuraRobot.AuraMotors.SLIDE, SlidePower_Up);
                while (opModeIsActive() && Aura.areMotorsBusy(AuraRobot.AuraMotors.SLIDE)) {
                    idle();
                }
            }
            else if (newPos < currPos) {
                Aura.setPower(AuraRobot.AuraMotors.SLIDE, SlidePower_Down);
                while (opModeIsActive() && Aura.areMotorsBusy(AuraRobot.AuraMotors.SLIDE)) {
                    idle();
                }
            }
            currPos = newPos;
            telemetry.addData("currPos updated to: ", currPos);
            telemetry.update();
        }

        telemetry.addData("rykUpSlide_rue: Current Slide Position: ", Aura.getCurrentPosition(AuraRobot.AuraMotors.SLIDE));
        telemetry.update();
    }


    public void mvrkUpSlide_rtp() {
        //Gamepad2 left -> Decrease Speed
        if (gamepad2.dpad_left) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_left.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (AuraRobot.UpAdjust <= 1) {
                    AuraRobot.UpAdjust = 1;
                } else {
                    AuraRobot.UpAdjust -= 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", AuraRobot.UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        //Gamepad 2right -> Increase Speed
        if (gamepad2.dpad_right) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_right.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (AuraRobot.UpAdjust >= 10) {
                    AuraRobot.UpAdjust = 10;
                } else {
                    AuraRobot.UpAdjust += 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", AuraRobot.UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        int newPos = currPos + (int) ( -gamepad2.left_stick_y * slideTicks_stepSize);
        if( newPos >= SLIDE_RAISE_HIGH)
            newPos = SLIDE_RAISE_HIGH;
        else if (newPos <= LowerLimit)
            newPos = LowerLimit;
        telemetry.addData("newPos calc from gamePad2.left_stick_y: ", newPos);
        telemetry.update();

        if (gamepad2.y) {
            if (!assumingHighPosition) {
                timer_gp2_buttonY.reset();
                assumingHighPosition = true;
            } else if (timer_gp2_buttonY.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_Y triggered. Set Tom&Jerry to High position");
                telemetry.update();

                newPos = SLIDE_RAISE_HIGH;
                assumingHighPosition = false;
            }
        }

        if (gamepad2.x) {
            if (!assumingMidPosition) {
                timer_gp2_buttonX.reset();
                assumingMidPosition = true;
            } else if (timer_gp2_buttonX.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_X triggered. Set Tom&Jerry to Mid position");
                telemetry.update();
                newPos = SLIDE_RAISE_MED;
                assumingMidPosition = false;
            }
        }

        if(gamepad2.a) {
            if (!assumingLowPosition) {
                timer_gp2_buttonY.reset();
                assumingLowPosition = true;
            } else if (timer_gp2_buttonA.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_A triggered. Set Tom&Jerry to Low position");
                telemetry.update();

                newPos = SLIDE_FLIP_HEIGHT;
                assumingLowPosition = false;
            }
        }

        if (gamepad2.b) {
            if (!assumingSLIDE_INTAKE_POS) {
                timer_gp2_buttonB.reset();
                assumingSLIDE_INTAKE_POS = true;
            } else if (timer_gp2_buttonB.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_B triggered. Set Tom&Jerry to Floor position");
                telemetry.update();

                newPos = LowerLimit;
                assumingSLIDE_INTAKE_POS = false;
            }
        }

        telemetry.addData("newPos from Any button triggers: ", newPos);
        telemetry.update();

        if( newPos != currPos && newPos >= LowerLimit && newPos <= SLIDE_RAISE_HIGH ) {
            Aura.setTargetPosition(AuraRobot.AuraMotors.SLIDE, newPos);
            Aura.setRunMode(AuraRobot.AuraMotors.SLIDE, RUN_TO_POSITION);
            if (newPos > currPos) {
                Aura.setPower(AuraRobot.AuraMotors.SLIDE, SlidePower_Up);
            }
            else if (newPos < currPos) {
                Aura.setPower(AuraRobot.AuraMotors.SLIDE, SlidePower_Down);
            }
            currPos = newPos;
            telemetry.addData("currPos updated to: ", currPos);
            telemetry.update();
        }

        telemetry.addData("rykUpSlide_rue: Current Slide Position: ", Aura.getCurrentPosition(AuraRobot.AuraMotors.SLIDE));
        telemetry.update();
    }


    public void mvrkSlideTester_rue(){

        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();

//        Aura.setPosition(AuraRobot.AuraServos.TWIN_TOWERS, Claw_Open_Pos);

//        Aura.setPosition(AuraRobot.AuraServos.TWIN_TOWERS, Claw_Close_Pos);

        Aura.setTargetPosition(AuraRobot.AuraMotors.SLIDE, SLIDE_FLIP_HEIGHT);
        Aura.setRunMode(AuraRobot.AuraMotors.SLIDE, RUN_TO_POSITION);
        Aura.setPower(AuraRobot.AuraMotors.SLIDE, SlidePower_Up);
        
//        Aura.setPosition(AuraRobot.AuraServos.TWIN_TOWERS, Claw_Open_Pos);
        
//        Aura.setPosition(AuraRobot.AuraServos.TWIN_TOWERS, Claw_Close_Pos);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Aura.setTargetPosition(AuraRobot.AuraMotors.SLIDE, SLIDE_FLIP_HEIGHT);
        Aura.setRunMode(AuraRobot.AuraMotors.SLIDE, RUN_TO_POSITION);
        Aura.setPower(AuraRobot.AuraMotors.SLIDE, SlidePower_Up);

//        Aura.setPosition(AuraRobot.AuraServos.TWIN_TOWERS, Claw_Open_Pos);
        
//        Aura.setPosition(AuraRobot.AuraServos.TWIN_TOWERS, Claw_Close_Pos);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Aura.setTargetPosition(AuraRobot.AuraMotors.SLIDE, SLIDE_FLIP_HEIGHT);
        Aura.setRunMode(AuraRobot.AuraMotors.SLIDE, RUN_TO_POSITION);
        Aura.setPower(AuraRobot.AuraMotors.SLIDE, SlidePower_Up);
        
//        Aura.setPosition(AuraRobot.AuraServos.TWIN_TOWERS, Claw_Open_Pos);
        
//        Aura.setPosition(AuraRobot.AuraServos.TWIN_TOWERS, Claw_Close_Pos);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        Aura.setTargetPosition(AuraRobot.AuraMotors.SLIDE, SLIDE_FLIP_HEIGHT);
        Aura.setRunMode(AuraRobot.AuraMotors.SLIDE, RUN_TO_POSITION);
        Aura.setPower(AuraRobot.AuraMotors.SLIDE, SlidePower_Up);
        
//        Aura.setPosition(AuraRobot.AuraServos.TWIN_TOWERS, Claw_Open_Pos);

//        Aura.setPosition(AuraRobot.AuraServos.TWIN_TOWERS, Claw_Close_Pos);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Aura.setTargetPosition(AuraRobot.AuraMotors.SLIDE, SLIDE_FLIP_HEIGHT);
        Aura.setRunMode(AuraRobot.AuraMotors.SLIDE, RUN_TO_POSITION);
        Aura.setPower(AuraRobot.AuraMotors.SLIDE, SlidePower_Up);
        
//        Aura.setPosition(AuraRobot.AuraServos.TWIN_TOWERS, Claw_Open_Pos);

        telemetry.addData("Tester_rue: Current Slide Position: ", Aura.getCurrentPosition(AuraRobot.AuraMotors.SLIDE));
        telemetry.update();
    }

    public void auraSlideTester_pid() {

        //Gamepad2 left -> Decrease Speed
        if (gamepad2.dpad_left) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_left.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (AuraRobot.UpAdjust <= 1) {
                    AuraRobot.UpAdjust = 1;
                } else {
                    AuraRobot.UpAdjust -= 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", AuraRobot.UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        //Gamepad 2right -> Increase Speed
        if (gamepad2.dpad_right) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_right.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (AuraRobot.UpAdjust >= 10) {
                    AuraRobot.UpAdjust = 10;
                } else {
                    AuraRobot.UpAdjust += 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", AuraRobot.UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        newPos = newPos + (int) (-gamepad2.left_stick_y * slideTicks_stepSize);
        if( newPos >= SLIDE_RAISE_HIGH)
            newPos = SLIDE_RAISE_HIGH;
        else if (newPos <= LowerLimit)
            newPos = LowerLimit;
        telemetry.addData("newPos", newPos);
        telemetry.update();

        if (gamepad2.y) {
            if (!assumingHighPosition) {
                timer_gp2_buttonY.reset();
                assumingHighPosition = true;
            } else if (timer_gp2_buttonY.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_Y triggered. Set Tom&Jerry to High position");
                telemetry.update();

                newPos = SLIDE_RAISE_HIGH;
                assumingHighPosition = false;
            }
        }

        if (gamepad2.x) {
            if (!assumingMidPosition) {
                timer_gp2_buttonX.reset();
                assumingMidPosition = true;
            } else if (timer_gp2_buttonX.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_X triggered. Set Tom&Jerry to Mid position");
                telemetry.update();
                newPos = SLIDE_RAISE_MED;
                assumingMidPosition = false;
            }
        }

        if(gamepad2.a) {
            if (!assumingLowPosition) {
                timer_gp2_buttonY.reset();
                assumingLowPosition = true;
            } else if (timer_gp2_buttonA.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_A triggered. Set Tom&Jerry to Low position");
                telemetry.update();

                newPos = SLIDE_FLIP_HEIGHT;
                assumingLowPosition = false;
            }
        }

        if (gamepad2.b) {
            if (!assumingSLIDE_INTAKE_POS) {
                timer_gp2_buttonB.reset();
                assumingSLIDE_INTAKE_POS = true;
            } else if (timer_gp2_buttonB.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_B triggered. Set Tom&Jerry to Floor position");
                telemetry.update();

                newPos = LowerLimit;
                assumingSLIDE_INTAKE_POS = false;
            }
        }

        telemetry.addData("newPos", newPos);
        telemetry.update();

        if( newPos != currPos && newPos >= LowerLimit && newPos <= SLIDE_RAISE_HIGH ) {
            double command = AuraRobot.liftController.output(newPos, Aura.getCurrentPosition(AuraRobot.AuraMotors.SLIDE));
            if(newPos < currPos)
                AuraRobot.SlidePower = Math.max(command/SLIDE_RAISE_HIGH, SlidePower_Down);
            else
                AuraRobot.SlidePower = Math.min(command/SLIDE_RAISE_HIGH, SlidePower_Up);

            Aura.setPower(AuraRobot.AuraMotors.SLIDE,AuraRobot.SlidePower);
            currPos = Aura.getCurrentPosition(AuraRobot.AuraMotors.SLIDE);
            telemetry.addData("currPos updated to: ", currPos);
            telemetry.addData("command power:", AuraRobot.SlidePower);
            telemetry.update();
        }

        telemetry.addData("rykUpSlide_pid: Current Slide Position: ", Aura.getCurrentPosition(AuraRobot.AuraMotors.SLIDE));
        telemetry.update();
    }
}
