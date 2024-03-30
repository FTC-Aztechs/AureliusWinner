package org.firstinspires.ftc.teamcode;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.*;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.AuraRobot.ELBOW_DOWN;
import static org.firstinspires.ftc.teamcode.AuraRobot.ELBOW_UP;
import static org.firstinspires.ftc.teamcode.AuraRobot.LEFT_FINGER_LOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.LEFT_FINGER_UNLOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.LowerLimit;
import static org.firstinspires.ftc.teamcode.AuraRobot.RIGHT_FINGER_LOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.RIGHT_FINGER_UNLOCK;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_INTAKE_POS;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_RAISE_HIGH;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_FLIP_HEIGHT;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_RAISE_LOW;
import static org.firstinspires.ftc.teamcode.AuraRobot.SLIDE_RAISE_MED;
import static org.firstinspires.ftc.teamcode.AuraRobot.SlidePower;
import static org.firstinspires.ftc.teamcode.AuraRobot.SlidePower_Down;
import static org.firstinspires.ftc.teamcode.AuraRobot.SlidePower_Up;
import static org.firstinspires.ftc.teamcode.AuraRobot.UpperLimit;
import static org.firstinspires.ftc.teamcode.AuraRobot.WRIST_INTAKE;
import static org.firstinspires.ftc.teamcode.AuraRobot.WRIST_TUCK;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;


@Config
public class AuraIntakeOuttakeController {
    public DcMotor Slide;
    public Servo Wrist;
    public Servo Elbow;
    public Servo LeftFinger;
    public Servo RightFinger;

    private RevBlinkinLedDriver BlinkBoard;

    public RevColorSensorV3 Left;
    public ColorRangeSensor Right;
    public ElapsedTime flipTimer;
    public ElapsedTime slideTimer;

    public ElapsedTime colorTimer;
    public ElapsedTime wristTimer; // TODO: Remove this after we have color sensors implemented


    // main states are State 1, State 3,  State 5 manual, and State 6 (in auto) other states are transions or unused. *Target pos can only be set to these)
    public enum ioState {
        STATE_0_UNINITIALIZED, // At Init, set Target to 1_RFI and leave Current at Uninitialized
        STATE_1_RFI,    // State 1: Ready for Intake
        STATE_2_ITA,     // State 2: Intake tucked away (slides raised to minimum)
        STATE_3_PS,    // State 3: Pixels Secured
        STATE_4_BF,     // State 4: Box Flipped
        STATE_5_RFO_LOW,    // State 5: Ready for Outtake, low height (unused)
        STATE_5_RFO_MID,    // State 5: Ready for Outtake, medium height (used in Auto)
        STATE_5_RFO_HIGH,   // State 5: Ready for Outtake, high height (unused)
        STATE_5_RFO_MANUAL, // State 5: Ready for Outtake, driver controlled height, driver controlled pixel drop (used in Manual)
        STATE_6_PR_BOTH,      // State 6: Pixels Released (used in auto)
    }

    ioState currState;
    ioState targetState;
    ioState nextState;
    public static double targetSlidePos;
    double currSlidePos;
    public static boolean isManual;
    public static boolean goingUp = false;
    public static AuraServoPIDController servoController;
    public static double slideTickError = 10;
    public static double SLIDE_WAIT_TIME_LIMIT = 0.5;
    public static double FLIP_WAIT_TIME_LIMIT = 0.5;
    public static double WRIST_WAIT_TIME_LIMIT = 1;
    private Telemetry telemetry;


    public static boolean bPixelsDetected = false;
    public static boolean rightDetected = false;
    public static boolean leftDetected = false;
    public boolean safeToUnload = false;

    //TODO: change numbers
    public static AuraPIDController slideUpPID = new AuraPIDController(12, 0, 0, 6); // KD Values .25 -> .32 KG Previous Values 3600 -> 5500 2/19/2023
    public static AuraPIDController slideDownPID = new AuraPIDController(12, 0, 0, 0);

    public AuraIntakeOuttakeController(HardwareMap hardwareMap, boolean Manual) {

        Wrist = hardwareMap.get(Servo.class, "tilt");
        Elbow = hardwareMap.get(Servo.class, "flip");
        LeftFinger = hardwareMap.get(Servo.class, "lefty");
        RightFinger = hardwareMap.get(Servo.class, "righty");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Left = hardwareMap.get(RevColorSensorV3.class, "Left");
        Right = hardwareMap.get(ColorRangeSensor.class, "Right");

        currState = ioState.STATE_0_UNINITIALIZED;
        targetState = ioState.STATE_1_RFI;
        nextState = ioState.STATE_1_RFI;

        flipTimer = new ElapsedTime();
        flipTimer.reset();

        slideTimer = new ElapsedTime();
        slideTimer.reset();

        wristTimer = new ElapsedTime();
        wristTimer.reset();

        colorTimer = new ElapsedTime();
        colorTimer.reset();

        currSlidePos = 0;
        targetSlidePos = SLIDE_INTAKE_POS;

        isManual = Manual;
    }
    public void init() {
//        Left.initialize();
//        Left.enableLed(true);
//        Right.enableLed(true);
//        Left.getLightDetected();

        Slide.setMode(STOP_AND_RESET_ENCODER);
        Slide.setMode(RUN_WITHOUT_ENCODER);
    }

    public void setTelemetry(Telemetry tele) {
        telemetry = tele;
    }

    public void setTargetPosition(double target) {
        if (target >= LowerLimit && target <= UpperLimit)
            targetSlidePos = target;
    }

    public void setTargetState(ioState eState)
    {
        if (!validStateTransition()){
            return;
        }

        flipTimer.reset();
        slideTimer.reset();
        wristTimer.reset();
        targetState = eState;

        if (targetState == ioState.STATE_1_RFI) {
            goingUp = false;
        } else {
            goingUp = true;
        }
    }

    public void getNextState() {
        switch (currState) {
            case STATE_0_UNINITIALIZED:
                nextState = ioState.STATE_1_RFI;
                break;
            case STATE_1_RFI:
                if (goingUp) {
                    nextState = ioState.STATE_2_ITA;
                } else {
                    nextState = ioState.STATE_1_RFI;
                }
                break;
            case STATE_2_ITA:
                if (goingUp) {
                    nextState = ioState.STATE_3_PS;
                } else {
                    nextState = ioState.STATE_1_RFI;
                }
                break;
            case STATE_3_PS:
                if (goingUp) {
                    nextState = ioState.STATE_4_BF;
                } else {
                    nextState = ioState.STATE_2_ITA;
                }
                break;
            case STATE_4_BF:
                if (goingUp) {
                    if (isManual) { // TODO: Unless we need to adjust slide pos to something else in Auto...
                        nextState = ioState.STATE_5_RFO_MANUAL;
                    } else {
                        nextState = ioState.STATE_5_RFO_LOW;
                    }
                } else {
                    nextState = ioState.STATE_3_PS;
                }
                break;
            case STATE_5_RFO_MANUAL:
                if (goingUp) {
                        nextState = ioState.STATE_5_RFO_MANUAL;
                } else {
                    nextState = ioState.STATE_4_BF;
                }
                break;
            case STATE_5_RFO_LOW:
                if (goingUp) {
                    nextState = ioState.STATE_6_PR_BOTH;
                } else {
                    nextState = ioState.STATE_4_BF;
                }
                break;
            case STATE_6_PR_BOTH:
                if (goingUp) {
                    nextState = ioState.STATE_6_PR_BOTH;
                } else {
                    nextState = ioState.STATE_4_BF;
                }
            default:
                break;
        }
    }

    public void AuraColor() {

        String[] colors = {"White", "Green", "Purple", "Yellow"};
        int[][] rightRanges = {
                {1400, 1700, 1600, 1950, 1500, 1800}, // White order is RGB
                {264, 364, 468, 568, 237, 337},      // Green
                {500, 750, 550, 720, 710, 950},      // Purple
                {782, 882, 584, 684, 312, 412}       // Yellow
        };
        int[][] leftRanges = {
                {1365, 1465, 2382, 2482, 2244, 2344},// White
                {348, 448, 1065, 1165, 460, 560},    // Green
                {800, 1100, 1200, 1650, 1600, 2300},  // Purple
                {1100, 1300, 1400, 1770, 420, 590}   // Yellow
        };

        // Check the color for Right sensor
        for (int i = 0; i < colors.length; i++) {
            if (Right.red() >= rightRanges[i][0] && Right.red() <= rightRanges[i][1] && Right.green() >= rightRanges[i][2] && Right.green() <= rightRanges[i][3] && Right.blue() >= rightRanges[i][4] && Right.blue() <= rightRanges[i][5]) {
                rightDetected = true;
            } else {
                rightDetected = false;
            }
        }

        // Check the color for Left sensor
        for (int i = 0; i < colors.length; i++) {
            if (Left.red() >= leftRanges[i][0] && Left.red() <= leftRanges[i][1] && Left.green() >= leftRanges[i][2] && Left.green() <= leftRanges[i][3] && Left.blue() >= leftRanges[i][4] && Left.blue() <= leftRanges[i][5]) {
                leftDetected = true;
            } else {
                leftDetected = false;
            }
        }


        if(rightDetected && leftDetected) {
            if(colorTimer.seconds() > .4) {
                setTargetState(ioState.STATE_3_PS);
                colorTimer.reset();
            }
        } else {
            colorTimer.reset();
        }
    }


    public void updateSlide() {

        //if(currSlidePos != targetSlidePos ) {
            double command;
//            = liftController.output(targetSlidePos, Slide.getCurrentPosition());
            if (targetSlidePos < currSlidePos) {
                command = slideDownPID.output(targetSlidePos, Slide.getCurrentPosition());
                SlidePower = Math.max(command / (SLIDE_RAISE_HIGH - SLIDE_INTAKE_POS), SlidePower_Down);
            } else {
                command = slideUpPID.output(targetSlidePos, Slide.getCurrentPosition());
                SlidePower = Math.min(command / (SLIDE_RAISE_HIGH - SLIDE_INTAKE_POS), SlidePower_Up);
            }
            Slide.setPower(SlidePower);
            currSlidePos = Slide.getCurrentPosition();
//            targetSlidePos = Slide.getTargetPosition();
        //}
        if(telemetry != null) {
            telemetry.addData("AuraIOController: Current Slide position: %f", currSlidePos);
            telemetry.addData("Current State: ", currState);
            telemetry.addData("TargetSlidePos:" , targetSlidePos);
            telemetry.addData("slidePower", SlidePower);
            telemetry.update();
        }
    }

    public void update()
    {

        if(currState == targetState)
        {
            updateSlide();
            AuraColor();
            return;
        }

        getNextState();

        switch(nextState) {
            case STATE_1_RFI:
//                AuraColor();
                // Ensure the Robot enforces these states on all motors & servos
                targetSlidePos = SLIDE_INTAKE_POS;
                Elbow.setPosition(ELBOW_DOWN);
                Wrist.setPosition(WRIST_INTAKE);
                LeftFinger.setPosition(LEFT_FINGER_UNLOCK); //unlock
                RightFinger.setPosition(RIGHT_FINGER_UNLOCK); //unlock

                currState = nextState;
                break;

            case STATE_2_ITA: // Intake tucked

                Wrist.setPosition(WRIST_TUCK);

                // Give the wrist some time to reach tuck position
                if(wristTimer.seconds() > WRIST_WAIT_TIME_LIMIT) {
                    currState = nextState;
                }
                break;

            case STATE_3_PS: // Pixels secured = main state

                LeftFinger.setPosition(LEFT_FINGER_LOCK); //lock
                RightFinger.setPosition(RIGHT_FINGER_LOCK); //lock

                // If Returning to intake - this is where elbow is flipped
                // Give it some time to complete the Elbow Flip down before
                // letting the slides down (later)
                if(!goingUp) {
                    Elbow.setPosition(ELBOW_DOWN);
                    if (flipTimer.seconds() > FLIP_WAIT_TIME_LIMIT) {
                        currState = nextState;
                    }
                }
                else {
                    currState = nextState;

                    // Reset Slide Timer such that state 4
                    // has a fresh timer to count
                    // Note: only needed when goingUp
                    slideTimer.reset();
                }

                break;

            case STATE_4_BF: // Box Flipped
                targetSlidePos = SLIDE_FLIP_HEIGHT;
                if(slideTimer.seconds() > SLIDE_WAIT_TIME_LIMIT)
                {
                    currState = nextState;
                    // Reset flip timer such that states 3 & states 5
                    // have a fresh timer to count
                    flipTimer.reset();
                }
                break;

            case STATE_5_RFO_MANUAL: // Ready for Outtake = main state in manual
                if(goingUp) {
                    Elbow.setPosition(ELBOW_UP);
                    if (flipTimer.seconds() > FLIP_WAIT_TIME_LIMIT) {
                        currState = nextState;

                        // Reset Slide Timer such that state 4 has a fresh timer to count
                        slideTimer.reset();
                    }
                }
                else {
                    currState = nextState;

                    // Reset Slide Timer such that state 4 has a fresh timer to count
                    slideTimer.reset();
                }
                break;

            case STATE_5_RFO_LOW: // Ready for Outtake = main state in manual
                if(goingUp) {
                    targetSlidePos = SLIDE_RAISE_LOW;
                    Elbow.setPosition(ELBOW_UP);
                    if (flipTimer.seconds() > FLIP_WAIT_TIME_LIMIT) {
                        currState = nextState;

                        // Reset Slide Timer such that state 4 has a fresh timer to count
                        slideTimer.reset();
                    }
                }
                else {
                    currState = nextState;

                    // Reset Slide Timer such that state 4 has a fresh timer to count
                    slideTimer.reset();
                }
                break;

            case STATE_6_PR_BOTH: // Pixel Release = main state in auto
                LeftFinger.setPosition(LEFT_FINGER_UNLOCK); //unlock
                RightFinger.setPosition(RIGHT_FINGER_UNLOCK); //unlock
                currState = nextState;
                break;

            default:
                break;
        }

        updateSlide();

    }

    // Ensure that this state transitoin is valid



    public boolean validStateTransition()
    {
        if (targetState == currState ) {
            return true;
        }

        if(currState == ioState.STATE_0_UNINITIALIZED)
        {
            if(targetState == ioState.STATE_1_RFI || targetState == ioState.STATE_3_PS ) {
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_1_RFI ) {
            if(targetState == ioState.STATE_3_PS
             ) {
                return true;
            }
            return false;
        }

//        if( currState == ioState.STATE_2_ITA ) {
//            if(targetState == ioState.STATE_1_RFI || targetState == ioState.STATE_5_RFO_MANUAL) {
//                return true;
//            }
//            return false;
//        }


        if( currState == ioState.STATE_3_PS) {
            if(targetState == ioState.STATE_1_RFI || targetState == ioState.STATE_5_RFO_MANUAL ){
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_5_RFO_MANUAL) {
            if(targetState == ioState.STATE_1_RFI || targetState == ioState.STATE_6_PR_BOTH){
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_6_PR_BOTH ) {
            if( targetState == ioState.STATE_1_RFI){
                return true;
            }
            return false;
        }

        return false;
    }

    public boolean readyToTransition(){
        if (Math.abs(targetSlidePos - currSlidePos) <= slideTickError) {
            return true;
        } else {
            return false;
        }
    }

}
