package org.firstinspires.ftc.teamcode;
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
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


// Aurelius Lift States
// State 1: Ready for Intake
//      * Slides @ lowest pos
//      * Wrist @ Intake angle
//      * Pixel grabbers released
//      * Elbow @ Intake
// State 2: Pixels Secured
//      - Slides @ lowest pos
//      - Wrist @ Tuck in angle
//      - Pixel grabbers released
//      - Elbow @ Intake
// State 3: Intake tucked away (slides raised to minimum)
//      * Slides @ minimum raised pos
//      * Wrist @ Tuck In angle
//      - Pixel grabbers secured
//      - Elbow @ Intake
// State 4: Box Flipped
//      - Slides @ minimum raised pos
//      - Wrist @ Tuck In angle
//      - Pixel grabbers in previous state (note: can be secured or unsecured depending on what state it is returning from)
//      * Elbow @ Outtake
// State 5: Ready for Outtake
//      - Slides @ minimum raised pos
//      * Wrist @ Outtake angle
//      - Pixel grabbers in previous state (note: can be secured or unsecured depending on what state it is returning from)
//      - Elbow @ Outtake
// State 6: Pixels Released
//      - Slides @ minimum raised pos
//      - Wrist @ Outtake angle
//      * Pixel grabbers Released
//      - Elbow @ Outtake

// Valid Transitions:
//    1 -> 2; 1 -> 3;
//    2 -> 3;
//    3 -> 4; 3 -> 1;
//    4 -> 5; 4 -> 3;
//    5 -> 6; 5 -> 4;
//    6 -> 5;

@Config
public class AuraIntakeOuttakeController {
    private DcMotor Slide;
    private Servo Wrist;
    private Servo Elbow;
    private Servo LeftFinger;
    private Servo RightFinger;

    public RevColorSensorV3 Left = null;

    public ColorRangeSensor Right = null;
    public ElapsedTime flipTimer;
    public ElapsedTime slideTimer;


    //TODO: main states are State 1, State 3, State 5 Mid (in auto), State 5 manual, and State 6 (in auto) other states are transions or unused. *Target pos can only be set to these)
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
    public static double servoPosError = 0.02;
    public static double SLIDE_WAIT_TIME_LIMIT = 1;
    public static double FLIP_WAIT_TIME_LIMIT = 1;
    private Telemetry telemetry;

    //TODO: change numbers
    public static AuraPIDController slideUpPID = new AuraPIDController(12, 0, 0, 0); // KD Values .25 -> .32 KG Previous Values 3600 -> 5500 2/19/2023
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


        currSlidePos = SLIDE_INTAKE_POS;
        targetSlidePos = SLIDE_INTAKE_POS;

        isManual = Manual;
    }

    public void init() {
        Left.initialize();
        Left.enableLed(true);
        Right.enableLed(true);
        Left.getLightDetected();

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
                    if (isManual) {
                        nextState = ioState.STATE_5_RFO_MANUAL;
                    } else {
                        nextState = ioState.STATE_5_RFO_MID;
                    }
                } else {
                    nextState = ioState.STATE_3_PS;
                }
                break;
            case STATE_5_RFO_MANUAL:
                if (goingUp) {
                    if (isManual) {
                        nextState = ioState.STATE_5_RFO_MANUAL;
                    } else {
                        nextState = ioState.STATE_6_PR_BOTH;
                    }
                } else {
                    nextState = ioState.STATE_4_BF;
                }
                break;
            case STATE_5_RFO_MID:
                if (goingUp) {
                    if (isManual) {
                        nextState = ioState.STATE_5_RFO_MID;
                    } else {
                        nextState = ioState.STATE_6_PR_BOTH;
                    }
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

    public void updateSlide() {

        if(currSlidePos != targetSlidePos ) {
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
        }
        if(telemetry != null) {
            telemetry.addData("AuraIOController: Current Slide position: %f", currSlidePos);
            telemetry.addData("Current State: ", currState);
            telemetry.addData("TargetSlidePos:" , targetSlidePos);
            telemetry.addData("slidePower", SlidePower);
            telemetry.update();
        }
    }

//    public void update() {
//
//        //ReadyI -> Secured: fingers lock
//        //ReadyI -> Tucked: slides up, tuck  wrist
//        //Secured -> Tucked: slides up, tuck wrist
//        //Tucked -> Flipped: elbow outtake
//        //Tucked -> ReadyI: slides down, untuck wrist, fingers release
//        //Flipped -> ReadyO: wrist outtake angle
//        //Flipped -> Tucked: elbow intake
//        //ReadyO -> Released: fingers release
//        //ReadyO -> Flipped: tuck wrist
//        //Released -> ReadyO: no op
//
//        if(currState != targetState ) {
//            getNextState();
//        } else {
//            nextState = currState;
//        }
//
//        switch(nextState) {
//            case STATE_1_RFI: // Ready for Intake = main state
//                targetSlidePos = SLIDE_INTAKE_POS;
//                updateSlide();
//                if(!bSlidesAtRest && slideTimer.seconds() > 1)
//                {
//                    bSlidesAtRest = true;
//                }
//                if(bSlidesAtRest) {
//                    Wrist.setPosition(WRIST_INTAKE);
//                    LeftFinger.setPosition(LEFT_FINGER_UNLOCK); //unlock
//                    RightFinger.setPosition(RIGHT_FINGER_UNLOCK); //unlock
//                    Elbow.setPosition(ELBOW_DOWN);
//                    currState = nextState;
//                    if (currState == targetState) {
//                        if (timer.milliseconds() > 500) { // TODO: Replce this with Color sensor logic
//                            //color sensor input = pixel in
//                            setTargetState(ioState.STATE_3_PS);
//                            bSlidesAtRest = true;
//                        }
//                    }
//                }
//                break;
//            case STATE_2_ITA: // Intake tucked
//                targetSlidePos = SLIDE_INTAKE_POS;
//                updateSlide();
//                Wrist.setPosition(WRIST_TUCK);
//                LeftFinger.setPosition(LEFT_FINGER_UNLOCK); //unlock
//                RightFinger.setPosition(RIGHT_FINGER_UNLOCK); //unlock
//                Elbow.setPosition(ELBOW_DOWN);
//                currState = nextState;
//                break;
//            case STATE_3_PS: // Pixels secured = main state
//
//                Wrist.setPosition(WRIST_TUCK);
//                LeftFinger.setPosition(LEFT_FINGER_LOCK); //lock
//                RightFinger.setPosition(RIGHT_FINGER_LOCK); //lock
//                Elbow.setPosition(ELBOW_DOWN);
//                currState = nextState;
////                if(!goingUp) {
////                    targetSlidePos = SLIDE_INTAKE_POS;
////                }
//                updateSlide();
//
//                break;
//            case STATE_4_BF: // Box Flipped
//                targetSlidePos = SLIDE_FLIP_HEIGHT;
//                updateSlide();
//                if(slideTimer.seconds() > 1)
//                {
//                    currState = nextState;
//                    slideTimer.reset();
//                }
////                if(bSlidesAtRest) {
////                    Wrist.setPosition(WRIST_TUCK);
////                    LeftFinger.setPosition(LEFT_FINGER_LOCK); //lock
////                    RightFinger.setPosition(RIGHT_FINGER_LOCK); //lock
////                    Elbow.setPosition(ELBOW_DOWN);
////                    currState = nextState;
////                    timer.reset();
////                }
//                break;
//            case STATE_5_RFO_LOW: // Ready for Outtake
//                targetSlidePos = SLIDE_RAISE_LOW;
//                updateSlide();
//                Wrist.setPosition(WRIST_TUCK);
//                Elbow.setPosition(ELBOW_UP);
//                currState = nextState;
//                break;
//            case STATE_5_RFO_MID: // Ready for Outtake
//                targetSlidePos = SLIDE_RAISE_MED;
//                updateSlide();
//                Wrist.setPosition(WRIST_TUCK);
//                Elbow.setPosition(ELBOW_UP);
//                currState = nextState;
//                break;
//            case STATE_5_RFO_HIGH: // Ready for Outtake
//                targetSlidePos = SLIDE_RAISE_HIGH;
//                updateSlide();
//                Wrist.setPosition(WRIST_TUCK);
//                Elbow.setPosition(ELBOW_UP);
//                currState = nextState;
//                break;
//            case STATE_5_RFO_MANUAL: // Ready for Outtake = main state in manual
//                updateSlide();
//                Wrist.setPosition(WRIST_TUCK);
//                Elbow.setPosition(ELBOW_UP);
//                currState = nextState;
//                break;
//            case STATE_6_PR_BOTH: // Pixel Release = main state in auto
//                updateSlide();
//                Wrist.setPosition(WRIST_TUCK);
//                Elbow.setPosition(ELBOW_UP);
//                LeftFinger.setPosition(LEFT_FINGER_UNLOCK); //unlock
//                RightFinger.setPosition(RIGHT_FINGER_UNLOCK); //unlock
//                currState = nextState;
//                break;
//            default:
//                break;
//        }
//
//        updateSlide();
////
////        if(currSlidePos != targetSlidePos) {
////            double command = liftController.output(targetSlidePos, SlideMotor.getCurrentPosition());
////            double slidePower = (targetSlidePos < currSlidePos) ?
////                    Math.max(command / HighJunction, SlidePower_Down) :
////                    Math.min(command / HighJunction, SlidePower_Up);
////            jerry.setPower(slidePower);
////            Slide.setPower(slidePower);
////            currSlidePos = SlideMotor.getCurrentPosition();
////        }
////        if(telemetry != null) {
////            telemetry.addData("Mvrk_LiftController: Current Slide position: %f", currSlidePos);
////            telemetry.update();
////        }
//    }

    public void update()
    {

        if(currState == targetState)
        {
            updateSlide();
            //updateServo();

            if (currState == ioState.STATE_1_RFI && flipTimer.milliseconds() > 2000) {
                // TODO: Replce this with Color sensor logic
                // TODO: GO TO state 3 if TWO PIXELS ARE LOADED.
                // TODO: OTHERWISE, REACT TO MANUAL OVERRIDE
                //color sensor input = pixel in
                setTargetState(ioState.STATE_3_PS);
            }

            return;
        }

        getNextState();

        switch(nextState) {
            case STATE_1_RFI:

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
                currState = nextState;

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

            case STATE_5_RFO_LOW: // Ready for Outtake
                targetSlidePos = SLIDE_RAISE_LOW;
                currState = nextState;
                break;

            case STATE_5_RFO_MID: // Ready for Outtake
                targetSlidePos = SLIDE_RAISE_MED;
                currState = nextState;
                break;

            case STATE_5_RFO_HIGH: // Ready for Outtake
                targetSlidePos = SLIDE_RAISE_HIGH;
                currState = nextState;
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

        if( currState == ioState.STATE_1_RFI ) {
            if(targetState == ioState.STATE_3_PS
             ) {
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_3_PS) {
            if(targetState == ioState.STATE_1_RFI || targetState == ioState.STATE_5_RFO_MANUAL || targetState == ioState.STATE_5_RFO_MANUAL){
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_5_RFO_MID) {
            if(targetState == ioState.STATE_6_PR_BOTH){
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_5_RFO_MANUAL) {
            if(targetState == ioState.STATE_1_RFI){
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
