package org.firstinspires.ftc.teamcode;
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
    private Servo   Wrist;
    private Servo   Elbow;
    private Servo   LeftFinger;
    private Servo   RightFinger;

    public RevColorSensorV3 Left = null;

    public ColorRangeSensor Right = null;

    public enum ioState {
        STATE_1_RFI,    // State 1: Ready for Intake
        STATE_2_ITA,     // State 2: Pixels Secured
        STATE_3_PS,    // State 3: Intake tucked away (slides raised to minimum)
        STATE_4_BF,     // State 4: Box Flipped
        STATE_5_RFO_LOW,    // State 5: Ready for Outtake
        STATE_5_RFO_MID,
        STATE_5_RFO_HIGH,
        STATE_5_RFO_MANUAL,
        STATE_6_PR,      // State 6: Pixels Released
    }

    ioState currState;
    ioState targetState;
    public static double targetSlidePos;
    double currSlidePos;
    public static AuraServoPIDController servoController;
    private Telemetry telemetry;

    public boolean safeToUnload = false;

    //TODO: change numbers
    public static AuraPIDController slideUpPID = new AuraPIDController(12, 0, 0, 6); // KD Values .25 -> .32 KG Previous Values 3600 -> 5500 2/19/2023
    public static AuraPIDController slideDownPID = new AuraPIDController(12, 0, 0, 0);

    public AuraIntakeOuttakeController(HardwareMap hardwareMap) {

        Wrist = hardwareMap.get(Servo.class, "tilt");
        Elbow = hardwareMap.get(Servo.class, "flip");
        LeftFinger = hardwareMap.get(Servo.class, "lefty");
        RightFinger = hardwareMap.get(Servo.class, "righty");
        Slide = hardwareMap.get(DcMotor.class, "Slide");

        Left = hardwareMap.get(RevColorSensorV3.class, "Left");
        Right = hardwareMap.get(ColorRangeSensor.class,"Right");

        currState = ioState.STATE_1_RFI;
        targetState = ioState.STATE_1_RFI;
        currSlidePos = Slide.getCurrentPosition();
        targetSlidePos = SLIDE_INTAKE_POS;
    }

    public void init(){
        Left.initialize();
        Left.enableLed(true);
        Right.enableLed(true);
        Left.getLightDetected();
    }

    public void setTelemetry(Telemetry tele) {
        telemetry = tele;
    }

    public void setTargetPosition(double target) {
        if(target >= LowerLimit && target <= UpperLimit)
            targetSlidePos = target;
    }

    public void setTargetState(ioState eState)
    {
        targetState = eState;
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

    public void update() {

        //ReadyI -> Secured: fingers lock
        //ReadyI -> Tucked: slides up, tuck  wrist
        //Secured -> Tucked: slides up, tuck wrist
        //Tucked -> Flipped: elbow outtake
        //Tucked -> ReadyI: slides down, untuck wrist, fingers release
        //Flipped -> ReadyO: wrist outtake angle
        //Flipped -> Tucked: elbow intake
        //ReadyO -> Released: fingers release
        //ReadyO -> Flipped: tuck wrist
        //Released -> ReadyO: no op

//        if(currState == targetState && targetState != ioState.STATE_1_RFI )
//            return;


        if(!validStateTransition())
            return;

        switch(targetState) {
            case STATE_1_RFI: // Ready for Intake
                targetSlidePos = SLIDE_INTAKE_POS;
                updateSlide();
                Wrist.setPosition(WRIST_INTAKE);
                LeftFinger.setPosition(LEFT_FINGER_UNLOCK); //unlock
                RightFinger.setPosition(RIGHT_FINGER_UNLOCK); //unlock
                Elbow.setPosition(ELBOW_DOWN);
                currState = targetState;
                safeToUnload = false;
                break;
            case STATE_2_ITA: // Intake tucked
                targetSlidePos = SLIDE_INTAKE_POS;
                updateSlide();
                Wrist.setPosition(WRIST_TUCK);
                LeftFinger.setPosition(LEFT_FINGER_UNLOCK); //unlock
                RightFinger.setPosition(RIGHT_FINGER_UNLOCK); //unlock
                Elbow.setPosition(ELBOW_DOWN);
                currState = targetState;
                safeToUnload = false;
                break;
            case STATE_3_PS: // Pixels secured
                targetSlidePos = SLIDE_FLIP_HEIGHT;
                updateSlide();
                Wrist.setPosition(WRIST_TUCK);
                LeftFinger.setPosition(LEFT_FINGER_LOCK); //lock
                RightFinger.setPosition(RIGHT_FINGER_LOCK); //lock
                Elbow.setPosition(ELBOW_DOWN);
                currState = targetState;
                safeToUnload = false;
                break;
            case STATE_4_BF: // Box Flipped
                targetSlidePos = SLIDE_FLIP_HEIGHT;
                updateSlide();
                Wrist.setPosition(WRIST_TUCK);
                Elbow.setPosition(ELBOW_UP);
                currState = targetState;
                safeToUnload = false;
                break;
            case STATE_5_RFO_LOW: // Ready for Outtake
                targetSlidePos = SLIDE_RAISE_LOW;
                updateSlide();
                Wrist.setPosition(WRIST_TUCK);
                Elbow.setPosition(ELBOW_UP);
                LeftFinger.setPosition(LEFT_FINGER_LOCK); //unlock
                RightFinger.setPosition(RIGHT_FINGER_LOCK); //unlock
                currState = targetState;
                safeToUnload = false;
                break;
            case STATE_5_RFO_MID: // Ready for Outtake
                targetSlidePos = SLIDE_RAISE_MED;
                updateSlide();
                Wrist.setPosition(WRIST_TUCK);
                Elbow.setPosition(ELBOW_UP);
                LeftFinger.setPosition(LEFT_FINGER_LOCK); //unlock
                RightFinger.setPosition(RIGHT_FINGER_LOCK); //unlock
                currState = targetState;
                safeToUnload = false;
                break;
            case STATE_5_RFO_HIGH: // Ready for Outtake
                targetSlidePos = SLIDE_RAISE_HIGH;
                updateSlide();
                Wrist.setPosition(WRIST_TUCK);
                Elbow.setPosition(ELBOW_UP);
                LeftFinger.setPosition(LEFT_FINGER_LOCK); //unlock
                RightFinger.setPosition(RIGHT_FINGER_LOCK); //unlock
                currState = targetState;
                safeToUnload = false;
                break;
            case STATE_5_RFO_MANUAL: // Ready for Outtake
                updateSlide();
                Wrist.setPosition(WRIST_TUCK);
                Elbow.setPosition(ELBOW_UP);
                LeftFinger.setPosition(LEFT_FINGER_LOCK); //unlock
                RightFinger.setPosition(RIGHT_FINGER_LOCK); //unlock
                currState = targetState;
                safeToUnload = false;
                break;
            case STATE_6_PR: // Pixel Release
                Wrist.setPosition(WRIST_TUCK);
                Elbow.setPosition(ELBOW_UP);
                LeftFinger.setPosition(LEFT_FINGER_UNLOCK); //unlock
                RightFinger.setPosition(RIGHT_FINGER_UNLOCK); //unlock
                currState = targetState;
                safeToUnload = true;
                break;
            default:
                break;
        }
//
//        if(currSlidePos != targetSlidePos) {
//            double command = liftController.output(targetSlidePos, SlideMotor.getCurrentPosition());
//            double slidePower = (targetSlidePos < currSlidePos) ?
//                    Math.max(command / HighJunction, SlidePower_Down) :
//                    Math.min(command / HighJunction, SlidePower_Up);
//            jerry.setPower(slidePower);
//            Slide.setPower(slidePower);
//            currSlidePos = SlideMotor.getCurrentPosition();
//        }
//        if(telemetry != null) {
//            telemetry.addData("Mvrk_LiftController: Current Slide position: %f", currSlidePos);
//            telemetry.update();
//        }
    }

    // Ensure that this state transitoin is valid



    public boolean validStateTransition()
    {

        if (targetState == currState ) {
            return true;
        }

        if( currState == ioState.STATE_1_RFI ) {
            if(targetState == ioState.STATE_2_ITA
            //&& color sensors say so
             ) {
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_2_ITA) {
            if(targetState == ioState.STATE_1_RFI || targetState == ioState.STATE_3_PS) {
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_3_PS) {
            if(targetState == ioState.STATE_4_BF || targetState == ioState.STATE_2_ITA){
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_4_BF ) {
            if(targetState == ioState.STATE_5_RFO_LOW || targetState == ioState.STATE_5_RFO_MID || targetState == ioState.STATE_5_RFO_HIGH || targetState == ioState.STATE_5_RFO_MANUAL ||targetState == ioState.STATE_3_PS){
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_5_RFO_LOW) {
            if(targetState == ioState.STATE_6_PR || targetState == ioState.STATE_5_RFO_MID || targetState == ioState.STATE_5_RFO_HIGH || targetState == ioState.STATE_5_RFO_MANUAL || targetState == ioState.STATE_4_BF ){
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_5_RFO_MID) {
            if(targetState == ioState.STATE_6_PR || targetState == ioState.STATE_5_RFO_LOW || targetState == ioState.STATE_5_RFO_HIGH || targetState == ioState.STATE_5_RFO_MANUAL || targetState == ioState.STATE_4_BF ){
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_5_RFO_HIGH) {
            if(targetState == ioState.STATE_6_PR || targetState == ioState.STATE_5_RFO_MID || targetState == ioState.STATE_5_RFO_LOW || targetState == ioState.STATE_5_RFO_MANUAL || targetState == ioState.STATE_4_BF ){
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_5_RFO_MANUAL) {
            if(targetState == ioState.STATE_6_PR || targetState == ioState.STATE_5_RFO_MID || targetState == ioState.STATE_5_RFO_HIGH || targetState == ioState.STATE_5_RFO_LOW || targetState == ioState.STATE_4_BF ){
                return true;
            }
            return false;
        }

        if( currState == ioState.STATE_6_PR ) {
            if( targetState == ioState.STATE_4_BF){
                return true;
            }
            return false;
        }

        return false;
    }

}
