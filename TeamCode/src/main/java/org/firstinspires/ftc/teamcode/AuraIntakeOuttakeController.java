package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.AuraRobot.LowerLimit;
import static org.firstinspires.ftc.teamcode.AuraRobot.SlidePower_Down;
import static org.firstinspires.ftc.teamcode.AuraRobot.SlidePower_Up;
import static org.firstinspires.ftc.teamcode.AuraRobot.UpperLimit;

import com.acmerobotics.dashboard.config.Config;
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
//      - Wrist @ Intake angle
//      * Pixel grabbers secured
//      - Elbow @ Intake
// State 3: Intake tucked away (slides raised to minimum)
//      * Slides @ minimum raised pos
//      * Wrist @ Tuck In angle
//      - Pixel grabbers in previous state (note: can be secured or unsecured depending on what state it is returning from)
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

    enum ioState {
        STATE_1_RFI,    // State 1: Ready for Intake
        STATE_2_PS,     // State 2: Pixels Secured
        STATE_3_ITA,    // State 3: Intake tucked away (slides raised to minimum)
        STATE_4_BF,     // State 4: Box Flipped
        STATE_5_RFO,    // State 5: Ready for Outtake
        STATE_6_PR      // State 6: Pixels Released
    }

    ioState currState;
    ioState targetState;
    double targetSlidePos;
    double currSlidePos;
    double SlidePower;

    public static double FINGERS_LOCK = 1;
    public static double FINGERS_UNLOCK = 0;
    public static double WRIST_INTAKE = 0.5;
    public static double WRIST_OUTTAKE = 0.5;
    public static double WRIST_TUCK = 0.8;
    public static double ELBOW_DOWN = 0.025;
    public static double ELBOW_UP = 0.55;
    public static double SLIDE_INTAKE_POS = 100;
    public static double SLIDE_RAISE_LOW = 250;
    public static double SLIDE_RAISE_MED = 700;
    public static double SLIDE_RAISE_HIGH = 1000;
    public static AuraServoPIDController servoController;
    private Telemetry telemetry;

    //TODO: change numbers
    public static AuraPIDController slideUpPID = new AuraPIDController(11, 0, 0, 5500); // KD Values .25 -> .32 KG Previous Values 3600 -> 5500 2/19/2023
    public static AuraPIDController slideDownPID = new AuraPIDController(0.8, 0, 0, 1500);

    public AuraIntakeOuttakeController(HardwareMap hardwareMap) {

        Wrist = hardwareMap.get(Servo.class, "tilt");
        Elbow = hardwareMap.get(Servo.class, "flip");
        LeftFinger = hardwareMap.get(Servo.class, "lefty")
        RightFinger = hardwareMap.get(Servo.class, "righty")
        Slide = hardwareMap.get(DcMotor.class, "slide");

        currState = ioState.STATE_1_RFI;
        targetState = ioState.STATE_1_RFI;

        targetSlidePos = SLIDE_INTAKE_POS;
        currSlidePos = SLIDE_INTAKE_POS;
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
            double command = 0;
            if (targetSlidePos < currSlidePos) {
                command = slideDownPID.output(targetSlidePos, Slide.getCurrentPosition());
                SlidePower = Math.max(command / (SLIDE_RAISE_HIGH - SLIDE_INTAKE_POS), SlidePower_Down);
            } else {
                command = Mvrk_Robot.slideUpPID.output(targetSlidePos, Slide.getCurrentPosition());
                SlidePower = Math.min(command / (SLIDE_RAISE_HIGH - SLIDE_INTAKE_POS), SlidePower_Up);
            }
            Slide.setPower(SlidePower);
            currSlidePos = Slide.getCurrentPosition();
        }
        if(telemetry != null) {
            telemetry.addData("AuraIOController: Current Slide position: %f", currPos);
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

        if( currState == targetState )
            return;

        if(!validStateTransition())
            return;

        switch(targetState) {
            case STATE_1_RFI:
                targetSlidePos = SLIDE_INTAKE_POS;
                updateSlide();
                Wrist.setPosition(wristIntake);
                LeftFinger.SetPosition(fingersUnlock); //unlock
                RightFinger.SetPosition(fingersUnlock); //unlock
                Elbow.setPosition(elbowDown);
                currState = targetState;
                break;
            case STATE_2_PS:
                targetSlidePos = SLIDE_INTAKE_POS;
                updateSlide();
                Wrist.setPosition(wristIntake);
                LeftFinger.SetPosition(fingersLock); //lock
                RightFinger.SetPosition(fingersLock); //lock
                Elbow.setPosition(elbowDown);
                currState = targetState;
                break;
            case STATE_3_ITA:
                targetSlidePos = SLIDE_RAISE_LOW;
                updateSlide();
                Wrist.setPosition(wristTuck);
                Elbow.setPosition(elbowDown);
                currState = targetState;
                break;
            case STATE_4_BF:
                targetSlidePos = SLIDE_RAISE_LOW;
                updateSlide();
                Wrist.setPosition(wristTuck);
                Elbow.setPosition(elbowUp);
                currState = targetState;
                break;
            case STATE_5_RFO:
                targetSlidePos = SLIDE_RAISE_LOW;
                updateSlide();
                Wrist.setPosition(wristOutake);
                Elbow.setPosition(elbowUp);
                currState = targetState;
                break;
            case STATE_6_PR:
                targetSlidePos = SLIDE_RAISE_LOW;
                updateSlide();
                Wrist.setPosition(wristOutake);
                Elbow.setPosition(elbowUp);
                LeftFinger.SetPosition(fingersUnlock); //unlock
                RightFinger.SetPosition(fingersUnlock); //unlock
                currState = targetState;
                break;
            default:
                break;
        }

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

        if (targetSTate == currState ) {
            return true;
        }

        if( currState == ioState.STATE_1_RFI ) {
            if(targetState == ioState.STATE_2_PS || targetState == ioState.STATE_3_ITA) {
                return true;
            }
            return  false;
        }

        if( currState == ioState.STATE_2_PS) {
            if(targetState == ioState.STATE_3_ITA) {
                return =true;
            }
            return = false;
        }

        if( currState == ioState.STATE_3_ITA ) {
            if(targetState == ioState.STATE_4_BF || targetState == ioState.STATE_1_RFI ){
                return =true;
            }
            return = false;
        }

        if( currState == ioState.STATE_4_BF ) {
            if(targetState == ioState.STATE_5_RFO || targetState == ioState.STATE_3_ITA){
                return =true;
            }
            return = false;
        }

        if( currState == ioState.STATE_5_RFO ) {
            if(targetState == ioState.STATE_6_PR || targetState == ioState.STATE_4_BF ){
                return =true;
            }
            return = false;
        }

        if( currState == ioState.STATE_6_PR ) {
            if( targetState == ioState.STATE_5_RFO ){
                return =true;
            }
            return = false;
        }

        return = false;
    }

}
