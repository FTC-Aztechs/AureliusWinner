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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

@Config
public class AuraRobot
{
    public enum AuraMotors
    {
        UPPER_LEFT,
        LOWER_LEFT,
        UPPER_RIGHT,
        LOWER_RIGHT,
        INTAKE,
        SLIDE,
        ROLLER,
        HANG,
        ALL_DRIVES,
        ALL_ATTACHMENTS,
        ALL
    }

    /* Public OpMode members. */
    public DcMotor Upper_Right = null;
    public DcMotor Upper_Left = null;
    public DcMotor Lower_Left = null;
    public DcMotor Lower_Right = null;

    public DcMotor Slide = null;
    public DcMotor intakeMotor = null;
    public CRServo Roller = null;


    public AuraIntakeController noodleWash;
    public AuraLaunchController boeing747;
    public AuraHangController hanger;
//    public Aura_DepositController depositFlipper;
    public AuraIntakeOuttakeController myIntakeController;
    public AuraHeadingEstimator myHeadingEstimator;

    public WebcamName Khimera = null;

    // speeds/times
    public static double UpAdjust = 10;
    public static double speedAdjust = 7;
    public static double bumperSpeedAdjust = 10;
    public static double dPadSpeedAdjust = 7;
    public static double dPadIntakeAdjust = 6;
    public static double SlidePower_Up= 1;
    public static double SlidePower_Down = -1;
    public static double SlidePower;
    public static int BUTTON_TRIGGER_TIMER_MS = 500;

    //dimensions for vuforia recognition
    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    public static final float halfField        = 72 * mmPerInch;
    public static final float halfTile         = 12 * mmPerInch;
    public static final float oneAndHalfTile   = 36 * mmPerInch;

    public static double motorTicks  = 537.7;
    public static double numRotations = 25;

    public RevColorSensorV3 Left = null;
//turn off color unless being used?
    public ColorRangeSensor Right = null;

    //claw variables
    public static AuraPIDController liftController = new AuraPIDController(12, 0, 0, 0);

    public static double Launcher_Set_Pos = 0;
    public static double Launcher_Fire_Pos = 1;

    public static double Lid_Close_Pos = 0.5;
    public  static double Lid_Open_Pos = 0.85;

    public static double Deposit_Down_Pos =  0.04;
    public static double Deposit_Up_Pos = 0.5;
    //Slide variables
    public static int LowerLimit = 0;
    public static double LEFT_FINGER_LOCK = 0.75;
    public static double RIGHT_FINGER_LOCK = 0.25;
    public static double LEFT_FINGER_UNLOCK = 0.45;
    public static double RIGHT_FINGER_UNLOCK = 0.5;
    public static double WRIST_INTAKE = 0.54;
    public static double WRIST_TUCK = 0.75;
    public static double ELBOW_DOWN = 0.023;
    public static double ELBOW_UP = 0.65;
    public static int SLIDE_INTAKE_POS = 25;
    public static int SLIDE_FLIP_HEIGHT = 680;
    public static int SLIDE_RAISE_LOW = 400;
    public static int SLIDE_RAISE_MED = 750;
    public static int SLIDE_RAISE_HIGH = 3000;
    public static double slideTicks_stepSize = 25;
    public static int FloorPosition  = 600;
    public static int HighJunction   = 15400; // 15400
    public static int HighJunction_Auto = 15250;
    public static int UpperLimit     = 3000;

    public static double HangExtend = 0.4;
    public static double HangIdle = 0; //0
    public static double FunkyIdle = 0.9;
    public static double FunkyUp = 0.57;
    public static double APRILTAG_TIMEOUT = 5;

   //------------------------------------------------------------

    /* local OpMode members. */
    public  HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public AuraRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        Upper_Right  = hwMap.get(DcMotor.class, "Upper_Right");
        Upper_Left = hwMap.get(DcMotor.class, "Upper_Left");
        Lower_Left = hwMap.get(DcMotor.class, "Lower_Left");
        Lower_Right = hwMap.get(DcMotor.class, "Lower_Right");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        Slide = hwMap.get(DcMotor.class, "Slide");
        Roller = hwMap.get(CRServo.class, "Roller");
        // Define and Initialize Color Sensors
        Left = hwMap.get(RevColorSensorV3.class, "Left");
        Right = hwMap.get(ColorRangeSensor.class,"Right");
        // Set all motors to zero power
        Upper_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Upper_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lower_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lower_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        Upper_Left.setDirection(DcMotor.Direction.REVERSE);  //-
        Upper_Right.setDirection(DcMotor.Direction.FORWARD); //+
        Lower_Left.setDirection(DcMotor.Direction.REVERSE); //- used to be
        Lower_Right.setDirection(DcMotor.Direction.FORWARD); //+ used to be

        intakeMotor.setDirection(DcMotor.Direction.FORWARD); //- used to be
        Slide.setDirection(DcMotorSimple.Direction.REVERSE);


        Pose2d initPose2d = new Pose2d(0,0,0);
        //AuraMecanumDrive = new MecanumDrive(hwMap, initPose2d);
//        Khimera = hwMap.get(WebcamName.class, "Kemera");

        noodleWash = new AuraIntakeController(hwMap);
        boeing747 = new AuraLaunchController(hwMap);
        hanger = new AuraHangController(hwMap);
//        depositFlipper = new Aura_DepositController(hwMap);
        myIntakeController = new AuraIntakeOuttakeController(hwMap);
    }
    String formatAngle( AngleUnit angleUnit, double angle) {
        return formatDegrees(angleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }


    public void setRunMode(AuraMotors eWhichMotor, DcMotor.RunMode eMode )
    {

        switch (eWhichMotor){
            case UPPER_LEFT:
                Upper_Left.setMode(eMode);
                break;
            case UPPER_RIGHT:
                Upper_Right.setMode(eMode);
                break;
            case LOWER_LEFT:
                Lower_Left.setMode(eMode);
                break;
            case LOWER_RIGHT:
                Lower_Right.setMode(eMode);
                break;

//            case CAT_MOUSE:
//                Jerry.setMode(eMode);
//                Tom.setMode(eMode);
//                break;
            case ALL_DRIVES:
                Lower_Right.setMode(eMode);
                Lower_Left.setMode(eMode);
                Upper_Right.setMode(eMode);
                Upper_Left.setMode(eMode);
                break;
            case ALL_ATTACHMENTS:
                break;
            case ALL:
//                Lower_Right.setMode(eMode);
//                Lower_Left.setMode(eMode);
//                Upper_Right.setMode(eMode);
//                Upper_Left.setMode(eMode);
//                Jerry.setMode(eMode);
//                Tom.setMode(eMode);
                break;
        }
    }

    public void setPower(AuraMotors eWhichMotor, double dPower)
    {

        switch (eWhichMotor){
            case UPPER_LEFT:
                Upper_Left.setPower(dPower);
                break;
            case UPPER_RIGHT:
                Upper_Right.setPower(dPower);
                break;
            case LOWER_LEFT:
                Lower_Left.setPower(dPower);
                break;
            case LOWER_RIGHT:
                Lower_Right.setPower(dPower);
                break;
            case INTAKE:
                intakeMotor.setPower(dPower);
                break;
            case ROLLER:
                Roller.setPower(dPower);
                break;
            case SLIDE:
                Slide.setPower(dPower);
                break;
            case ALL_DRIVES:
                Lower_Right.setPower(dPower);
                Lower_Left.setPower(dPower);
                Upper_Right.setPower(dPower);
                Upper_Left.setPower(dPower);
                break;
            case ALL:
            default:
                break;
        }
    }

    public int getCurrentPosition( AuraMotors eWhichMotor )
    {
        switch(eWhichMotor)
        {
            case UPPER_LEFT:
                return Upper_Left.getCurrentPosition();
            case LOWER_LEFT:
                return Lower_Left.getCurrentPosition();
            case UPPER_RIGHT:
                return Upper_Right.getCurrentPosition();
            case LOWER_RIGHT:
                return Lower_Right.getCurrentPosition();
            case SLIDE:
                return Slide.getCurrentPosition();
            default:
                return 0;
        }
    }



    public boolean areMotorsBusy(AuraMotors eWhichMotor) {

        switch(eWhichMotor)
        {
            case UPPER_LEFT: // upper left
                return Upper_Left.isBusy();
            case LOWER_LEFT: // lower left
                return Lower_Left.isBusy();
            case UPPER_RIGHT: // upper right
                return Upper_Right.isBusy();
            case LOWER_RIGHT: // lower right
                return Lower_Right.isBusy();
//            case CAT_MOUSE:
//                return Jerry.isBusy() && Tom.isBusy();
            case ALL_DRIVES: // All Drives
                return Lower_Left.isBusy() && Lower_Right.isBusy() && Upper_Left.isBusy() && Upper_Right.isBusy();
//            case ALL_ATTACHMENTS:
//                //return Linac.isBusy() && duck_wheel.isBusy() && Da_Winch.isBusy();
            case ALL:
//                return Lower_Left.isBusy() && Lower_Right.isBusy() && Upper_Left.isBusy() && Upper_Right.isBusy();
            default:
                return false;
        }
    }

    public void setTargetPosition(AuraMotors eWhichMotor, int iPos ) {
        switch (eWhichMotor) {
            case UPPER_LEFT:
                Upper_Left.setTargetPosition(iPos);
                break;
            case LOWER_LEFT:
                Lower_Left.setTargetPosition(iPos);
                break;
            case UPPER_RIGHT:
                Upper_Right.setTargetPosition(iPos);
                break;
            case LOWER_RIGHT:
                Lower_Right.setTargetPosition(iPos);
                break;
//            case CAT_MOUSE:
//                Jerry.setTargetPosition(iPos);
//                Tom.setTargetPosition(iPos);
//                break;
            case ALL_DRIVES:
                Lower_Right.setTargetPosition(iPos);
                Lower_Left.setTargetPosition(iPos);
                Upper_Right.setTargetPosition(iPos);
                Upper_Left.setTargetPosition(iPos);
                break;
            case ALL:
//                Lower_Right.setTargetPosition(iPos);
//                Lower_Left.setTargetPosition(iPos);
//                Upper_Right.setTargetPosition(iPos);
//                Upper_Left.setTargetPosition(iPos);
//                Jerry.setTargetPosition(iPos);
//                Tom.setTargetPosition(iPos);
            default:
                break;
        }
    }

}

