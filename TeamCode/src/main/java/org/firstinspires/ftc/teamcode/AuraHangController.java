package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.AuraRobot.HANG_FLIPPER_DOWN;
import static org.firstinspires.ftc.teamcode.AuraRobot.HANG_FLIPPER_UP;
import static org.firstinspires.ftc.teamcode.AuraRobot.HANG_EXTENDER_EXTEND;
import static org.firstinspires.ftc.teamcode.AuraRobot.HANG_EXTENDER_RETRACT;
import static org.firstinspires.ftc.teamcode.AuraRobot.HANG_MOTOR_TICKS;
import static org.firstinspires.ftc.teamcode.AuraRobot.HANG_NUM_MOTOR_ROTATIONS;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class AuraHangController {
    public Servo hangExtender;

    public Servo hangFlipper;

    private DcMotor hangMotor;
    enum HangState
    {
        Idle,
        Hang,
        Up
    }
    AuraHangController.HangState currState;
    AuraHangController.HangState targetState;
    Telemetry telemetry;

    public AuraHangController(HardwareMap hardwareMap) {
        // TODO: Configure these right and flip it correctly.
        hangFlipper = hardwareMap.get(Servo.class, "Extend");
        hangExtender = hardwareMap.get(Servo.class, "Funky");
        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        currState = AuraHangController.HangState.Idle;
        targetState = HangState.Idle;
    }

    public void init(){
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setTelemetry(Telemetry tele)
    {
        telemetry = tele;
    }

    public void setTargetState(AuraHangController.HangState state) {
        targetState = state;
    }

    public void update() {
        //  Open -> Open: No-op
        //  Open -> Close: Close, update curr
        //  Open -> Auto: Set in Auto, update curr
        //  Close -> Open: Open, update curr
        //  Close->Close: No-op
        //  Close->Auto: set in Auto, update curr
        //  Auto->Open: Open, update curr
        //  Auto->Close: Close, update curr
        //  Auto->Auto : Auto, update curr
        if (currState != HangState.Idle && currState == targetState)
            return;

        switch (targetState) {
            case Hang:
//                telemetry.addData("MOTOR TICKS",(int)Math.floor(numRotations * motorTicks));
                hangMotor.setTargetPosition((int)Math.floor(HANG_NUM_MOTOR_ROTATIONS * HANG_MOTOR_TICKS));
                hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hangMotor.setPower(1);
                currState = AuraHangController.HangState.Hang;
                break;
            case Up:
                hangExtender.setPosition(HANG_EXTENDER_EXTEND);
                hangFlipper.setPosition(HANG_FLIPPER_UP);
                currState = HangState.Up;
                break;
            case Idle:
                hangExtender.setPosition(HANG_EXTENDER_RETRACT);
                hangFlipper.setPosition(HANG_FLIPPER_DOWN);
                currState = AuraHangController.HangState.Idle;
                break;
            }

        return;
    }
}