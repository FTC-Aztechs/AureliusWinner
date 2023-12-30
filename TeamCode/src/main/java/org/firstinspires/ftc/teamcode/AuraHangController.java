package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.AuraRobot.FunkyIdle;
import static org.firstinspires.ftc.teamcode.AuraRobot.FunkyUp;
import static org.firstinspires.ftc.teamcode.AuraRobot.HangExtend;
import static org.firstinspires.ftc.teamcode.AuraRobot.HangIdle;
import static org.firstinspires.ftc.teamcode.AuraRobot.motorTicks;
import static org.firstinspires.ftc.teamcode.AuraRobot.numRotations;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class AuraHangController {
    public Servo hanger;

    public Servo funky;

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
        hanger = hardwareMap.get(Servo.class, "Extend");
        funky = hardwareMap.get(Servo.class, "Funky");
        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        currState = AuraHangController.HangState.Idle;
        targetState = HangState.Idle;
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
                hangMotor.setTargetPosition((int)Math.floor(numRotations * motorTicks));
                hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hangMotor.setPower(1);
                currState = AuraHangController.HangState.Hang;
                break;
            case Up:
                hanger.setPosition(HangExtend);
                funky.setPosition(FunkyUp);
                currState = HangState.Up;
                break;
            case Idle:
                hanger.setPosition(HangIdle);
                funky.setPosition(FunkyIdle);
                currState = AuraHangController.HangState.Idle;
                break;
            }

        return;
    }
}