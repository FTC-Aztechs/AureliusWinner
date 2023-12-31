package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.AuraRobot.Deposit_Down_Pos;
import static org.firstinspires.ftc.teamcode.AuraRobot.Deposit_Up_Pos;
import static org.firstinspires.ftc.teamcode.AuraRobot.Lid_Close_Pos;
import static org.firstinspires.ftc.teamcode.AuraRobot.Lid_Open_Pos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@Config
public class Aura_DepositController {
    public Servo Deposit;
    public Servo Lid;

    public enum DepositState {
        Up,
        Open,
        Down
    }

    DepositState currState;
    DepositState targetState;
    Telemetry telemetry;

    public Aura_DepositController(HardwareMap hardwareMap) {
        Deposit = hardwareMap.get(Servo.class, "DepositServo");
        Lid = hardwareMap.get(Servo.class, "LidServo");
        Deposit.setPosition(Deposit_Down_Pos);
        Lid.setPosition(Lid_Close_Pos);
        currState = DepositState.Down;
    }

    public void setTelemetry(Telemetry tele)
    {
        telemetry = tele;
    }

    public void setTargetState(DepositState state) {
        targetState = state;
    }

    public void update()
    {
        //  Open -> Open: No-op
        //  Open -> Close: Close, update curr
        //  Open -> Auto: Set in Auto, update curr
        //  Close -> Open: Open, update curr
        //  Close->Close: No-op
        //  Close->Auto: set in Auto, update curr
        //  Auto->Open: Open, update curr
        //  Auto->Close: Close, update curr
        //  Auto->Auto : Auto, update curr
        if(currState == targetState)
            return;

        switch(targetState) {
            case Down:
                Deposit.setPosition(Deposit_Down_Pos);
                Lid.setPosition(Lid_Close_Pos);
                currState = Aura_DepositController.DepositState.Down;
                break;
            case Up:
                Deposit.setPosition(Deposit_Up_Pos);
                currState = Aura_DepositController.DepositState.Up;
                break;
            case Open:
                Lid.setPosition(Lid_Open_Pos);
                currState = Aura_DepositController.DepositState.Open;
                break;
        }
        return;
    }
}