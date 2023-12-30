package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class AuraIntakeController {
    private DcMotor intakeMotor;

    enum intakeState
    {
        IN,
        OUT
    }
    intakeState currState;
    intakeState targetState;
    Telemetry telemetry;

    public AuraIntakeController(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        currState = intakeState.OUT;
    }

    public void setTelemetry(Telemetry tele)
    {
        telemetry = tele;
    }

    public void setTargetState(intakeState state) {
        targetState = state;
    }

    public void update(float speed) {
        //  Open -> Open: No-op
        //  Open -> Close: Close, update curr
        //  Open -> Open: No-op
        //  Open -> Close: Close, update curr

        if (currState == targetState)
            return;

        switch (targetState) {
            case OUT:
                intakeMotor.setPower(speed * 0.8);
                currState = intakeState.OUT;
                break;
            case IN:
                intakeMotor.setPower(-speed);
                currState = intakeState.IN;
                break;
        }

        if (currState == targetState)
            return;

        switch (targetState) {
            case OUT:
                intakeMotor.setPower(speed);
                currState = intakeState.OUT;
                break;
            case IN:
                intakeMotor.setPower(-speed);
                currState = intakeState.IN;
                break;
        }
    }
}