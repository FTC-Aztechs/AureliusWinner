package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AuraRobot.Launcher_Fire_Pos;
import static org.firstinspires.ftc.teamcode.AuraRobot.Launcher_Set_Pos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class AuraLaunchController {
    public Servo launcher;
    enum launchState
    {
        Set,
        Launch
    }
    launchState currState;
    launchState targetState;
    Telemetry telemetry;

    public AuraLaunchController(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(Servo.class, "Launcher");
        currState = launchState.Set;
    }

    public void init(){
        launcher.setPosition(Launcher_Set_Pos);
    }

    public void setTelemetry(Telemetry tele)
    {
        telemetry = tele;
    }

    public void setTargetState(launchState state) {
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
            case Set:
                launcher.setPosition(Launcher_Set_Pos);
                currState = launchState.Set;
                break;
            case Launch:
                launcher.setPosition(Launcher_Fire_Pos);
                currState = launchState.Launch;
                break;
        }

        return;
    }
}