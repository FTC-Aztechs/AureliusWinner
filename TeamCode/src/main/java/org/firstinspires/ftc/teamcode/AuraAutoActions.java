package org.firstinspires.ftc.teamcode;

public class AuraAutoActions {
    //TODO: look at sample action class here and write your own
//    public class Shooter {
//        private DcMotorEx motor;
//
//        public Shooter(HardwareMap hardwareMap) {
//            motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
//        }
//
//        public class SpinUp implements Action {
//            @Override
//            public void init() {
//                motor.setPower(0.8);
//            }
//
//            @Override
//            public boolean loop(TelemetryPacket packet) {
//                double vel = motor.getVelocity();
//
//                packet.put("shooterVelocity", vel);
//
//                return vel < 10_000.0;
//            }
//        }
//
//        public Action spinUp() {
//            return new SpinUp();
//        }
//    }
//
//    public class ShooterOpMode extends ActionOpMode {
//        @Override
//        public void runOpMode() throws InterruptedException {
//            Shooter shooter = new Shooter(hardwareMap);
//
//            waitForStart();
//
//            runBlocking(shooter.spinUp());
//        }
//    }

//    import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
//import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
//
//    public class MoveLiftSlideActionFinishImmediate implements Action {
//        //Declare and set a timeout threshold for the command called TIMEOUT_TIME_SECONDS - I suggest 1.5 seconds for now
//        final double TIMEOUT_TIME_SECONDS = 3;
//
//        //Declare currentTicks and targetTicks for use locally
//        int targetTicks;
//        int currentTicks;
//        private final LiftSlideSubsystem.LiftStates targetState;
//
//        private boolean hasNotInit = true;
//        private boolean finished = false;
//
//        //declare a timeoutTimer (type ElapsedTime) to timeout the command if it doesn't finish
//        private ElapsedTime timeoutTimer;
//
//        //declare a timeout boolean
//        boolean timeout;
//
//        public MoveLiftSlideActionFinishImmediate(LiftSlideSubsystem.LiftStates inputState) {
//            targetState = inputState;
//            timeoutTimer = new ElapsedTime();
//        }
//
//        public void init() {
//            hasNotInit=false;
//            //When the command is first run set the targetState of the subsystem to the targetState and set the target ticks to the target ticks of that state
//            Robot.getInstance().getLiftSlideSubsystem().setTargetState(targetState);
//            Robot.getInstance().getLiftSlideSubsystem().setTargetTicks(Robot.getInstance().getLiftSlideSubsystem().getTargetState().ticks);
//
//            //reset the timer
//            timeoutTimer.reset();
//            //set the timeout to false since we have not timed out yet
//            timeout=false;
//
//            //get the currentTicks and the targetTicks from the subsystem
//            currentTicks = Robot.getInstance().getLiftSlideSubsystem().getCurrentTicks();
//            targetTicks = Robot.getInstance().getLiftSlideSubsystem().getTargetTicks();
//
//            //Check if targetTicks is greater than MAX_TARGET_TICKS and if it is set the target to the max
//            //This makes sure that if we accidentally put a very large number as our target ticks we don't break the robot
//            if (targetTicks> Robot.getInstance().getLiftSlideSubsystem().MAX_TARGET_TICKS)
//            {
//                targetTicks=Robot.getInstance().getLiftSlideSubsystem().MAX_TARGET_TICKS;
//            }
//
//            //Check if targetTicks is lower than MIN_TARGET_TICKS and if it is set the target to the min
//            //This makes sure that if we accidentally put a very low negative number as our target ticks we don't break the robot
//            if (targetTicks < Robot.getInstance().getLiftSlideSubsystem().MIN_TARGET_TICKS)
//            {
//                targetTicks=Robot.getInstance().getLiftSlideSubsystem().MIN_TARGET_TICKS;
//            }
//
//            //if the target ticks are higher than the current ticks, then use EXTENSION_POWER
//            if (targetTicks > currentTicks) {
//                Robot.getInstance().getLiftSlideSubsystem().liftSlide.setPower(LiftSlideSubsystem.liftSlideParameters.EXTENSION_LIFT_POWER);
//            }
//
//            //if the target ticks are lower than the current ticks, then use RETRACTION_POWER
//            if (targetTicks < currentTicks) {
//                Robot.getInstance().getLiftSlideSubsystem().liftSlide.setPower(LiftSlideSubsystem.liftSlideParameters.RETRACTION_LIFT_POWER);
//            }
//
//            //Set the target position using the targetTicks
//            Robot.getInstance().getLiftSlideSubsystem().liftSlide.setTargetPosition(targetTicks);
//
//            //set the lift motor to RUN TO POSITION - this might not be necessary
//            Robot.getInstance().getLiftSlideSubsystem().liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (hasNotInit) init();
//
//            //This action is empty because we don't need to do anything while the action is running
//            //the RUN_TO_POSITION we set in the init means that the lift motor is going to automatically try to reach the target
//
//            return false;
//        }
//
//    }


}
