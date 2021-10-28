package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.util.InterpLUT; // gsehsch das?
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.Targets;
import org.firstinspires.ftc.teamcode.lib.VelocityPIDFController;

@Config
public class Shooter {
    // PID coefficients
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.0052, 0, 0.000016); //0.0038, 0, 0.000015

    // feedforward gains
    public static double kV = 0.00034;
    public static double kA = 0.000135;
    public static double kStatic = 0;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelocity = 0.0;

    private double currentVelocity = 0.0;

    // Our velocity controller
    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Define 3 states. on, off or coast
    public enum Mode {
        SHOOTING,
        IDLE,
        OFF
    }

    private enum FeederState {
        PUSHING,
        RETRACTING,
        RETRACTED,
        EXTENDED
    }

    private DcMotorEx shooter1, shooter2;

    private Servo feeder, flap;

    private Mode mode;

    private FeederState feederState;

    private final double actuationTime = 0.1;
    private final ElapsedTime feederTimer = new ElapsedTime();

    private final double feederStartPosition = 0.42;
    private final double feederExtendedPosition = 0.53;

    private final double flapRestPosition = 0.7;

    private double targetVelocity;
    private static double distance;

    //Init the Look up table
    InterpLUT lutHighgoal = new InterpLUT();
    InterpLUT lutPowershots = new InterpLUT();
    InterpLUT lutPowershotsAuto = new InterpLUT();


    public Shooter(HardwareMap hardwaremap) {
        // motors8
        shooter1 = hardwaremap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwaremap.get(DcMotorEx.class, "shooter2");

        //shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        //shooter2.setDirection(DcMotorEx.Direction.REVERSE);

        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // servos
        feeder = hardwaremap.get(Servo.class, "feeder");
        flap = hardwaremap.get(Servo.class, "flap");

        feeder.setPosition(feederStartPosition);
        flap.setPosition(flapRestPosition);
        feederState = FeederState.RETRACTED;

        mode = Mode.IDLE;

        // values for high goal lut
        lutHighgoal.add(-1000000, 0.495);
        lutHighgoal.add(65, 0.515);
        lutHighgoal.add(70, 0.52);
        lutHighgoal.add(75, 0.535);
        lutHighgoal.add(80, 0.543);
        lutHighgoal.add(85, 0.555);
        lutHighgoal.add(90, 0.57);
        lutHighgoal.add(95, 0.565);  //ursprünglich 0.568
        lutHighgoal.add(100, 0.575);
        lutHighgoal.add(105, 0.58);
        lutHighgoal.add(110, 0.585);
        lutHighgoal.add(115, 0.585);
        lutHighgoal.add(120, 0.59);
        lutHighgoal.add(125, 0.59);
        lutHighgoal.add(200000000, 0.59);

        //generating final equation for lutHighgoal
        lutHighgoal.createLUT();

        // same for power shots
        lutPowershots.add(-1000000, 0.5);
        lutPowershots.add(65, 0.52);
        lutPowershots.add(70, 0.525);
        lutPowershots.add(75, 0.535);
        lutPowershots.add(80, 0.547);
        lutPowershots.add(85, 0.56);
        lutPowershots.add(90, 0.57);
        lutPowershots.add(95, 0.573);
        lutPowershots.add(100, 0.575);
        lutPowershots.add(105, 0.58);
        lutPowershots.add(110, 0.59);
        lutPowershots.add(115, 0.59);
        lutPowershots.add(120, 0.595);
        lutPowershots.add(125, 0.595);
        lutPowershots.add(200000000, 0.6);

        //generating final equation for lutPowershots
        lutPowershots.createLUT();

        lutPowershotsAuto.add(-1000000, 0.4);
        lutPowershotsAuto.add(75, 0.45);//545
        lutPowershotsAuto.add(200000000, 0.4);

        //generating final equation for lutPowershotsAuto
        lutPowershotsAuto.createLUT();
    }

    public Mode getMode() {
        return mode;
    }

    public double getShooterVelocity() {
        return Globals.ticksPerSecondToRpm(currentVelocity, 1);
    }

    public double getTargetVelocity() {
        return Globals.ticksPerSecondToRpm(targetVelocity, 1);
    }

    public static void setDistance(double distance) {
        Shooter.distance = distance;
    }

    public void shoot() {
        // check if all requirements are met
        if(mode == Mode.SHOOTING && ((Globals.shots == 0 && targetVelocity * 0.94 <= currentVelocity && currentVelocity <= targetVelocity * 1.1) ||
                (Globals.shots != 0 && targetVelocity * 0.9 <= currentVelocity && currentVelocity <= targetVelocity * 1.05))) {
            if(feederState == FeederState.EXTENDED) {
                feeder.setPosition(feederStartPosition);
                feederState = FeederState.RETRACTING;
                feederTimer.reset();
            } else if (feederState == FeederState.RETRACTED){
                feeder.setPosition(feederExtendedPosition);
                feederState = FeederState.PUSHING;
                feederTimer.reset();
                Globals.shots++;
            }
        } else  { //if(feederState == FeederState.EXTENDED)
            feeder.setPosition(feederStartPosition);
            feederState = FeederState.RETRACTING;
            feederTimer.reset();
        }
    }

    public void Retract() {
        if(feederState == FeederState.EXTENDED) {
            feeder.setPosition(feederStartPosition);
            feederState = FeederState.RETRACTING;
            feederTimer.reset();
        }
    }

    public void forceExtend() {
        feeder.setPosition(feederExtendedPosition);
        feederState = FeederState.PUSHING;
        feederTimer.reset();
    }

    public void setMode(Mode mode) {
        this.mode = mode;
        if (mode == Mode.SHOOTING) {
            veloTimer.reset();
        }
    }

    public void update() {
        // Get the velocity from the motor with the encoder
        currentVelocity = shooter1.getVelocity();

        if(Globals.currentTargetType == Targets.TargetType.HIGHGOAL) {
            this.targetVelocity = Globals.rpmToTicksPerSecond(Globals.highGoalRPM, 1);
        } else if (Globals.autonomous) {
            this.targetVelocity = Globals.rpmToTicksPerSecond(Globals.powerShotAutoRPM, 1);
        } else {
            this.targetVelocity = Globals.rpmToTicksPerSecond(Globals.powerShotRPM, 1);
        }

        //packet for dashboard graph
        TelemetryPacket packet = new TelemetryPacket();

        // values to make graph look better
        packet.put("lower bound", 0.0);
        packet.put("upper bound", 6000.0);

        switch (this.mode)
        {
            case IDLE: // no power
                shooter1.setPower(0);
                shooter2.setPower(0);
                Globals.shots = 0;
                break;
            case OFF:
                shooter1.setPower(0);
                shooter2.setPower(0);
                Globals.shots = 0;
            case SHOOTING: // shooting speed including pidf
                // flap
                if(Globals.currentTargetType == Targets.TargetType.HIGHGOAL) {
                    flap.setPosition(lutHighgoal.get(distance));
                } /*else  if(Globals.autonomous) {
                    flap.setPosition(lutPowershotsAuto.get(distance));
                }*/ else {
                    flap.setPosition(lutPowershots.get(distance)+0.005);
                }

                // Call necessary controller methods
                veloController.setTargetVelocity(targetVelocity);
                veloController.setTargetAcceleration((targetVelocity - lastTargetVelocity) / veloTimer.seconds());
                veloTimer.reset();

                lastTargetVelocity = targetVelocity;

                // Get the position from the motor with the encoder
                double motorPos = shooter1.getCurrentPosition();

                // Update the controller and set the power for each motor
                double power = veloController.update(motorPos, currentVelocity);
                shooter1.setPower(power);
                shooter2.setPower(power);
                break;
        }
        packet.put("currentVelo", Globals.ticksPerSecondToRpm(currentVelocity, 1));
        packet.put("targetVelo", Globals.ticksPerSecondToRpm(targetVelocity, 1));

        // control feeder arm
        if(feederState == FeederState.RETRACTING && feederTimer.seconds() > actuationTime) {
            feederState = FeederState.RETRACTED;
        }
        if(feederState == FeederState.PUSHING && feederTimer.seconds() > actuationTime) {
            feederState = FeederState.EXTENDED;
        }

        // send shooter speeds to dashboard
        dashboard.sendTelemetryPacket(packet);
    }

}
