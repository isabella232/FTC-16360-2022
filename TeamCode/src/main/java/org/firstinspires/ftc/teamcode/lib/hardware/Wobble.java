package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Wobble {
    private Servo wobbleArm1, wobbleArm2, wobbleGripper;

    public enum ArmState {
        START_POS,
        INTAKE,
        OUTTAKE,
        STORED,
        RELEASE,
        DROPPING
    }

    public enum GripperState {
        OPEN,
        CLOSED_TIGHT,
        CLOSED_LOOSE
    }

    private ArmState armState = ArmState.START_POS;

    private GripperState gripperState = GripperState.CLOSED_TIGHT;

    public Wobble(HardwareMap hardwareMap) {
        wobbleArm1 = hardwareMap.get(Servo.class, "wobbleArmLeft");
        wobbleArm2 = hardwareMap.get(Servo.class, "wobbleArmRight");
        wobbleGripper = hardwareMap.get(Servo.class, "wobbleGripper");
    }

    // in between bounds, automatically sets second servo according to first one
    private void setWobbleArmPosition (double targetPosition) {
        wobbleArm1.setPosition(targetPosition);
        wobbleArm2.setPosition(1-wobbleArm1.getPosition()); //since they are mirrored, boundaries should match
    }

    // in between bounds
    private void setWobbleGripperPosition (double targetPosition) {
        wobbleGripper.setPosition(targetPosition);
    }

    public ArmState getArmState() {
        return armState;
    }

    public void setArmState(ArmState armState) {
        this.armState = armState;
        switch (armState) {
            case INTAKE:
                setWobbleArmPosition(1);
                break;
            case STORED:
                setWobbleArmPosition(0.05);
                break;
            case DROPPING:
                setWobbleArmPosition(0.45);
                break;
            case RELEASE:
                setWobbleArmPosition(0.45);
                break;
            case OUTTAKE:
                setWobbleArmPosition(0.75);
                break;
            case START_POS:
                setWobbleArmPosition(0);
                break;
        }
    }

    public GripperState getGripperState() {
        return gripperState;
    }

    public void setGripperState(GripperState gripperState) {
        this.gripperState = gripperState;
        switch (gripperState) {
            case OPEN:
                setWobbleGripperPosition(1);
                break;
            case CLOSED_LOOSE:
                setWobbleGripperPosition(0.45);
                break;
            case CLOSED_TIGHT:
                setWobbleGripperPosition(0.35);
                break;
        }
    }
}
