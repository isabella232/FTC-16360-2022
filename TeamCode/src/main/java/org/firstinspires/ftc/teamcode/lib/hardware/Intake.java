package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotorEx intakeMotor;

    private Servo intakeHolder1, intakeHolder2, ringArm;

    private final double intakeHolder1StartPos = 0;
    private final double intakeHolder1EndPos = 0.1;
    private final double intakeHolder2StartPos = 0.34;
    private final double intakeHolder2EndPos = 0.25;
    private final double ringArmExtendedPos = 1;
    private final double ringArmClearingPos = 0.65;
    private final double ringArmLiftedPos = 0.5;
    private final double ringArmStoredPos = 0;

    public enum Mode {
        IDLE,
        FORWARD,
        REVERSE
    }
    public enum ArmPos {
        CLEARING,
        EXTENDED,
        LIFTED
    }

    private Mode mode = Mode.IDLE;
    public ArmPos armPos = ArmPos.CLEARING;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        setMode(Mode.IDLE);

        intakeHolder1 = hardwareMap.get(Servo.class, "intakeHolderLeft");
        intakeHolder2 = hardwareMap.get(Servo.class, "intakeHolderRight");

        ringArm = hardwareMap.get(Servo.class, "ringArm");
        ringArm.setPosition(ringArmStoredPos);
    }

    public void holdIntake() {
        intakeHolder1.setPosition(intakeHolder1StartPos);
        intakeHolder2.setPosition(intakeHolder2StartPos);
    }

    public void releaseIntake() {
        intakeHolder1.setPosition(intakeHolder1EndPos);
        intakeHolder2.setPosition(intakeHolder2EndPos);
    }

    // ring arm methods
    public void setRingArmExtendedPos() {
        ringArm.setPosition(ringArmExtendedPos);
        armPos = ArmPos.EXTENDED;
    }

    public void setRingArmClearingPos() {
        ringArm.setPosition(ringArmClearingPos);
        armPos = ArmPos.CLEARING;
    }

    public void setRingArmLiftedPos() {
        ringArm.setPosition(ringArmLiftedPos);
        armPos = ArmPos.LIFTED;
    }

    public Mode getMode() {
        return mode;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
        changeMode();
    }

    private void changeMode() {
        switch (mode) {
            case IDLE:
                intakeMotor.setPower(0);
                break;
            case FORWARD:
                intakeMotor.setPower(1);
                break;
            case REVERSE:
                intakeMotor.setPower(-1);
                break;

        }
    }

}
