package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private boolean enabled;
    private DcMotorEx motor;

    public enum StateIntake{
        ON,
        IDLE,
        REVERSE
    }

    public StateIntake intakeState;
    public Intake(HardwareMap hardwareMap) {
        enabled = false;
        motor = hardwareMap.get(DcMotorEx.class, "Intake");
        intakeState = StateIntake.IDLE;
    }

    public void toggle() {
        enabled = !enabled;
    }

    public void update() {
        /*
        if(enabled) {
            motor.setPower(0.5);
        } else {
            motor.setPower(0);
        }
         */
        switch (intakeState){
            case ON:
                motor.setPower(0.45);
                break;
            case IDLE:
                motor.setPower(0);
                break;
            case REVERSE:
                motor.setPower((-0.45));
                break;
        }

    }

}
