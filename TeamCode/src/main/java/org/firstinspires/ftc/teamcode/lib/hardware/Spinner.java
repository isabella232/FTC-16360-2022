package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spinner {
    HardwareMap hardwareMap;
    enum State {
        SPINNING,
        IDLE
    }
    public State state;

    private DcMotorEx motor;

    public Spinner(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        //initialize Spinner Motor
        motor = hardwareMap.get(DcMotorEx.class, "spinner");
        //motor.setDirection(DcMotor.Direction.REVERSE);

        //set inital state
        state = State.IDLE;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setSpinning() {
        state = State.SPINNING;
    }

    public void setIdle() {
        state = State.IDLE;
    }

    public void update() {
        switch (state) {
            case IDLE:
                motor.setTargetPosition(motor.getCurrentPosition());
                break;
            case SPINNING:
                motor.setPower(0.15);
                break;
        }
    }

}
