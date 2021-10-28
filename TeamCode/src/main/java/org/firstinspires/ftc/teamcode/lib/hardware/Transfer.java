package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {

    private DcMotorEx transferMotor;

    public enum Mode {
        IDLE,
        FORWARD,
        REVERSE
    }

    private Mode mode = Mode.IDLE;

    public Transfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");

        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //transferMotor.setDirection(DcMotorEx.Direction.REVERSE);

        setMode(Mode.IDLE);
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
                transferMotor.setPower(0);
                break;
            case FORWARD:
                transferMotor.setPower(0.85);
                break;
            case REVERSE:
                transferMotor.setPower(-0.6);
                break;

        }
    }
}
