package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private boolean enabled;
    private DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        enabled = false;
        motor = hardwareMap.get(DcMotorEx.class, "Intake");
    }

    public void toggle() {
        enabled = !enabled;
    }

    public void update() {
        if(enabled) {
            motor.setPower(0.6);
        } else {
            motor.setPower(0);
        }
    }
}
