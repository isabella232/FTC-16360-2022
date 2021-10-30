package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.hardware.Robot;

public class RobotTele extends Robot {
    Controller controller1, controller2;
    int i = 0;

    public RobotTele(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        super(hardwareMap);

        // initialise controllers
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

    }

    @Override
    public void update() {
        // We update drive continuously in the background, regardless of state
        drive.update();

        // update controllers
        controller1.update();
        controller2.update();

        // update controls according to button states
        updateControls();
    }

    private void updateControls() {
/*
        if (controller1.getaButton() == Controller.ButtonState.ON_PRESS) { // set to aiming mode
            setRobotState(RobotState.AIMING);
            Globals.updateTarget();
            if (Globals.currentTargetType == Targets.TargetType.HIGHGOAL) {
                intake.setRingArmClearingPos();
            }
        }
 */
    }
}
