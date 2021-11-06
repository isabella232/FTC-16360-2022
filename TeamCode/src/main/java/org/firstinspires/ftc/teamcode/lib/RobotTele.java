package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.hardware.Arm;
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
        if (controller1.getaButton() == Controller.ButtonState.ON_PRESS) {
            spinner.setSpinning();
        }
        if (controller1.getaButton() == Controller.ButtonState.ON_RELEASE) {
            spinner.setIdle();
        }

        if (controller2.getaButton() == Controller.ButtonState.ON_PRESS) {
            arm.armState = Arm.StateArm.BOTTOM;
        }
        if (controller2.getbButton() == Controller.ButtonState.ON_PRESS) {
            arm.armState = Arm.StateArm.MIDDLE;
        }
        if (controller2.getyButton() == Controller.ButtonState.ON_PRESS) {
            arm.armState = Arm.StateArm.TOP;
        }
        if (controller2.getxButton() == Controller.ButtonState.ON_PRESS) {
            arm.armState = Arm.StateArm.FRONT;
        }
        if (controller2.getLeftBumper() == Controller.ButtonState.ON_PRESS) {
            arm.toggleHand();
        }
    }
}
