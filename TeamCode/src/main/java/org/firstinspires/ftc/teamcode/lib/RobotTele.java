package org.firstinspires.ftc.teamcode.lib;

import android.widget.Spinner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.hardware.Arm;
import org.firstinspires.ftc.teamcode.lib.hardware.Intake;
import org.firstinspires.ftc.teamcode.lib.hardware.Robot;

public class RobotTele extends Robot {
    Controller controller1, controller2;
    int i = 0;
    Gamepad gamepad1, getGamepad2;
    Telemetry telemetry;

    public RobotTele(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        super(hardwareMap);

        // initialise controllers
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void updateRT() {
        // We update drive continuously in the background, regardless of state
        drive.update();

        update();

        // update controllers
        controller1.update();
        controller2.update();

        // update controls according to button states
        updateControls();

        // set motor arm offset
        arm.motorOffset = (int) -controller2.getLeftJoystickYValue()*60;

        //get controller input to drive
        drive.setWeightedDrivePower(new Pose2d(
                -gamepad1.left_stick_y * 0.4,
                -gamepad1.left_stick_x * 0.4, -gamepad1.right_stick_x * 0.4));
    }

    private void updateControls() {
        if (gamepad1.a) {//controller1.getaButton() == Controller.ButtonState.ON_PRESS) {
            spinner.setSpinning();
        }
        if (controller1.getaButton() == Controller.ButtonState.ON_RELEASE) {
            spinner.setIdle();
        }

        if (gamepad1.b) {//controller1.getaButton() == Controller.ButtonState.ON_PRESS) {
            spinner.setReversed();
        }
        if (controller1.getbButton() == Controller.ButtonState.ON_RELEASE) {
            spinner.setIdle();
        }

        if (controller2.getaButton() == Controller.ButtonState.ON_PRESS) {
            arm.armState = Arm.StateArm.BOTTOM;
            arm.handState = Arm.StateHand.CLOSED;
        }
        if (controller2.getbButton() == Controller.ButtonState.ON_PRESS) {
            arm.armState = Arm.StateArm.MIDDLE;
            arm.handState = Arm.StateHand.CLOSED;
        }
        if (controller2.getyButton() == Controller.ButtonState.ON_PRESS) {
            arm.armState = Arm.StateArm.TOP;
            arm.handState = Arm.StateHand.CLOSED;
        }
        if (controller2.getxButton() == Controller.ButtonState.ON_PRESS) {
            arm.armState = Arm.StateArm.FRONT;
            arm.handState = Arm.StateHand.OPEN;
        }
        if (controller2.getdPadDown() == Controller.ButtonState.ON_PRESS) {
            arm.armState = Arm.StateArm.IDLE;
            arm.handState = Arm.StateHand.OPEN;
        }
        if (controller2.getdPadUp() == Controller.ButtonState.ON_PRESS) {
            arm.armState = Arm.StateArm.RESETING;
            arm.handState = Arm.StateHand.OPEN;
        }
        if (controller2.getdPadUp() == Controller.ButtonState.ON_RELEASE){
            arm.armState = Arm.StateArm.RESET;
        }
        if (controller2.getLeftBumper() == Controller.ButtonState.ON_PRESS) {
            arm.handState = Arm.StateHand.CLOSED;
        }
        if (controller2.getRightBumper() == Controller.ButtonState.ON_PRESS){
            arm.handState = Arm.StateHand.OPEN;
        }
        if (controller2.getLeftTrigger() == Controller.ButtonState.ON_PRESS) {
            intake.intakeState = Intake.StateIntake.REVERSE;
        }
        if (controller2.getLeftTrigger() == Controller.ButtonState.ON_RELEASE){
            intake.intakeState = Intake.StateIntake.IDLE;
        }
        if (controller2.getRightTrigger() == Controller.ButtonState.ON_PRESS){
            intake.intakeState = Intake.StateIntake.ON;
        }
    }
}
