package org.firstinspires.ftc.teamcode.lib;

import android.provider.Settings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.lib.hardware.Intake;
import org.firstinspires.ftc.teamcode.lib.hardware.Robot;
import org.firstinspires.ftc.teamcode.lib.hardware.Shooter;
import org.firstinspires.ftc.teamcode.lib.hardware.Transfer;
import org.firstinspires.ftc.teamcode.lib.hardware.Wobble;

import java.util.Arrays;

public class RobotTele extends Robot {
    Controller controller1, controller2;
    int i = 0;

    public RobotTele(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        super(hardwareMap);

        // set wobble goal to right state
        wobblegripperOpen();
        //wobbleStoringPos();

        // initialise controllers
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        // set target to high goal
        Globals.setTarget(Targets.TargetType.HIGHGOAL);

        // get starting pose from auto
        switch(Globals.rings) {
            case 0:
                PoseStorage.currentPose = new Pose2d(10, 38, Math.toRadians(270));
                break;
            case 1:
                PoseStorage.currentPose = new Pose2d(10, 10, Math.toRadians(270));
                break;
            case 4:
                PoseStorage.currentPose = new Pose2d(10, 38, Math.toRadians(270));
                break;
            default:
                PoseStorage.currentPose = new Pose2d(10, 38, Math.toRadians(270));
                break;
        }



        drive.setPoseEstimate(PoseStorage.currentPose);
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

        // update joystick values in autoAim
        autoAim.updateJoysticks(controller1.getLeftJoystickXValue(), controller1.getLeftJoystickYValue(), controller1.getRightJoystickXValue());

        // update autoAim
        autoAim.update();

        // update shooter pidf and flap in the background
        Shooter.setDistance(AutoAim.getDistance());
        shooter.update();

        // set drive motor power
        drive.setWeightedDrivePower(autoAim.getDriveDirection());

        // Read pose
        poseEstimate = drive.getPoseEstimate();

        // Continually write pose to PoseStorage
        PoseStorage.currentPose = poseEstimate;
    }

    private void updateControls() {
        // controller 1

        // cancel driveToPoint
        /*
        if((Math.abs(controller1.getLeftJoystickXValue()) > 0.5 || Math.abs(controller1.getLeftJoystickYValue()) > 0.5 || Math.abs(controller1.getRightJoystickXValue()) > 0.5) && robotState == RobotState.AUTO_POSITION) {
            drive.cancelFollowing();
            setRobotState(RobotState.DRIVING);
        }

         */

        if (controller1.getaButton() == Controller.ButtonState.ON_PRESS) { // set to aiming mode
            setRobotState(RobotState.AIMING);
            Globals.updateTarget();
            if (Globals.currentTargetType == Targets.TargetType.HIGHGOAL) {
                intake.setRingArmClearingPos();
            }
        }

        if (controller1.getbButton() == Controller.ButtonState.ON_PRESS) { // set to intaking mode
            setRobotState(RobotState.INTAKING);
            if (Globals.currentTargetType == Targets.TargetType.HIGHGOAL) {
                intake.setRingArmExtendedPos();
            } else {
                intake.setRingArmLiftedPos();
            }
        }

        if(controller1.getxButton() == Controller.ButtonState.ON_PRESS) // reset y and heading components of pose
            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 62.8, Math.toRadians(358)));

        if(controller1.getyButton() == Controller.ButtonState.ON_PRESS) // reset pose
            drive.setPoseEstimate(new Pose2d(-62, 62.8, 0));

        if(controller1.getLeftTrigger() == Controller.ButtonState.ON_PRESS) // reverse intake
            reverseIntake();

        if(controller1.getLeftTrigger() == Controller.ButtonState.ON_RELEASE) // reset intake
            transferIdle();

        if(controller1.getRightTrigger() == Controller.ButtonState.PRESSED) { // shoot
            if(getRobotState() != RobotState.SHOOTING) {
                setRobotState(RobotState.SHOOTING);
            }
            shoot();
            if(Globals.currentTargetType == Targets.TargetType.HIGHGOAL) {
                intake.setRingArmExtendedPos();
            }
        }

        if(controller1.getRightBumper() == Controller.ButtonState.ON_PRESS) { // close / lift wobble arm
            if(wobble.getArmState() == Wobble.ArmState.INTAKE) {
                if(wobble.getGripperState() == Wobble.GripperState.OPEN) {
                    wobbleGrab();
                } else {
                    wobbleDroppingPos();
                }
            } else {
                wobbleStoringPos();
                if(wobble.getGripperState() == Wobble.GripperState.CLOSED_TIGHT) {
                    wobbleLoosenGrip();
                }
            }
        }



        if(controller1.getLeftBumper() == Controller.ButtonState.ON_PRESS) { // open / drop wobble arm
            intake.setRingArmLiftedPos();
            if(wobble.getArmState() == Wobble.ArmState.RELEASE || wobble.getArmState() == Wobble.ArmState.DROPPING) {
                if(wobble.getGripperState() != Wobble.GripperState.OPEN) {
                    wobblegripperOpen();
                } else {
                    transferIdle();
                    wobbleIntakingPos();
                }
            } else {
                if(wobble.getArmState() == Wobble.ArmState.STORED && wobble.getGripperState() != Wobble.GripperState.OPEN) {
                    wobbleReleasePos();
                } else {
                    wobbleIntakingPos();
                }
            }
        }


        // controller 2
        if(controller2.getdPadUp() == Controller.ButtonState.ON_PRESS) { // increase distance by subtracting 2 inches to x pose
            drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(4, 0, 0)));
        }

        if(controller2.getdPadDown() == Controller.ButtonState.ON_PRESS) { // increase distance by adding 2 inches to x pose
            drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(-4, 0, 0)));
        }

        if(controller2.getdPadLeft() == Controller.ButtonState.ON_PRESS) { // correct heading by adding 1 degree
            drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(0, 0, Math.toRadians(-2))));
        }

        if(controller2.getdPadRight() == Controller.ButtonState.ON_PRESS) { // correct heading by subtracting 1 degree
            drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(0, 0, Math.toRadians(2))));
        }

        if(controller2.getRightBumper() == Controller.ButtonState.ON_PRESS) { // change to the next right target
            switch (Globals.currentTargetType) {
                case HIGHGOAL:
                if(Globals.alliance == Globals.Alliance.BLUE)
                    Globals.setTarget(Targets.TargetType.OUTER_POWERSHOT);
                break;
                case OUTER_POWERSHOT:
                    if(Globals.alliance == Globals.Alliance.BLUE)
                        Globals.setTarget(Targets.TargetType.CENTER_POWERSHOT);
                    else
                        Globals.setTarget(Targets.TargetType.HIGHGOAL);
                    break;
                case CENTER_POWERSHOT:
                    if(Globals.alliance == Globals.Alliance.BLUE)
                        Globals.setTarget(Targets.TargetType.INNER_POWERSHOT);
                    else
                        Globals.setTarget(Targets.TargetType.OUTER_POWERSHOT);
                    break;
                case INNER_POWERSHOT:
                    if(Globals.alliance == Globals.Alliance.RED)
                        Globals.setTarget(Targets.TargetType.CENTER_POWERSHOT);
                    break;
            }
            Globals.updateTarget();
        }

        if(controller2.getLeftBumper() == Controller.ButtonState.ON_PRESS) { // change to the next left target
            switch (Globals.currentTargetType) {
                case HIGHGOAL:
                    if(Globals.alliance == Globals.Alliance.RED)
                        Globals.setTarget(Targets.TargetType.OUTER_POWERSHOT);
                    break;
                case OUTER_POWERSHOT:
                    if(Globals.alliance == Globals.Alliance.RED)
                        Globals.setTarget(Targets.TargetType.CENTER_POWERSHOT);
                    else
                        Globals.setTarget(Targets.TargetType.HIGHGOAL);
                    break;
                case CENTER_POWERSHOT:
                    if(Globals.alliance == Globals.Alliance.RED)
                        Globals.setTarget(Targets.TargetType.INNER_POWERSHOT);
                    else
                        Globals.setTarget(Targets.TargetType.OUTER_POWERSHOT);
                    break;
                case INNER_POWERSHOT:
                    if(Globals.alliance == Globals.Alliance.BLUE)
                        Globals.setTarget(Targets.TargetType.CENTER_POWERSHOT);
                    break;
            }
            Globals.updateTarget();
        }

        if(controller2.getLeftTrigger() == Controller.ButtonState.ON_PRESS) // reverse intake
            reverseIntake();

        if(controller2.getLeftTrigger() == Controller.ButtonState.ON_RELEASE) // reset intake
            transferIdle();

        if(controller2.getRightTrigger() == Controller.ButtonState.ON_PRESS) // drop ring arm
        {
            if (intake.armPos == Intake.ArmPos.EXTENDED) {
                intake.setRingArmLiftedPos();
            } else {
                intake.setRingArmExtendedPos();
            }
        }

        if(controller2.getaButton() == Controller.ButtonState.ON_PRESS) // activate intake and transfer
            intake();

        if(controller2.getbButton() == Controller.ButtonState.ON_PRESS) // shut down intake and transfer
            transferIdle();

        if(controller2.getxButton() == Controller.ButtonState.ON_PRESS) // reverse transfer
            reverseTransfer();

        if(controller2.getxButton() == Controller.ButtonState.ON_RELEASE) // reverse transfer
            intake();

        if(controller2.getyButton() == Controller.ButtonState.ON_PRESS) { // change aiming mode
            if(Globals.currentAimingMode == AutoAim.Mode.ALIGN_TO_HEADING)
                Globals.currentAimingMode = AutoAim.Mode.ALIGN_TO_POINT;
            else
                Globals.currentAimingMode = AutoAim.Mode.ALIGN_TO_HEADING;
            if(robotState == RobotState.AIMING || robotState == RobotState.SHOOTING)
                autoAim.setCurrentMode(Globals.currentAimingMode);
        }

        if(controller2.getRightJoystickYValue() > 0.5) {
            intake.setRingArmLiftedPos();
        }
        if(controller2.getRightJoystickYValue() < -0.5) {
            intake.setRingArmExtendedPos();
        }
        if(controller2.getRightJoystickXValue() > 0.5) {
            intake.setRingArmClearingPos();
        }
    }
}
