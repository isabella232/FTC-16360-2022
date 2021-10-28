package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

public class AutoAim {

    public static double DRAWING_TARGET_RADIUS = 2;

    // Define 3 states, driver control or alignment (point of heading) control
    public enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_HEADING,
        ALIGN_TO_POINT
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(new PIDCoefficients(12, 0, 0));

    // Declare a drive direction
    // Pose representing desired x, y, and angular velocity
    private Pose2d driveDirection = new Pose2d();

    private float leftJoystickX = 0;
    private float leftJoystickY = 0;
    private float rightJoystickX = 0;

    public AutoAim() {
        // Set input bounds for the heading controller
        // Automatic
        // ally handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);
    }

    private static double distance = 0;
    private static double headingError = 0;

    public void update() {
        Pose2d poseEstimate = PoseStorage.currentPose;
        Pose2d shooterPoseEstimate = poseEstimate.plus(new Pose2d(5*Math.sin(poseEstimate.getHeading()), -5*Math.cos(poseEstimate.getHeading()), 0));

        // Declare a target vector you'd like your bot to align with
        // Can be any x/y coordinate of your choosing
        Vector2d targetPosition = Globals.currentTarget;

        // Declare telemetry packet for dashboard field drawing
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        if (currentMode == Mode.NORMAL_CONTROL) {
            // Standard teleop control

            // Convert gamepad input into desired pose velocity
            driveDirection = new Pose2d(
                    -leftJoystickY,
                    -leftJoystickX,
                    -rightJoystickX
            );

        } else {  // auto aim

            // Create a vector from the gamepad x/y inputs which is the field relative movement
            // Then, rotate that vector by the inverse of that heading for field centric control
            Vector2d fieldFrameInput = new Vector2d(
                    -leftJoystickY,
                    -leftJoystickX
            );
            // field centric stuff
            Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

            Vector2d difference;
            if(Globals.currentAimingMode == Mode.ALIGN_TO_POINT) {
                // Difference between the target vector and the bot's position
                difference = targetPosition.minus(shooterPoseEstimate.vec());
            } else {
                // Create artificial target at 4 heading
                difference = new Vector2d(Targets.targetX-shooterPoseEstimate.getX(), 0);
            }
            // calculate distance for flap
            distance = difference.norm();
            // correct for curved shooting
            difference = difference.minus(new Vector2d(0, (Targets.targetX-shooterPoseEstimate.getX())*Math.tan(Math.toRadians(Globals.aimingHeadingError))));
            // Obtain the target angle for feedback and derivative for feedforward
            double theta = difference.angle();
            // Not technically omega because its power. This is the derivative of atan2
            double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

            //calculate remaining heading to change
            headingError = Math.abs(Math.toDegrees(theta));

            // Set the target heading for the heading controller to our desired angle
            headingController.setTargetPosition(theta);

            // Set desired angular velocity to the heading controller output + angular
            // velocity feedforward
            double headingInput = (headingController.update(poseEstimate.getHeading())
                    * DriveConstants.kV + thetaFF)
                    * DriveConstants.TRACK_WIDTH;

            // Combine the field centric x/y velocity with our derived angular velocity
            driveDirection = new Pose2d(
                    robotFrameInput,
                    headingInput
            );
            // Draw the target on the field
            fieldOverlay.setStroke("#dd2c00");
            fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

            // Draw lines to target
            fieldOverlay.setStroke("#b89eff");
            fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
            fieldOverlay.setStroke("#ffce7a");
            fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
            fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
        }
        // Update the heading controller with our current heading
        headingController.update(poseEstimate.getHeading());

        // Draw bot on canvas
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

        // Send telemetry packet off to dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    public void updateJoysticks(float leftJoystickX, float leftJoystickY, float rightJoystickX) {
        this.leftJoystickX = leftJoystickX;
        this.leftJoystickY = leftJoystickY;
        this.rightJoystickX = rightJoystickX;
    }

    public void setCurrentMode(Mode mode) {
        currentMode = mode;
    }

    public Mode getCurrentMode() {
        return currentMode;
    }

    public Pose2d getDriveDirection() {
        return driveDirection;
    }

    public static double getDistance() {
        return distance;
    }

    public static double getHeadingError() {
        return headingError;
    }
}