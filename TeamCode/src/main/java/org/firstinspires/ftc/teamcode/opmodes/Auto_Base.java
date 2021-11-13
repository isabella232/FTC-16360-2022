package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.Vision;
import org.firstinspires.ftc.teamcode.lib.hardware.Arm;
import org.firstinspires.ftc.teamcode.lib.hardware.Robot;

//@Disabled
public class Auto_Base{

    // This enum defines our "state"
    // This is defines the possible steps our program will take
    enum State {
        IDLE,
        START_TO_SHUB,
        DEPOSIT_BLOCK,
        SHUB_TO_BARRIER,
        SHUB_TO_D0,
        SHUB_TO_DUCK,
        SPIN_DUCK,
        SPINNING,
        DUCK_TO_HOME,
	    BARRIER_TO_WAREHOUSE
    }

    enum StartPos {
        RED_INNER,
        RED_OUTER,
        BLUE_INNER,
        BLUE_OUTER
    }

    // Out Starting Position
    public StartPos startPos = StartPos.BLUE_INNER;

    // Our current State
    State currentState;

    // Declare our start pose
    Pose2d startPose;

    // robot class
    Robot robot;

    // Vision Class Variable ToDo
    Vision vision;

    // Timer for timed actions
    ElapsedTime waitTimer;

    //
    Globals.BarcodePos barcodePos;

    //Telemetry
    Telemetry telemetry;

    //Trajectories
    Trajectory startToShub,shubToBarrier, barrierToWarehouse, shubToDuck, duckToHome;

    public Auto_Base(HardwareMap hardwareMap, Telemetry telemetry, StartPos startPos){
        //copy telemetry, startPos
        this.telemetry = telemetry;
        this.startPos = startPos;

        // Initialize the robot class
        robot = new Robot(hardwareMap);

        //set arm to idle and close hand
        robot.arm.armState = Arm.StateArm.IDLE;
        robot.arm.handState = Arm.StateHand.CLOSED;
        robot.update();
        robot.arm.forceReset();

        // Initialize the vision Class ToDo
        vision = new Vision(hardwareMap);

        // a timer for timed actions
        waitTimer = new ElapsedTime();

        // define first state
        currentState = State.START_TO_SHUB;

        // get Barcode Position ToDO
        barcodePos = vision.getBarcodePosition();
        barcodePos = Globals.BarcodePos.MIDDLE;

        /*
         **  Trajectories
         */
        switch (startPos) {
            case RED_INNER:
                startPose = new Pose2d(62.4, 76.85, Math.toRadians(180));
                startToShub = robot.drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(37.35, 59, Math.toRadians(0)))
                        .build();
                shubToBarrier = robot.drive.trajectoryBuilder(startToShub.end())
                        .lineToLinearHeading(new Pose2d(65, 75, Math.toRadians(90)))
                        .build();
                barrierToWarehouse = robot.drive.trajectoryBuilder(shubToBarrier.end())
                        .lineToLinearHeading(new Pose2d(65, 118, Math.toRadians(90)))
                        .build();
                break;
            case RED_OUTER:
                startPose = new Pose2d(62.4, 29.6, Math.toRadians(180));
                startToShub = robot.drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(37.35, 59, Math.toRadians(0)))//
                        .build();
                shubToDuck = robot.drive.trajectoryBuilder(startToShub.end())
                        .lineToLinearHeading(new Pose2d(57.3, 8.39, Math.toRadians(90)))
                        .build();
                duckToHome = robot.drive.trajectoryBuilder(shubToDuck.end())
                        .lineToLinearHeading(new Pose2d(35.4, 8.39, Math.toRadians(90)))
                        .build();
                break;
            case BLUE_INNER:
                startPose = new Pose2d(-62.4, 76.85, Math.toRadians(0));
                startToShub = robot.drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(-37.35, 59, Math.toRadians(180)))
                        .build();
                shubToBarrier = robot.drive.trajectoryBuilder(startToShub.end())
                        .lineToLinearHeading(new Pose2d(-65, 75, Math.toRadians(90)))
                        .build();
                barrierToWarehouse = robot.drive.trajectoryBuilder(shubToBarrier.end())
                        .lineToLinearHeading(new Pose2d(-65, 118, Math.toRadians(90)))
                        .build();
                break;
            case BLUE_OUTER:
                startPose = new Pose2d(-62.4, 29.6, Math.toRadians(0));
                startToShub = robot.drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(-37.35, 59, Math.toRadians(180)))
                        .build();
                shubToDuck = robot.drive.trajectoryBuilder(startToShub.end())
                        .lineToLinearHeading(new Pose2d(-57.3, 8.39, Math.toRadians(90)))
                        .build();
                duckToHome = robot.drive.trajectoryBuilder(shubToDuck.end())
                        .lineToLinearHeading(new Pose2d(-35.4, 8.39, Math.toRadians(90)))
                        .build();
                break;
        }
        // Save initial pose to PoseStorage
        Globals.currentPose = startPose;

        // Set initial pose
        robot.drive.setPoseEstimate(startPose);
    }

    public void update() {

        switch (currentState) {
            case START_TO_SHUB:
                currentState = State.DEPOSIT_BLOCK;
                robot.drive.followTrajectoryAsync(startToShub);
                    //quality code right here
                switch (barcodePos) {
                    case TOP:
                        robot.arm.armState = Arm.StateArm.TOP;
                        break;
                    case MIDDLE:
                        robot.arm.armState = Arm.StateArm.MIDDLE;
                        break;
                    case BOTTOM:
                        robot.arm.armState = Arm.StateArm.BOTTOM;
                }
                break;

            case DEPOSIT_BLOCK:
                if(!robot.drive.isBusy()) {
                    robot.arm.handState = Arm.StateHand.OPEN;
                    if (startPos == StartPos.BLUE_INNER || startPos == StartPos.RED_INNER) {// if we started inner
                        currentState = State.SHUB_TO_BARRIER;
                    } else {
                        currentState = State.SHUB_TO_D0;
                    }
                    waitTimer.reset();
                }
                break;

            case SHUB_TO_BARRIER:
                if(waitTimer.seconds() > 1) {
                    currentState = State.BARRIER_TO_WAREHOUSE;
                    robot.drive.followTrajectoryAsync(shubToBarrier);
                }
                break;

            case BARRIER_TO_WAREHOUSE:
                if(!robot.drive.isBusy()) {
                    robot.arm.armState = Arm.StateArm.FRONT;
                    robot.drive.followTrajectoryAsync(barrierToWarehouse);
                    currentState = State.IDLE;
                }
                break;

            case SHUB_TO_D0:
                if(waitTimer.seconds() > 1) {
                    currentState = State.SHUB_TO_DUCK;
                    robot.drive.followTrajectoryAsync(shubToDuck);
                    waitTimer.reset();
                }
                break;

            case SHUB_TO_DUCK:
                if(waitTimer.milliseconds() > 750) {
                    currentState = State.SPIN_DUCK;
                    robot.arm.armState = Arm.StateArm.FRONT;
                }

            case SPIN_DUCK:
                if(!robot.drive.isBusy()) {
                    robot.spinner.setSpinning();
                    currentState = Auto_Base.State.SPINNING;
                    waitTimer.reset();
                }
                break;
            case SPINNING:
                if(waitTimer.seconds() > 2) {
                    robot.spinner.setIdle();
                    currentState = Auto_Base.State.DUCK_TO_HOME;
                }
                break;
            case DUCK_TO_HOME:
                currentState = Auto_Base.State.IDLE;
                robot.drive.followTrajectoryAsync(duckToHome);
                break;
        }

        // We update robot continuously in the background, regardless of state
        robot.update();

        // Read pose
        Pose2d poseEstimate = robot.drive.getPoseEstimate();

        // Continually write pose to PoseStorage
        Globals.currentPose = poseEstimate;

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("position", barcodePos);
        telemetry.addData("state", currentState);
        telemetry.update();
    }
}