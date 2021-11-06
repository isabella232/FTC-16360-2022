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

    // Define our start pose
    Pose2d startPose = new Pose2d(-62, 23, Math.toRadians(0));

    // robot class
    Robot robot;

    // Vision Class Variable
    Vision vision;

    // Timer for timed actions
    ElapsedTime waitTimer;

    //
    Globals.BarcodePos barcodePos;

    //Telemetry
    Telemetry telemetry;

    //Trajectories
    Trajectory startToShub,shubToBarrier, barrierToWarehouse, shubToDuck, duckToHome;

    public Auto_Base(HardwareMap hardwareMap, Telemetry telemetry){
        //copy telemetry
        this.telemetry = telemetry;

        // Initialize the robot class
        robot = new Robot(hardwareMap);

        // Initialize the vision Class
        vision = new Vision(hardwareMap);

        // Set initial pose
        robot.drive.setPoseEstimate(startPose);

        // Save initial pose to PoseStorage
        Globals.currentPose = startPose;

        // a timer for timed actions
        waitTimer = new ElapsedTime();

        // define first state
        currentState = State.START_TO_SHUB;

        // get Barcode Position
        barcodePos = vision.getBarcodePosition();

        /*
         **  Trajectories
         */
        switch (startPos) {
            case RED_INNER:
                startToShub = robot.drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                        .build();
                shubToBarrier = robot.drive.trajectoryBuilder(startToShub.end())
                        .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                        .build();
                barrierToWarehouse = robot.drive.trajectoryBuilder(shubToBarrier.end())
                        .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                        .build();
                break;
            case RED_OUTER:
                startToShub = robot.drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(0, 1, Math.toRadians(0)))
                        .build();
                shubToDuck = robot.drive.trajectoryBuilder(startToShub.end())
                        .lineToLinearHeading(new Pose2d(0, 3, Math.toRadians(0)))
                        .build();
                duckToHome = robot.drive.trajectoryBuilder(shubToDuck.end())
                        .lineToLinearHeading(new Pose2d(0, 3, Math.toRadians(0)))
                        .build();
                break;
            case BLUE_INNER:
                startToShub = robot.drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(0, 2, Math.toRadians(0)))
                        .build();
                shubToBarrier = robot.drive.trajectoryBuilder(startToShub.end())
                        .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                        .build();
                barrierToWarehouse = robot.drive.trajectoryBuilder(shubToBarrier.end())
                        .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                        .build();
                break;
            case BLUE_OUTER:
                startToShub = robot.drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(0, 3, Math.toRadians(0)))
                        .build();
                shubToDuck = robot.drive.trajectoryBuilder(startToShub.end())
                        .lineToLinearHeading(new Pose2d(0, 3, Math.toRadians(0)))
                        .build();
                duckToHome = robot.drive.trajectoryBuilder(shubToDuck.end())
                        .lineToLinearHeading(new Pose2d(0, 3, Math.toRadians(0)))
                        .build();
                break;
        }
    }

    public void update() {

        switch (currentState) {
            case START_TO_SHUB:
                currentState = State.DEPOSIT_BLOCK;
                robot.drive.followTrajectoryAsync(startToShub);
                break;

            case DEPOSIT_BLOCK:
                if(!robot.drive.isBusy()) {
                    //deposit block
                    if (startPos == StartPos.BLUE_INNER || startPos == StartPos.RED_INNER) {// if we started inner
                        currentState = State.SHUB_TO_BARRIER;
                    } else {
                        currentState = State.SHUB_TO_DUCK;
                    }
                }
                break;

            case SHUB_TO_BARRIER:
                currentState = State.BARRIER_TO_WAREHOUSE;
                robot.drive.followTrajectoryAsync(shubToBarrier);
                break;

            case BARRIER_TO_WAREHOUSE:
                if(!robot.drive.isBusy()) {
                    robot.drive.followTrajectoryAsync(barrierToWarehouse);
                    currentState = State.IDLE;
                }
                break;

            case SHUB_TO_DUCK:
                currentState = Auto_Base.State.SPIN_DUCK;
                robot.drive.followTrajectoryAsync(shubToDuck);
                break;
            case SPIN_DUCK:
                if(!robot.drive.isBusy()) {
                    robot.spinner.setSpinning();
                    currentState = Auto_Base.State.SPINNING;
                    waitTimer.reset();
                }
                break;
            case SPINNING:
                if(waitTimer.seconds() > 1) {
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