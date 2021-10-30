package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.Vision;
import org.firstinspires.ftc.teamcode.lib.hardware.Robot;

//@Disabled
@Autonomous(group = "opmodes")
public class Auto_Blue extends LinearOpMode {

    // This enum defines our "state"
    // This is defines the possible steps our program will take
    enum State {
        IDLE,
        START_TO_SHUB,
        DEPOSIT_BLOCK,
        SHUB_TO_BARRIER,
        OVER_BARRIER
    }

    enum BarcodePos{
        BOTTOM,
        MIDDLE,
        TOP
    }

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
    BarcodePos barcodePos;

    //Trajectories
    Trajectory startToShub,shubToBarrier;

    private void initialize(){
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

        //define first state
        currentState = State.START_TO_SHUB;

        /*
         **  Trajectories
         */

        startToShub = robot.drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

        shubToBarrier = robot.drive.trajectoryBuilder(startToShub.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();
        //...
    }

    private BarcodePos scanBarcode() {
        //...
        return BarcodePos.TOP;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //define Variables etc.
        initialize();

        waitForStart();

        if (isStopRequested()) return;

        // clear cache for bulk reading
        for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }

        //scan Barcodes
        barcodePos = scanBarcode();

        // fix position error - updateDistance(driveToShoot1.end().minus(new Pose2d(5, 0, 0)));

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState) {
                case START_TO_SHUB:
                    currentState = State.DEPOSIT_BLOCK;
                    robot.drive.followTrajectoryAsync(startToShub);
                    break;
                case DEPOSIT_BLOCK:
                    if(!robot.drive.isBusy()) {
                        currentState = State.SHUB_TO_BARRIER;
                    }
                    break;
                case SHUB_TO_BARRIER:
                    currentState = State.OVER_BARRIER;
                    robot.drive.followTrajectoryAsync(shubToBarrier);
                    break;
                case OVER_BARRIER:
                    if(!robot.drive.isBusy()) {
                        //drive over barrier manually
                        currentState = State.IDLE;
                    }
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
        // write pose to PoseStorage one last time
        Globals.currentPose = robot.drive.getPoseEstimate();
    }
}