package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.Targets;
import org.firstinspires.ftc.teamcode.lib.Vision;
import org.firstinspires.ftc.teamcode.lib.hardware.Robot;
import org.firstinspires.ftc.teamcode.lib.hardware.Shooter;

//@Disabled
@Autonomous(group = "opmodes")
public class Auto_Blue_Classic extends LinearOpMode {

    private void updateDistance(Pose2d shootingPos) {
        Shooter.setDistance((new Vector2d(72, 35).minus(shootingPos.vec())).norm());
    }

    // robot class
    Robot robot;

    // This enum defines our "state"
    // This is defines the possible steps our program will take
    enum State {
        DRIVE_TO_SHOOT_0,
        DRIVE_TO_SHOOT_1,
        DRIVE_TO_SHOOT_WAIT,
        SHOOTING_1,
        SHOOTING_1_2,
        SHOOTING_1_2_WAIT,
        INTAKING_1,
        WAIT_1,
        SHOOTING_4_2,
        INTAKING_2,
        WAIT_2,
        SHOOTING_3,
        DELIVER_WOBBLE_1,
        DROP_WOBBLE_1,
        CLEAR_WOBBLE_1,
        GET_WOBBLE_2,
        GRAB_WOBBLE_2,
        DELIVER_WOBBLE_2,
        DROP_WOBBLE_2,
        CLEAR_WOBBLE_2,
        PARK,
        IDLE
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    int[] measurements = new int[30];

    // Define our start pose
    Pose2d startPose = new Pose2d(-62, 23, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        // Variable for rings
        int rings = 0;

        // Initialize the robot class
        robot = new Robot(hardwareMap);

        // Initialize the shooter
        Vision vision = new Vision(hardwareMap);

        // Set initial pose
        robot.drive.setPoseEstimate(startPose);

        // Save initial pose to PoseStorage
        PoseStorage.currentPose = startPose;

        // set intake servos to hold
        robot.holdIntake();

        // set gripper to close
        robot.wobbleGrab();

        // Time to shoot
        double shootTime = 0.41;

        // Time for all rings to pass into hopper
        double transferTime = 0.7;

        // Time for wobble goal to be dropped
        double wobbleDropTime = 0.2;

        // Time for wobble goal to be grabbed
        double wobbleGrabTime = 0.8;

        // a timer for timed actions
        ElapsedTime waitTimer = new ElapsedTime();

        // trajectory moves to the spot to shoot at the high goal
        Trajectory driveToShoot0 = robot.drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-12, 30, Math.toRadians(3)))
                .build();

        Trajectory driveToShoot1 = robot.drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-46, 30, Math.toRadians(1)))
                .build();

        Trajectory driveToShoot2 = robot.drive.trajectoryBuilder(driveToShoot1.end())
                .lineToLinearHeading(new Pose2d(-12, 30, Math.toRadians(358)))
                .build();


        // Trajectory to intake single ring
        Trajectory intaking1_1 = robot.drive.trajectoryBuilder(driveToShoot1.end())
                .lineToLinearHeading(new Pose2d(-10, 30, Math.toRadians(3)))
                .build();

        // Trajectory to intake first several rings of 4 stack
        Trajectory intaking1_4 = robot.drive.trajectoryBuilder(driveToShoot1.end())
                .lineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(0)))
                .build();

        // Trajectory to intake remaining rings of 4 stack
        Trajectory intaking2_4 = robot.drive.trajectoryBuilder(intaking1_4.end())
                .lineToLinearHeading(new Pose2d(-10, 36, Math.toRadians(358)))
                .build();


        // Trajectories to deposit first wobble goal
        Trajectory depositWobble1_0 = robot.drive.trajectoryBuilder(driveToShoot0.end())
                .lineToLinearHeading(new Pose2d(10, 50, Math.toRadians(270)))
                .build();

        Trajectory depositWobble1_1 = robot.drive.trajectoryBuilder(driveToShoot2.end())
                .lineToLinearHeading(new Pose2d(40, 23, Math.toRadians(270)))
                .build();

        Trajectory depositWobble1_4 = robot.drive.trajectoryBuilder(intaking2_4.end())
                .lineToLinearHeading(new Pose2d(62, 42, Math.toRadians(270)))
                .build();


        // Trajectories to clear first wobble goal after depositing
        Trajectory clearWobble1_0 = robot.drive.trajectoryBuilder(depositWobble1_0.end())
                .lineTo(new Vector2d(10, 40))
                .build();

        Trajectory clearWobble1_1 = robot.drive.trajectoryBuilder(depositWobble1_1.end())
                .lineTo(new Vector2d(37, 10))
                .build();

        Trajectory clearWobble1_4 = robot.drive.trajectoryBuilder(depositWobble1_4.end())
                .lineTo(new Vector2d(50, 30))
                .build();


        // Trajectories to pick up the second wobble goal
        Trajectory getWobble2_0 = robot.drive.trajectoryBuilder(clearWobble1_0.end())
                .lineToLinearHeading(new Pose2d(-34.5, 45, Math.toRadians(0)))
                .build();

        Trajectory getWobble2_1 = robot.drive.trajectoryBuilder(clearWobble1_1.end())
                .lineToLinearHeading(new Pose2d(-34.5, 45, Math.toRadians(0)))
                .build();

        Trajectory getWobble2_4 = robot.drive.trajectoryBuilder(clearWobble1_4.end())
                .lineToLinearHeading(new Pose2d(-34.5, 45, Math.toRadians(0)))
                .build();


        // Trajectories to deposit second wobble goal
        Trajectory deliverWobble2_0 = robot.drive.trajectoryBuilder(getWobble2_0.end())
                .lineToLinearHeading(new Pose2d(15, 41, Math.toRadians(270)))
                .build();

        Trajectory deliverWobble2_1 = robot.drive.trajectoryBuilder(getWobble2_1.end())
                .lineToLinearHeading(new Pose2d(29, 23, Math.toRadians(270)))
                .build();

        Trajectory deliverWobble2_4 = robot.drive.trajectoryBuilder(getWobble2_4.end())
                .lineToLinearHeading(new Pose2d(55, 41, Math.toRadians(270)))
                .build();


        // Trajectories to clear the second wobble goal after dropping it
        Trajectory clearWobble2_1 = robot.drive.trajectoryBuilder(deliverWobble2_1.end())
                .lineTo(new Vector2d(23.5, 10))
                .build();


        // Trajectories to park
        Trajectory park_0 = robot.drive.trajectoryBuilder(deliverWobble2_0.end())
                .lineToLinearHeading(new Pose2d(10, 38, Math.toRadians(270)))
                .build();

        Trajectory park_1 = robot.drive.trajectoryBuilder(clearWobble2_1.end())
                .lineToLinearHeading(new Pose2d(10, 10, Math.toRadians(269)))
                .build();

        Trajectory park_4 = robot.drive.trajectoryBuilder(deliverWobble2_4.end())
                .lineToLinearHeading(new Pose2d(10, 38, Math.toRadians(268)))
                .build();




        while (!isStarted()) {

            for(int i = 1; i < 30; i++) {
                measurements[i] = measurements[i - 1];
            }
            measurements[0] = vision.getRingAmount();
            if (vision.getRingAmount() == 4) {
                measurements[0] = 2;
            }

            telemetry.addData("rings", vision.getRingAmount());
            telemetry.update();
        }

        if (isStopRequested()) return;

        // clear cache for bulk reading
        for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }

        //get ring amount
        for(int i = 0; i < 30; i++) {
            rings += measurements[i];
        }

        rings = Math.round(rings / 30);

        if(rings == 2) rings = 4;



        //drop the intake
        robot.dropIntake();
        waitTimer.reset();
        while(waitTimer.seconds() <= 0.3)
        {
        }
        //robot.wobbleStoringPos();

        switch (rings) {
            case 0:
                currentState = State.DRIVE_TO_SHOOT_0;
                break;
            case 1:
                currentState = State.DRIVE_TO_SHOOT_1;
                break;
            case 4:
                currentState = State.DRIVE_TO_SHOOT_1;
                break;
        }

        Globals.shots = 0; // reset ring counter

        Globals.currentTargetType = Targets.TargetType.HIGHGOAL;

        while (opModeIsActive() && !isStopRequested()) {
            // clear cache for bulk reading
            for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }

            // The state machine logic

            // We define the flow of the state machine through this switch statement
            switch (currentState) {

                case DRIVE_TO_SHOOT_0:
                    updateDistance(driveToShoot0.end().minus(new Pose2d(0, 0, 0)));
                    robot.drive.followTrajectoryAsync(driveToShoot0);
                    robot.setRobotState(Robot.RobotState.SHOOTING);
                    currentState = State.DRIVE_TO_SHOOT_WAIT;
                    waitTimer.reset();
                    break;

                case DRIVE_TO_SHOOT_1:
                    if(rings == 1) {
                        updateDistance(driveToShoot1.end().minus(new Pose2d(25, 0, 0)));
                    } else {
                        updateDistance(driveToShoot1.end().minus(new Pose2d(5, 0, 0)));
                    }
                    robot.drive.followTrajectoryAsync(driveToShoot1);
                    robot.setRobotState(Robot.RobotState.SHOOTING);
                    currentState = State.DRIVE_TO_SHOOT_WAIT;
                    waitTimer.reset();
                    break;

                case DRIVE_TO_SHOOT_WAIT:
                    if (!robot.drive.isBusy()) {
                        currentState = State.SHOOTING_1;
                        robot.setRobotState(Robot.RobotState.SHOOTING);
                        waitTimer.reset();
                    }
                    break;

                case SHOOTING_1:
                    robot.shoot();
                        switch(rings) {
                            case 0:
                                if (Globals.shots > 3) {
                                    robot.wobbleOuttakingPos();
                                    robot.drive.followTrajectoryAsync(depositWobble1_0);
                                    robot.setRobotState(Robot.RobotState.AUTO_DRIVING);
                                    currentState = State.DELIVER_WOBBLE_1;
                                    break;
                                }
                            case 1:
                                if (Globals.shots > 0 && rings == 1) {      //how tf chunt er bi 0 ring da ine
                                    robot.wobbleOuttakingPos();
                                    robot.drive.followTrajectoryAsync(driveToShoot2);
                                    updateDistance(driveToShoot2.end());
                                    Globals.shots = 0;
                                    robot.intake();
                                    currentState = State.SHOOTING_1_2_WAIT;
                                    break;
                                }
                            case 4:
                                if (Globals.shots > 3) {
                                    robot.wobbleOuttakingPos();
                                    robot.drive.followTrajectoryAsync(intaking1_4);
                                    updateDistance(intaking1_4.end());
                                    Globals.shots = 0;
                                    robot.intake();
                                    currentState = State.INTAKING_1;
                                    break;
                                }
                            telemetry.addData("Time", waitTimer.seconds());
                        }
                    break;

                case SHOOTING_1_2_WAIT:
                    if (!robot.drive.isBusy()) {
                        waitTimer.reset();
                        robot.transferIdle();
                        robot.intakeIdle();
                        currentState = State.SHOOTING_1_2;
                        waitTimer.reset();
                    }
                    break;

                case SHOOTING_1_2:
                    robot.shoot();
                    if(Globals.shots > 3) {
                        robot.drive.followTrajectoryAsync(depositWobble1_1);
                        robot.setRobotState(Robot.RobotState.AUTO_DRIVING);
                        currentState = State.DELIVER_WOBBLE_1;
                    }
                    break;

                case INTAKING_1:
                    if (!robot.drive.isBusy()) {
                        currentState = State.WAIT_1;
                        waitTimer.reset();
                    }
                    break;

                case WAIT_1:
                    if (waitTimer.seconds() >= transferTime) {
                        robot.transferIdle();
                        currentState = State.SHOOTING_4_2;
                        robot.setRobotState(Robot.RobotState.SHOOTING);
                        waitTimer.reset();
                    }
                    break;

                case SHOOTING_4_2:
                    robot.shoot();
                    if (Globals.shots > 2) {
                        robot.drive.followTrajectoryAsync(intaking2_4);
                        updateDistance(intaking2_4.end());
                        Globals.shots = 0;
                        robot.intake();
                        currentState = State.INTAKING_2;
                    }
                    break;

                case INTAKING_2:
                    if (!robot.drive.isBusy()) {
                        robot.transferIdle();
                        currentState = State.WAIT_2;
                        waitTimer.reset();
                    }
                    break;

                case WAIT_2:
                    if (waitTimer.seconds() >= transferTime) {
                        robot.transferIdle();
                        currentState = State.SHOOTING_3;
                        robot.setRobotState(Robot.RobotState.SHOOTING);
                        waitTimer.reset();
                    }
                    break;

                case SHOOTING_3:
                    robot.shoot();
                    if (Globals.shots > 3) {
                        robot.drive.followTrajectoryAsync(depositWobble1_4);
                        robot.setRobotState(Robot.RobotState.AUTO_DRIVING);
                        currentState = State.DELIVER_WOBBLE_1;
                    }
                    break;

                case DELIVER_WOBBLE_1:
                    if (!robot.drive.isBusy()) {
                        robot.wobblegripperOpen();
                        robot.wobbleStoringPos();
                        currentState = State.DROP_WOBBLE_1;
                        waitTimer.reset();
                    }
                    break;

                case DROP_WOBBLE_1:
                    if (waitTimer.seconds() >= wobbleDropTime) {
                        switch(rings) {
                            case 0:
                                robot.drive.followTrajectoryAsync(clearWobble1_0);
                                break;
                            case 1:
                                robot.drive.followTrajectoryAsync(clearWobble1_1);
                                break;
                            case 4:
                                robot.drive.followTrajectoryAsync(clearWobble1_4);
                                break;
                        }
                        currentState = State.CLEAR_WOBBLE_1;
                    }
                    break;

                case CLEAR_WOBBLE_1:
                    if (!robot.drive.isBusy()) {
                        robot.wobbleIntakingPos();
                        switch(rings) {
                            case 0:
                                robot.drive.followTrajectoryAsync(getWobble2_0);
                                break;
                            case 1:
                                robot.drive.followTrajectoryAsync(getWobble2_1);
                                break;
                            case 4:
                                robot.drive.followTrajectoryAsync(getWobble2_4);
                                break;
                        }
                        currentState = State.GET_WOBBLE_2;
                    }
                    break;

                case GET_WOBBLE_2:
                    if (!robot.drive.isBusy()) {
                        robot.wobbleGrab();
                        currentState = State.GRAB_WOBBLE_2;
                        waitTimer.reset();
                    }
                    break;

                case GRAB_WOBBLE_2:
                    if (waitTimer.seconds() >= wobbleGrabTime) {
                        robot.wobbleOuttakingPos();
                        switch(rings) {
                            case 0:
                                robot.drive.followTrajectoryAsync(deliverWobble2_0);
                                break;
                            case 1:
                                robot.drive.followTrajectoryAsync(deliverWobble2_1);
                                break;
                            case 4:
                                robot.drive.followTrajectoryAsync(deliverWobble2_4);
                                break;
                        }
                        currentState = State.DELIVER_WOBBLE_2;
                    }
                    break;

                case DELIVER_WOBBLE_2:
                    if (!robot.drive.isBusy()) {
                        robot.wobblegripperOpen();
                        robot.wobbleStoringPos();
                        currentState = State.DROP_WOBBLE_2;
                        waitTimer.reset();
                    }
                    break;

                case DROP_WOBBLE_2:
                    if (waitTimer.seconds() >= wobbleDropTime) {
                        switch(rings) {
                            case 0:
                                robot.drive.followTrajectoryAsync(park_0);
                                currentState = State.PARK;
                                break;
                            case 1:
                                robot.drive.followTrajectoryAsync(clearWobble2_1);
                                currentState = State.CLEAR_WOBBLE_2;
                                break;
                            case 4:
                                robot.drive.followTrajectoryAsync(park_4);
                                currentState = State.PARK;
                                break;
                        }
                    }
                    break;

                case CLEAR_WOBBLE_2:
                    if (!robot.drive.isBusy()) {
                        robot.drive.followTrajectoryAsync(park_1);
                        currentState = State.PARK;
                    }
                    break;

                case PARK:
                    if (!robot.drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;

                case IDLE:
                    // Do nothing
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update robot continuously in the background, regardless of state
            robot.update();

            // Read pose
            Pose2d poseEstimate = robot.drive.getPoseEstimate();

            // Continually write pose to PoseStorage
            PoseStorage.currentPose = poseEstimate;


            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("robotstate", robot.getRobotState());
            telemetry.addData("rings", rings);
            telemetry.addData("state", currentState);
            telemetry.update();
        }
        // write pose to PoseStorage one last time
        PoseStorage.currentPose = robot.drive.getPoseEstimate();
    }
}