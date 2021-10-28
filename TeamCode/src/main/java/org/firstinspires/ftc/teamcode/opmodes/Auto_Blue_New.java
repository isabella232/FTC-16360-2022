package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.Targets;
import org.firstinspires.ftc.teamcode.lib.Vision;
import org.firstinspires.ftc.teamcode.lib.hardware.Intake;
import org.firstinspires.ftc.teamcode.lib.hardware.Robot;
import org.firstinspires.ftc.teamcode.lib.hardware.Shooter;
import org.firstinspires.ftc.teamcode.lib.hardware.Transfer;
import org.opencv.core.Mat;

import java.util.Arrays;


@Autonomous(group = "opmodes")
public class Auto_Blue_New extends LinearOpMode {

    private void updateDistance(Pose2d shootingPos) {
        Shooter.setDistance((new Vector2d(72, 35).minus(shootingPos.vec())).norm());
    }

    // robot class
    Robot robot;

    // This enum defines our "state"
    // This is defines the possible steps our program will take
    enum State {
        DRIVE_TO_SHOOT_1,
        SHOOTING_1,
        INTAKING_1,
        WAIT_1,
        SHOOTING_2,
        INTAKING_2,
        WAIT_2,
        SHOOTING_3,
        DELIVER_WOBBLE_1,
        DROP_WOBBLE_1,
        COLLECTRINGS_0,
        COLLECTRINGS_1,
        TAKEIN_0,
        TAKEIN_1,
        CLEAR_WOBBLE_1,
        GET_WOBBLE_2,
        GRAB_WOBBLE_2,
        DRIVE_TO_SHOOT_2,
        SHOOTING_4_HELPER,
        SHOOTING_4,
        DELIVER_WOBBLE_2,
        DROP_WOBBLE_2,
        CLEAR_WOBBLE_2,
        PARK,
        IDLE,
        TRAJECTORY_1,
        SHOOT_POWERSHOT_0,
        SHOOTING_POWERSHOT_0,
        SHOOT_POWERSHOT_1,
        SHOOTING_POWERSHOT_1,
        SHOOT_POWERSHOT_2,
        SHOOTING_POWERSHOT_2
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    Pose2d startPose = new Pose2d(-62, 23, Math.toRadians(3));

    @Override
    public void runOpMode() throws InterruptedException {
        // Variable for rings
        int rings = 0;

        Globals.autonomous = true;

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
        double wobbleDropTime = 0.05;

        // Time for wobble goal to be grabbed
        double wobbleGrabTime = 0.8;

        // a timer for timed actions
        ElapsedTime waitTimer = new ElapsedTime();

        //
        Trajectory driveToPowerShot0 = robot.drive.trajectoryBuilder(startPose)     //ToDo Name
                .lineToLinearHeading(new Pose2d(-5, 24, Math.toRadians(3)))//-3, 16
                .build();

        //
        Trajectory driveToPowershot1 = robot.drive.trajectoryBuilder(driveToPowerShot0.end())       //ToDo Name
                .lineToLinearHeading(
                        new Pose2d(-5, 0 , Math.toRadians(1)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


/*        Trajectory driveToPowershot2 = robot.drive.trajectoryBuilder(driveToPowershot1.end())
                .lineToLinearHeading(new Pose2d(-4, 2, Math.toRadians(355)))
                .build();
*/
        // trajectory moves to the spot to shoot at the high goal
        Trajectory driveToShoot1 = robot.drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-46, 30, Math.toRadians(3)))
                .build();


        // Trajectory to intake single ring
        Trajectory intaking1_1 = robot.drive.trajectoryBuilder(driveToShoot1.end())
                .lineToLinearHeading(new Pose2d(-10, 30, Math.toRadians(3)))
                .build();

        Trajectory driveToPowerShot0_1 = robot.drive.trajectoryBuilder(intaking1_1.end())
                .lineToLinearHeading(new Pose2d (-4, 16, Math.toRadians(355)))
                .build();


        // Trajectory to intake first several rings of 4 stack
        Trajectory intaking1_4 = robot.drive.trajectoryBuilder(driveToShoot1.end())
                .lineToLinearHeading(new Pose2d(-25, 30, Math.toRadians(3)))
                .build();


        // Trajectory to intake remaining rings of 4 stack
        Trajectory intaking2_4 = robot.drive.trajectoryBuilder(intaking1_4.end())
                .lineToLinearHeading(new Pose2d(-10, 36, Math.toRadians(0)))
                .build();


        // Trajectories to deposit first wobble goal
        Trajectory depositWobble1_0 = robot.drive.trajectoryBuilder(driveToPowershot1.end())
                .lineToLinearHeading(new Pose2d(10, 45, Math.toRadians(270)))
                .build();

        Trajectory depositWobble1_1 = robot.drive.trajectoryBuilder(driveToPowershot1.end())
                .splineTo(new Vector2d(45,  -10), Math.toRadians(0))
                .splineTo(new Vector2d(58, 16), Math.toRadians(90))
                .splineTo(new Vector2d(20, 45), Math.toRadians(180))
                .build();

        Trajectory depositWobble1_4 = robot.drive.trajectoryBuilder(intaking2_4.end())
                .lineToLinearHeading(new Pose2d(62, 42, Math.toRadians(270)))
                .build();


        // Trajectories to clear first wobble goal after depositing
        Trajectory clearWobble1_0 = robot.drive.trajectoryBuilder(depositWobble1_0.end())
                .splineTo(new Vector2d(40, 40), 0)
                .splineTo(new Vector2d(58, 8), Math.toRadians(270))
                .splineTo(new Vector2d(0, 24), Math.toRadians(90))
                .build();

        Trajectory clearWobble1_1 = robot.drive.trajectoryBuilder(depositWobble1_1.end())
                .lineToLinearHeading(new Pose2d(-24, 62, Math.toRadians(15)))
                .build();

        Trajectory clearWobble1_4 = robot.drive.trajectoryBuilder(depositWobble1_4.end())
                .lineTo(new Vector2d(50, 30))
                .build();
        /*
        Trajectory intakerings_0 = robot.drive.trajectoryBuilder(clearWobble1_0.end())
                .lineTo(new Vector2d(55, 50))
                .build();

        Trajectory intakerings_1 = robot.drive.trajectoryBuilder(clearWobble1_1.end())
                .lineTo(new Vector2d(55, 50))
                .build();



        Trajectory takein_0 = robot.drive.trajectoryBuilder(clearWobble1_0.end())
                .lineTo(new Vector2d(55, -9), 0)
                .build();

        Trajectory takein_1 = robot.drive.trajectoryBuilder(clearWobble1_1.end())
                .lineTo(new Vector2d(55, -9))
                .build();
*/



        // Trajectories to pick up the second wobble goal
        Trajectory getWobble2_0 = robot.drive.trajectoryBuilder(clearWobble1_0.end())
                .lineToLinearHeading(new Pose2d(-34.5, 44, Math.toRadians(0)))
                .build();

        Trajectory getWobble2_1 = robot.drive.trajectoryBuilder(clearWobble1_1.end())
                .lineToLinearHeading(new Pose2d(-34.5, 50, Math.toRadians(0)))
                .build();

        Trajectory getWobble2_4 = robot.drive.trajectoryBuilder(clearWobble1_4.end())
                .lineToLinearHeading(new Pose2d(-34.5, 50, Math.toRadians(0)))
                .build();

        Trajectory drivetoshoot_2 = robot.drive.trajectoryBuilder(getWobble2_1.end())
                .lineToLinearHeading(new Pose2d(-50, 30, Math.toRadians(0)))
                .build();

        Trajectory shooting2 = robot.drive.trajectoryBuilder(drivetoshoot_2.end())       //ToDo Name
                .lineToLinearHeading(
                        new Pose2d(-5, 30 , Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        // Trajectories to deposit second wobble goal
        Trajectory deliverWobble2_0 = robot.drive.trajectoryBuilder(drivetoshoot_2.end())
                .lineToLinearHeading(new Pose2d(15, 41, Math.toRadians(270)))
                .build();

        Trajectory deliverWobble2_1 = robot.drive.trajectoryBuilder(shooting2.end())
                .lineToLinearHeading(new Pose2d(28, 24, Math.toRadians(270)))
                .build();

        Trajectory deliverWobble2_4 = robot.drive.trajectoryBuilder(getWobble2_4.end())
                .lineToLinearHeading(new Pose2d(53, 41, Math.toRadians(270)))
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
                .lineToLinearHeading(new Pose2d(10, 10, Math.toRadians(270)))
                .build();

        Trajectory park_4 = robot.drive.trajectoryBuilder(deliverWobble2_4.end())
                .lineToLinearHeading(new Pose2d(10, 38, Math.toRadians(270)))
                .build();




        waitForStart();

        if (isStopRequested()) return;

        // clear cache for bulk reading
        for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }

        currentState = State.TRAJECTORY_1;

        // scan the rings
        waitTimer.reset();
        /*while(waitTimer.seconds() <= 1.5)
        {
            rings = vision.getRingAmount();
        }

         */
        Globals.rings = rings;
        rings = 1;

        //drop the intake
        robot.dropIntake();
        waitTimer.reset();
        while(waitTimer.seconds() <= 0.3)
        {
        }
        updateDistance(driveToShoot1.end().minus(new Pose2d(10, 0, 0)));
        if(rings == 4)  {
            robot.drive.followTrajectoryAsync(driveToShoot1);
        } else {
            robot.drive.followTrajectoryAsync(driveToPowerShot0);
            Globals.currentTargetType = Targets.TargetType.OUTER_POWERSHOT;
            robot.forceRetract();
        }
        robot.setRobotState(Robot.RobotState.SHOOTING);

        //robot.wobbleStoringPos();

        while (opModeIsActive() && !isStopRequested()) {
            // clear cache for bulk reading
            for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }

            // We update robot continuously in the background, regardless of state
            robot.update();

            // Read pose
            Pose2d poseEstimate = robot.drive.getPoseEstimate();

            // Continually write pose to PoseStorage
            PoseStorage.currentPose = poseEstimate;


            // The state machine logic

            // We define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    if (rings != 4) {
                        if(!robot.drive.isBusy()) {
                            robot.drive.followTrajectoryAsync(driveToPowershot1);
                            currentState = State.SHOOT_POWERSHOT_0;
                        }
                    } else {
                        updateDistance(driveToShoot1.end().minus(new Pose2d(5, 0, 0)));
                        robot.drive.followTrajectoryAsync(driveToShoot1);
                        robot.setRobotState(Robot.RobotState.SHOOTING);
                        currentState = State.DRIVE_TO_SHOOT_1;
                    }
                    break;


                case SHOOT_POWERSHOT_0:
                    robot.forceRetract();
                    if (poseEstimate.getY() < 20.2) {
                        currentState = State.SHOOT_POWERSHOT_1;
                        robot.forceExtend();
                        Globals.currentTargetType = Targets.TargetType.CENTER_POWERSHOT;
                        //waitTimer.reset();
                    }
                    break;
/*
                case SHOOTING_POWERSHOT_0:
                    robot.shoot();
                    if (waitTimer.seconds() >= shootTime) {
                        robot.drive.followTrajectoryAsync(driveToPowershot1);
                        currentState = State.SHOOT_POWERSHOT_1;
                        waitTimer.reset();
                    }
                    break;
*/
                case SHOOT_POWERSHOT_1:
                    robot.forceRetract();
                    if (poseEstimate.getY() < 14) {
                        currentState = State.SHOOT_POWERSHOT_2;
                        robot.forceExtend();
                        Globals.currentTargetType = Targets.TargetType.INNER_POWERSHOT;

                        //waitTimer.reset();
                    }
                    break;
/*
                case SHOOTING_POWERSHOT_1:
                    robot.shoot();
                    if (waitTimer.seconds() >= shootTime) {
                        robot.drive.followTrajectoryAsync(driveToPowershot2);
                        waitTimer.reset();
                        currentState = State.SHOOT_POWERSHOT_2;
                    }
                    break;
*/
                case SHOOT_POWERSHOT_2:
                    robot.forceRetract();
                    if (poseEstimate.getY() < 8.5) {
                        currentState = State.SHOOTING_POWERSHOT_2;
                        robot.forceExtend();
                    }
                    break;

                case SHOOTING_POWERSHOT_2:  //ToDo Name
                    if (!robot.drive.isBusy()) {
                        robot.setRobotState(Robot.RobotState.DRIVING);
                        currentState = State.DELIVER_WOBBLE_1;
                        if (rings == 0)
                            robot.drive.followTrajectoryAsync(depositWobble1_0);
                        else if(rings == 1) {
                            robot.drive.followTrajectoryAsync(depositWobble1_1);
                            robot.wobbleOuttakingPos();
                            robot.intake();
                        }
                    }
                    break;


                case DRIVE_TO_SHOOT_1:
                    if (!robot.drive.isBusy()) {
                        currentState = State.SHOOTING_1;
                        waitTimer.reset();
                    }
                    break;

                case SHOOTING_1:
                    robot.shoot();

                    switch (rings) {
                        case 1:
                            if (waitTimer.seconds() >= shootTime) {
                                robot.wobbleOuttakingPos();
                                robot.drive.followTrajectoryAsync(intaking1_1);
                                updateDistance(intaking1_1.end());
                                robot.intake();
                                currentState = State.INTAKING_1;
                            }
                            break;
                        case 4:
                            if (waitTimer.seconds() >= 3 * shootTime) {
                                robot.wobbleOuttakingPos();
                                robot.drive.followTrajectoryAsync(intaking1_4);
                                updateDistance(intaking1_4.end());
                                robot.intake();
                                currentState = State.INTAKING_1;
                            }
                            break;
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
                        if (rings == 1) {
                            updateDistance(driveToPowerShot0_1.end().minus(new Pose2d(5, 0, 0)));
                            robot.drive.followTrajectoryAsync(driveToPowerShot0_1);
                            robot.setRobotState(Robot.RobotState.SHOOTING);
                            currentState = State.SHOOT_POWERSHOT_0;
                       }
                        else
                            currentState = State.SHOOTING_2;
                        waitTimer.reset();
                    }
                    break;

                case SHOOTING_2:
                    robot.shoot();
                    if (waitTimer.seconds() >= 2*shootTime) {
                        if (rings == 4) {
                            robot.drive.followTrajectoryAsync(intaking2_4);
                            updateDistance(intaking2_4.end());
                            robot.intake();
                            currentState = State.INTAKING_2;
                        } else {
                            robot.drive.followTrajectoryAsync(depositWobble1_1);
                            robot.setRobotState(Robot.RobotState.DRIVING);
                            robot.wobbleOuttakingPos();
                            currentState = State.DELIVER_WOBBLE_1;
                        }
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
                        waitTimer.reset();
                    }
                break;

                case SHOOTING_3:
                    robot.shoot();
                    if (waitTimer.seconds() >= 3*shootTime) {
                        robot.drive.followTrajectoryAsync(depositWobble1_4);
                        robot.setRobotState(Robot.RobotState.DRIVING);
                        robot.wobbleOuttakingPos();
                        currentState = State.DELIVER_WOBBLE_1;
                    }
                    break;

                case DELIVER_WOBBLE_1:
                    if (!robot.drive.isBusy()) {
                        robot.wobbleOuttakingPos();
                        robot.wobblegripperOpen();
                        currentState = State.DROP_WOBBLE_1;
                        waitTimer.reset();
                    }
                    break;

                case DROP_WOBBLE_1:
                    if (waitTimer.seconds() >= wobbleDropTime) {
                        switch(rings) {
                            case 0:
                                robot.drive.followTrajectoryAsync(clearWobble1_0);
                                robot.intake();
                                currentState = State.CLEAR_WOBBLE_1;
                                break;
                            case 1:
                                robot.drive.followTrajectoryAsync(clearWobble1_1);
                                robot.intake();
                                currentState = State.CLEAR_WOBBLE_1;
                                break;
                            case 4:
                                robot.drive.followTrajectoryAsync(clearWobble1_4);
                                currentState = State.CLEAR_WOBBLE_1;
                                break;
                        }
                    }
                    break;/*
                case COLLECTRINGS_0:
                    if (!robot.drive.isBusy()) {
                        robot.intake();
                        robot.drive.followTrajectoryAsync(intakerings_0);
                        currentState = State.TAKEIN_0;
                    }
                    break;

                case COLLECTRINGS_1:
                    if (!robot.drive.isBusy()) {
                        robot.intake();
                        robot.drive.followTrajectoryAsync(intakerings_1);
                        currentState = State.TAKEIN_1;
                    }
                    break;

                case TAKEIN_0:
                    if (!robot.drive.isBusy()) {
                        robot.drive.followTrajectoryAsync(takein_0);
                        currentState = State.CLEAR_WOBBLE_1;
                    }
                    break;

                case TAKEIN_1:
                    if (!robot.drive.isBusy()) {
                        robot.drive.followTrajectoryAsync(takein_1);
                        currentState = State.CLEAR_WOBBLE_1;
                    }
                    break;
                    */


                case CLEAR_WOBBLE_1:
                    if (!robot.drive.isBusy()) {
                        robot.intakeIdle();
                        robot.wobbleIntakingPos();
                        switch(rings) {
                            case 0:
                                robot.intakeIdle();
                                robot.drive.followTrajectoryAsync(getWobble2_0);
                                robot.transferIdle();
                                break;
                            case 1:
                                robot.intakeIdle();
                                robot.drive.followTrajectoryAsync(getWobble2_1);
                                robot.transferIdle();
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
                        if (rings != 4) {
                            currentState = State.GRAB_WOBBLE_2;
                        }
                        else
                            currentState = State.GRAB_WOBBLE_2;
                        waitTimer.reset();
                    }
                    break;

                case DRIVE_TO_SHOOT_2:
                    if (!robot.drive.isBusy()) {
                        robot.wobbleOuttakingPos();
                        robot.drive.followTrajectoryAsync(shooting2);
                        Globals.currentTargetType = Targets.TargetType.HIGHGOAL;
                        robot.setRobotState(Robot.RobotState.SHOOTING);
                        robot.intake();
                        currentState = State.SHOOTING_4_HELPER;
                    }
                    break;

                case SHOOTING_4_HELPER: // ToDO change name
                    robot.shoot();
                    updateDistance(poseEstimate.plus(new Pose2d(15, 0, 0)));
                    if (!robot.drive.isBusy()) {
                        robot.transferIdle();
                        robot.setRobotState(Robot.RobotState.DRIVING);
                        waitTimer.reset();
                        currentState = State.DELIVER_WOBBLE_2;
                        switch (rings) {
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
                    }
                    break;

                case SHOOTING_4:
                    robot.shoot();
                    if (waitTimer.seconds() >= 3 * shootTime) {
                        currentState =  State.DELIVER_WOBBLE_2;
                    }
                    break;

                case GRAB_WOBBLE_2:
                    if (waitTimer.seconds() >= wobbleGrabTime) {
                        switch (rings) {
                            case 0:
                                robot.drive.followTrajectoryAsync(deliverWobble2_0);
                                break;
                            case 1:
                                currentState = State.DRIVE_TO_SHOOT_2;
                                robot.drive.followTrajectoryAsync(drivetoshoot_2);
                                break;
                            case 4:
                                if (waitTimer.seconds() >= wobbleGrabTime) {
                                    robot.wobbleOuttakingPos();
                                    robot.drive.followTrajectoryAsync(deliverWobble2_4);
                                    currentState = State.DELIVER_WOBBLE_2;
                                }
                                break;
                        }
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
                        Globals.autonomous = false;
                    }
                    break;

                case IDLE:
                    // Do nothing
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("rings", rings);
            telemetry.addData("state", currentState);
            telemetry.update();
        }
        // write pose to PoseStorage one last time
        PoseStorage.currentPose = robot.drive.getPoseEstimate();
    }
}