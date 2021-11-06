package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.Globals;

public class Robot {
    protected HardwareMap hardwareMap;

    public SampleMecanumDrive drive;
    public Spinner spinner;
    public Arm arm;

    public enum RobotState {
        DRIVING
    }

    protected static RobotState robotState;

    protected Pose2d poseEstimate = new Pose2d();

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        //pass HardwareMap to hardware classes, initialize
        spinner = new Spinner(hardwareMap);
        arm = new Arm(hardwareMap);

        // set robot state to idle
        robotState = RobotState.DRIVING;

        // set robot pose
        drive.setPoseEstimate(Globals.currentPose);

        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //turn on bulk reading
        for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void setRobotState(RobotState desiredRobotState) {
        robotState = desiredRobotState;
        switch(desiredRobotState) {

        }
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public void update() {
        // We update hardware classes continuously in the background, regardless of state
        drive.update();
        spinner.update();

        //Update all Classes here
        ///...
    }

    /*
    ** ROBOT CONTROL METHODS
     */

    //...

}
