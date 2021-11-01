package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.RobotTele;
import org.firstinspires.ftc.teamcode.lib.hardware.Robot;


@TeleOp(group = "advanced")
public class Test extends LinearOpMode {
    private Robot robot;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
/*
        DcMotor shooter1 = hardwareMap.get(DcMotor.class, "shooter1");

        // initialize robot
        robot = new Robot(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        waitForStart();

        robot.setRobotState(Robot.RobotState.SHOOTING);
        robot.update();

        while (opModeIsActive() && !isStopRequested()) {
            // clear cache for bulk reading
            for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }

            robot.shoot();
            if(Globals.shots > 3) {
                robot.setRobotState(Robot.RobotState.DRIVING);
            }
            robot.update();

            Globals.autonomous = true;

            telemetry.addData("state", robot.getRobotState());
            telemetry.addData("shots", Globals.shots);
            telemetry.update();
        }*/
    }
}
