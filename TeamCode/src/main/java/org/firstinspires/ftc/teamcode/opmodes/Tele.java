package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.RobotTele;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "advanced")
public class Tele extends LinearOpMode {
    private RobotTele robot;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        // initialize robot
        robot = new RobotTele(hardwareMap, gamepad1, gamepad2, telemetry);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // clear cache for bulk reading
            for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }

            // update robot and all it's elements
            robot.updateRT();

            if(gamepad1.a) {
                telemetry.addData("tele class", "ture");
            }

            //Telemetry
            telemetry.addData("spinner motor", robot.arm.motor.getCurrentPosition());
            telemetry.addData("motorOffset", robot.arm.motorOffset);
            telemetry.addData("PIDF", robot.arm.motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

            telemetry.addData("robot state", robot.getRobotState());
            telemetry.update();
        }
    }
}
