package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.AutoAim;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.RobotTele;
import org.firstinspires.ftc.teamcode.lib.hardware.Intake;
import org.firstinspires.ftc.teamcode.lib.hardware.Robot;
import org.firstinspires.ftc.teamcode.lib.hardware.Shooter;


@TeleOp(group = "advanced")
public class ServoTest extends LinearOpMode {
    private RobotTele robot;
    private Controller controller;
    private Servo servo;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor shooter1 = hardwareMap.get(DcMotor.class, "shooter1");

        // initialize robot
        robot = new RobotTele(hardwareMap, gamepad1, gamepad2);

        servo = hardwareMap.get(Servo.class, "feeder");
        servo.setPosition(0.6);
        controller = new Controller(gamepad1);

        waitForStart();

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // clear cache for bulk reading
            for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }

            Globals.autonomous = true;
            controller.update();
            robot.update();

            if(controller.getdPadUp() == Controller.ButtonState.ON_PRESS) {
                servo.setPosition(servo.getPosition() + 0.02);
            }
            if(controller.getdPadDown() == Controller.ButtonState.ON_PRESS) {
                servo.setPosition(servo.getPosition() - 0.02);
            }
            if(controller.getaButton() == Controller.ButtonState.ON_PRESS) {
                robot.shoot();
            }

            telemetry.addData("pos", servo.getPosition());
            telemetry.update();
        }
    }
}
