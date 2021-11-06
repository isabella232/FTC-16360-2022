package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.RobotTele;


@TeleOp(group = "advanced")
public class ServoTest extends LinearOpMode {
    private RobotTele robot;
    private Controller controller;
    private Servo servo;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        // initialize robot
        //robot = new RobotTele(hardwareMap, gamepad1, gamepad2);

        double posL = 0;
        double posR = 0;
        DcMotorEx modor;
        Servo left, right;
        modor = hardwareMap.get(DcMotorEx.class, "Arm");
        left = hardwareMap.get(Servo.class, "leftHand");
        right = hardwareMap.get(Servo.class, "rightHand");
        controller = new Controller(gamepad1);

        waitForStart();

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // clear cache for bulk reading
            for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }

            if(controller.getdPadUp() == Controller.ButtonState.ON_PRESS) {
                posR += 0.005;
            }
            if(controller.getdPadDown() == Controller.ButtonState.ON_PRESS) {
                posR -= 0.005;
            }

            if(controller.getdPadRight() == Controller.ButtonState.ON_PRESS) {
                posL += 0.005;
            }
            if(controller.getdPadLeft() == Controller.ButtonState.ON_PRESS) {
                posL -= 0.005;
            }

            right.setPosition(posR);
            left.setPosition(posL);

            controller.update();

            telemetry.addData("Motor", modor.getCurrentPosition());
            telemetry.addData("posL", posL);
            telemetry.addData("posR", posR);
            telemetry.update();
        }
    }
}
