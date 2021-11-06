package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Vision;

@TeleOp(name="Camera", group="Iterative Opmode")
//@Disabled
public class EasyOpenCVExample extends OpMode {

    Vision vision;

    @Override
    public void init() {
        vision = new Vision(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //telemetry.addData("w", vision.getRingAmount());
        telemetry.log();
    }

    @Override
    public void loop()  {
        //telemetry.addData("w", vision.getRingAmount());
        telemetry.update();

    }
}
