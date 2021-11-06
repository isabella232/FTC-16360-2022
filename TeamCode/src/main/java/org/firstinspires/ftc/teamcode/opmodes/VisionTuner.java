package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Vision;


@TeleOp(group = "advanced")
public class VisionTuner extends LinearOpMode {
    private Vision vision;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {


        // initialize robot
        vision = new Vision(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            // clear cache for bulk reading
            for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }

            telemetry.addData("num", vision.pipeline.avg1);
            telemetry.update();

        }
    }
}
