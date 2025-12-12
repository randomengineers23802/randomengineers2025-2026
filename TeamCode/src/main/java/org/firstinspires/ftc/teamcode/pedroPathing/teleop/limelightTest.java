package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LimelightTest")
public class limelightTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0); // pipeline 0 only detects april tag 22
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // 2D tracking using camera offsets
                double tx = result.getTx(); // horizontal offset (degrees)
                double ty = result.getTy(); // vertical offset (degrees)
                double ta = result.getTa(); // target area

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("ta", ta);
                telemetry.update();
            }
        }
    }
}
