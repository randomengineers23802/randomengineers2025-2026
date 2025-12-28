package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.shooterControl;

@Disabled
@TeleOp(name = "AutoAim Tuner FINAL", group = "Tuning")
public class autoAimTuner extends LinearOpMode {

    private shooterControl shooter;
    private Follower follower;

    // Increments
    double pIncrement = 0.002;
    double iIncrement = 0.0001;
    double dIncrement = 0.002;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Setup Follower exactly like your working code
        follower = Constants.createFollower(hardwareMap);

        // We use a safe default Pose, but we MUST call startTeleopDrive
        follower.setStartingPose(new Pose(0,0,0));
        follower.startTeleopDrive();

        // 2. Setup Shooter
        shooter = new shooterControl(hardwareMap, follower);
        shooter.setPipeline("red"); // Match your working setup

        telemetry.addLine("A/B/X + Dpad to Tune");
        telemetry.addLine("HOLD Left Trigger to AutoAim (Like your TeleOp)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean changed = false;

            // PID Tuning Logic
            if (gamepad1.a) { // Tune P
                if (gamepad1.dpad_up) { shooter.autoAimPID.p += pIncrement; changed = true; }
                if (gamepad1.dpad_down) { shooter.autoAimPID.p -= pIncrement; changed = true; }
            } else if (gamepad1.b) { // Tune I
                if (gamepad1.dpad_up) { shooter.autoAimPID.i += iIncrement; changed = true; }
                if (gamepad1.dpad_down) { shooter.autoAimPID.i -= iIncrement; changed = true; }
            } else if (gamepad1.x) { // Tune D
                if (gamepad1.dpad_up) { shooter.autoAimPID.d += dIncrement; changed = true; }
                if (gamepad1.dpad_down) { shooter.autoAimPID.d -= dIncrement; changed = true; }
            }

            // Logic copied from your working TeleOp (using Left Trigger)
            if (gamepad1.left_trigger > 0.2) {
                shooter.autoAim();
            } else {
                // Manual drive
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            }

            // Update the follower!
            follower.update();

            // Display current PID values so you can write them down
            telemetry.addData("--- TUNING (Hold A, B, or X) ---", "");
            telemetry.addData("P (A)", "%.4f", shooter.autoAimPID.p);
            telemetry.addData("I (B)", "%.4f", shooter.autoAimPID.i);
            telemetry.addData("D (X)", "%.4f", shooter.autoAimPID.d);
            telemetry.addLine("--------------------------------");

            if (shooter.limelight.getLatestResult() != null && shooter.limelight.getLatestResult().isValid()) {
                telemetry.addData("Target X Error", "%.2f", shooter.limelight.getLatestResult().getTx());
            } else {
                telemetry.addLine("Limelight: NO TARGET");
            }

            telemetry.update();

            if (changed) {
                sleep(150); // Slow down the incrementing speed
            }
        }
    }
}