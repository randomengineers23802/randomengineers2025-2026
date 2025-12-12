package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="aprilTagTeleop")
public class aprilTagTeleop extends LinearOpMode {
    private Limelight3A limelight;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        limelight.pipelineSwitch(0);  // pipeline 0 only detects april tag 22
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double desiredHeading = follower.getHeading() + Math.toRadians(result.getTx());
                Pose current = follower.getPose();
                follower.setPose(new Pose(current.getX(), current.getY(), desiredHeading));
                telemetry.addData("Heading (rad)", desiredHeading);
                telemetry.addData("tx (rad)", Math.toRadians(result.getTx()));
            }

            Pose pose = follower.getPose();
            telemetry.addData("pose X", String.format("%.3f", pose.getX()));
            telemetry.addData("pose Y", String.format("%.3f", pose.getY()));
            telemetry.addData("pose Heading (deg)", String.format("%.3f", Math.toDegrees(pose.getHeading())));
            telemetry.update();
        }
    }
}
