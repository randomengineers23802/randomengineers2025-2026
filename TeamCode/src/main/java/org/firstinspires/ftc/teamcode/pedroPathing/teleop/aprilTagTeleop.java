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
                double tx = Math.toRadians(result.getTx());
                double desiredHeading = follower.getHeading() + tx;
                follower.turnTo(desiredHeading);
                telemetry.addData("desiredHeading", Math.toDegrees(desiredHeading));
                telemetry.addData("tx", Math.toDegrees(tx));
            }
            telemetry.addData("isBusy", follower.isBusy());
            telemetry.addData("actualHeading", Math.toDegrees(follower.getHeading()));
            telemetry.update();
        }
    }
}
