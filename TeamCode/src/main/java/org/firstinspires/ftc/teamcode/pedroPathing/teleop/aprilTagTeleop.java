package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.Constants;

@TeleOp(name="aprilTagTeleop")
public class aprilTagTeleop extends OpMode {

    private Limelight3A limelight;
    private Follower follower;

    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;

    // PID constants for rotation
    private final double kP = 0.04;
    private final double kI = 0.0;
    private final double kD = 0.025;

    private double integral = 0;
    private double lastError = 0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void start() {
        // This is called after waitForStart automatically
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();

        LLResult result = limelight.getLatestResult();
        double rotationPower = 0;

        if (result != null && result.isValid()) {
            double error = result.getTx();
            integral += error * getRuntime();
            double derivative = error - lastError;

            rotationPower = kP * error + kI * integral + kD * derivative;
            rotationPower = Math.max(-1.0, Math.min(1.0, rotationPower));

            lastError = error;

            telemetry.addData("Rotation error (deg)", error);
            telemetry.addData("Rotation power", rotationPower);
        }

        motorFL.setPower(rotationPower);
        motorBL.setPower(rotationPower);
        motorFR.setPower(-rotationPower);
        motorBR.setPower(-rotationPower);

        Pose pose = follower.getPose();
        telemetry.addData("pose X", String.format("%.3f", pose.getX()));
        telemetry.addData("pose Y", String.format("%.3f", pose.getY()));
        telemetry.addData("pose Heading (deg)", String.format("%.3f", Math.toDegrees(pose.getHeading())));
        telemetry.update();
    }
}
