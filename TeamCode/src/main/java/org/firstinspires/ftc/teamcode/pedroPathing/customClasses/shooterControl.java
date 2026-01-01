package org.firstinspires.ftc.teamcode.pedroPathing.customClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class shooterControl {

    public DcMotorEx ShooterL;
    public DcMotorEx ShooterR;
    public Limelight3A limelight;
    private Follower follower;

    public double targetGoalX;
    public double targetGoalY;
    private ElapsedTime timer = new ElapsedTime();

    PIDFCoefficients shooterPIDF = new PIDFCoefficients(200.0, 0.0, 10.0, 12.3);

    public shooterControl(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        ShooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        ShooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        ShooterL.setDirection(DcMotorEx.Direction.REVERSE);
        ShooterR.setDirection(DcMotorEx.Direction.FORWARD);
        ShooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ShooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ShooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterPIDF);
        ShooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterPIDF);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        timer.reset();
    }

    public void resetFollower() {
        follower.breakFollowing();
    }

    public void setShooterVelocity(String range) {
        double targetVelocity = 0;
        switch (range) {
            case "close":
                targetVelocity = 1080;
                break;
            case "far":
                targetVelocity = 1200;
                break;
        }
        ShooterL.setVelocity(targetVelocity);
        ShooterR.setVelocity(targetVelocity);
    }

    public void shooterStop() {
        ShooterL.setPower(0);
        ShooterR.setPower(0);
    }

    public void setPipeline(String goalColor) {
        switch (goalColor) {
            case "blue":
                limelight.pipelineSwitch(0);
                targetGoalX = 16;
                targetGoalY = 131;
                break;
            case "red":
                limelight.pipelineSwitch(1);
                targetGoalX = 128;
                targetGoalY = 131;
                break;
        }
    }

    public void autoAim() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double currentHeading = follower.getPose().getHeading();

            double targetHeading = currentHeading + Math.toRadians(tx);
            follower.turnTo(targetHeading);
        }
        else {
            Pose currentPose = follower.getPose();
            double errorX = targetGoalX - currentPose.getX();
            double errorY = targetGoalY - currentPose.getY();

            double targetAngle = Math.atan2(errorY, errorX) + Math.PI;

            double angularError = targetAngle - currentPose.getHeading();
            while (angularError > Math.PI) angularError -= 2 * Math.PI;
            while (angularError < -Math.PI) angularError += 2 * Math.PI;

            if (Math.abs(angularError) > Math.toRadians(10.0)) {
                double setPower = 0.5;

                double rotationPower = Math.signum(angularError) * setPower;

                follower.setTeleOpDrive(0, 0, rotationPower, false);
            } else {
                follower.setTeleOpDrive(0, 0, 0, false);
            }
        }
    }
}