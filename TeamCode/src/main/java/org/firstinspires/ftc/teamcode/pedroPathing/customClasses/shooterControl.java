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

    public void resetFollowerConstants() {
        follower.setHeadingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.8, 0, 0.02, 0.02));
    }

    public void setShooterVelocity(String range) {
        double targetVelocity = 0;
        switch (range) {
            case "close":
                targetVelocity = 1100;
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
        follower.setHeadingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(1.6, 0, 0.02, 0.02));

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
            follower.turnTo(targetAngle);
        }
    }
}