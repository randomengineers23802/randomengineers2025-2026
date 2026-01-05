package org.firstinspires.ftc.teamcode.pedroPathing.customClasses;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class robotControl {

    public DcMotorEx ShooterL;
    public DcMotorEx ShooterR;
    public Limelight3A limelight;
    private DcMotor intake;
    private DcMotor belt ;
    private Servo BlueBoi;
    private Follower follower;
    private TelemetryManager panelsTelemetry;
    public double targetGoalX;
    public double targetGoalY;
    private ElapsedTime timer = new ElapsedTime();

    PIDFCoefficients shooterPIDF = new PIDFCoefficients(80.0, 0.0, 0.0, 12.3);

    public robotControl(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        ShooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        ShooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        ShooterL.setDirection(DcMotorEx.Direction.REVERSE);
        ShooterR.setDirection(DcMotorEx.Direction.FORWARD);
        ShooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ShooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ShooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterPIDF);
        ShooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterPIDF);
        intake = hardwareMap.get(DcMotor.class, "intake");
        belt = hardwareMap.get(DcMotor.class, "belt");
        belt.setDirection(DcMotor.Direction.REVERSE);
        BlueBoi = hardwareMap.get(Servo.class, "BlueBoi");
        BlueBoi.setPosition(0.65);

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

    public void beltOn() {
        belt.setPower(0.8);
    }

    public void beltOff() {
        belt.setPower(0.0);
    }

    public void intakeOn() {
        intake.setPower(1.0);
    }

    public void intakeOff() {
        intake.setPower(0.0);
    }

    public void blueBoiOpen() {
        BlueBoi.setPosition(1.0);
    }

    public void blueBoiClosed() {
        BlueBoi.setPosition(0.65);
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