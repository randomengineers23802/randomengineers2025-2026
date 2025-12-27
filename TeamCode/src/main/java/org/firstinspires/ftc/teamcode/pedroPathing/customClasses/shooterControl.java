package org.firstinspires.ftc.teamcode.pedroPathing.customClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;

public class shooterControl {

    public DcMotorEx ShooterL;
    public DcMotorEx ShooterR;
    public Limelight3A limelight;
    private Follower follower;

    private double previousError = 0;
    private double totalError = 0;
    private ElapsedTime timer = new ElapsedTime();

    PIDFCoefficients shooterPIDF = new PIDFCoefficients(200.0, 0.0, 10.0, 12.3);
    PIDCoefficients autoAimPID = new PIDCoefficients(0.012, 0, 0.0);

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

    public void setShooterVelocity(String range) {
        double targetVelocity = 0;
        switch (range) {
            case "close": targetVelocity = 1100; break;
            case "far": targetVelocity = 1200; break;
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
                break;
            case "red":
                limelight.pipelineSwitch(1);
                break;
        }
    }

    public void autoAim() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double error = result.getTx();
            double deltaTime = timer.seconds();
            timer.reset();

            if (deltaTime > 0.2)
                deltaTime = 0.01;

            double p = autoAimPID.p * error;
            totalError += error * deltaTime;
            double i = autoAimPID.i * totalError;
            double d = autoAimPID.d * ((error - previousError) / deltaTime);
            previousError = error;

            double motorOutput = Math.max(-1.0, Math.min(1.0, p + i + d));
            follower.setTeleOpDrive(0,0, -motorOutput);
        }
        else {
            previousError = 0;
            totalError = 0;
            timer.reset();
            follower.setTeleOpDrive(0, 0, 0);
        }
    }
}