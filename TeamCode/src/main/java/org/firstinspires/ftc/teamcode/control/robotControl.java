package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.math.Vector;

public class robotControl {

    public DcMotorEx ShooterL;
    public DcMotorEx ShooterR;
    public Limelight3A limelight;
    private DcMotor intake;
    private DcMotor belt ;
    private Servo BlueBoi;
    private Follower follower;
    private Pose goalTarget;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private static final double flywheelOffset = 0;
    private static final double flywheelMinSpeed = 0;
    private static final double flywheelMaxSpeed = 2000;
    private static final double scoreHeight = 26;
    private static final double passthroughPointRadius = 3;

    public double flywheelInchesPerSec;

    PIDFCoefficients shooterLPIDF = new PIDFCoefficients(60.0, 0.0, 0.0, 11.875);
    PIDFCoefficients shooterRPIDF = new PIDFCoefficients(60.0, 0.0, 0.0, 11.62);
    PIDFCoefficients aimPIDF = new PIDFCoefficients(1.2, 0.0, 0.1, 0.02);

    public robotControl(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        ShooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        ShooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        ShooterL.setDirection(DcMotorEx.Direction.REVERSE);
        ShooterR.setDirection(DcMotorEx.Direction.FORWARD);
        ShooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ShooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ShooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterLPIDF);
        ShooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterRPIDF);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        belt = hardwareMap.get(DcMotor.class, "belt");
        belt.setDirection(DcMotor.Direction.REVERSE);
        BlueBoi = hardwareMap.get(Servo.class, "BlueBoi");
        BlueBoi.setPosition(0.65);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        timer.reset();
    }

    public double autoAim() {
        Pose currentPose = follower.getPose();
        double distanceX = goalTarget.getX() - currentPose.getX();
        double distanceY = goalTarget.getY() - currentPose.getY();
        double angleToGoal = Math.atan2(distanceY, distanceX) + Math.PI;

        double error = angleToGoal - currentPose.getHeading();
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        double dt = timer.seconds();
        timer.reset();

        double derivative = 0;
        if (dt > 0.001) {
            derivative = (error - lastError) / dt;
            lastError = error;
        }
        double feedForward = Math.signum(error) * aimPIDF.f;
        double aimPower = (error * aimPIDF.p) + (derivative * aimPIDF.d) + feedForward;

        if (Math.abs(error) < Math.toRadians(1.0))
            return 0;
        else {
            return Range.clip(aimPower, -1.0, 1.0);
        }
    }

    public ShotParameters updateShooting() {
        Pose currentPose = follower.getPose();
        ShotParameters shotParameters = calculateShotVectorAndTurret(currentPose);

        double error = shotParameters.heading - currentPose.getHeading();
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        double dt = timer.seconds();
        timer.reset();

        double derivative = 0;
        if (dt > 0.001) {
            derivative = (error - lastError) / dt;
            lastError = error;
        }
        double feedForward = Math.signum(error) * aimPIDF.f;
        double aimPower = (error * aimPIDF.p) + (derivative * aimPIDF.d) + feedForward;

        if (Math.abs(error) < Math.toRadians(1.0))
            return new ShotParameters(shotParameters.flywheelTicks, 0);
        else {
            return new ShotParameters(shotParameters.flywheelTicks, Range.clip(aimPower, -1.0, 1.0));
        }
    }

    public void setShooterVelocity(String range) {
        double targetVelocity = 0;
        switch (range) {
            case "close":
                targetVelocity = 1100;
                break;
            case "far":
                targetVelocity = 1180;
                break;
        }
        ShooterL.setVelocity(targetVelocity);
        ShooterR.setVelocity(targetVelocity);
    }

    public void setShooterVelocity(double ticks) {
        ShooterL.setVelocity(ticks);
        ShooterR.setVelocity(ticks);
    }

    public void shooterStop() {
        ShooterL.setPower(0);
        ShooterR.setPower(0);
    }

    public void setAlliance(String goalColor) {
        switch (goalColor) {
            case "blue":
                goalTarget = new Pose(4, 140);
                break;
            case "red":
                goalTarget = new Pose(140, 140);
                break;
        }
    }

    public Vector robotToGoalVector(Pose currentPose) {
        double dx = goalTarget.getX() - currentPose.getX();
        double dy = goalTarget.getY() - currentPose.getY();
        return new Vector(new Pose(dx, dy));
    }

    public static double getFlywheelTicksFromVelocity(double velocity) {
        return Range.clip((0.0176676 * Math.pow(velocity, 2)) - (5.11447 * velocity) + 1366.42995 + flywheelOffset, flywheelMinSpeed, flywheelMaxSpeed); // originally divided velocity by 12
    }

    public ShotParameters calculateShotVectorAndTurret(Pose currentPose) {
        Vector robotToGoalVector = robotToGoalVector(currentPose);
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - passthroughPointRadius;
        double y = scoreHeight;

        double fixedExitAngle = Math.toRadians(48.3);
        double cosTheta = Math.cos(fixedExitAngle);
        double flywheelSpeed = Math.sqrt((g * Math.pow(x, 2)) / (2 * Math.pow(cosTheta, 2) * (x * Math.tan(fixedExitAngle) - y)));

        Vector robotVelocity = follower.getVelocity();
        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();
        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double vz = flywheelSpeed * Math.sin(fixedExitAngle);
        double time = x / (flywheelSpeed * Math.cos(fixedExitAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);

        flywheelSpeed = Math.sqrt(Math.pow(vz, 2) + Math.pow(nvr, 2));
        flywheelInchesPerSec = flywheelSpeed;

        double turretVelCompOffset = Math.atan2(perpendicularComponent, ivr);

        double heading = robotToGoalVector.getTheta() - turretVelCompOffset + Math.PI;
        double flywheelTicks = getFlywheelTicksFromVelocity(flywheelSpeed);

        return new ShotParameters(flywheelTicks, heading);
    }

    public void beltOn() { belt.setPower(0.8); }
    public void beltOff() { belt.setPower(0.0); }
    public void intakeOn() { intake.setPower(1.0); }
    public void intakeOff() { intake.setPower(0.0); }
    public void blueBoiOpen() { BlueBoi.setPosition(1.0); }
    public void blueBoiClosed() { BlueBoi.setPosition(0.65); }
}