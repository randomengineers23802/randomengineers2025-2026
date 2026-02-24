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
    private double arcRadius = 80; //guess will need to change later

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

        if (Math.abs(error) < Math.toRadians(1.0)) //won't move if robot is within 1 degree
            return 0;
        else {
            return Range.clip(aimPower, -1.0, 1.0);
        }
    }

    public Pose closestPoseOnArc() {
        Pose currentPose = follower.getPose();

        double distanceX = currentPose.getX() - goalTarget.getX();
        double distanceY = currentPose.getY() - goalTarget.getY();
        double distanceToCenter = Math.hypot(distanceX, distanceY);

        double closestPointX = goalTarget.getX() + (distanceX / distanceToCenter) * arcRadius;
        double closestPointY = goalTarget.getY() + (distanceY / distanceToCenter) * arcRadius;
        double angleToGoal = Math.atan2(closestPointY, closestPointX) + Math.PI;
        Pose arcTargetPose = new Pose(closestPointX, closestPointY, angleToGoal);
        return arcTargetPose;
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

    public void shooterStop() {
        ShooterL.setPower(0);
        ShooterR.setPower(0);
    }

    public void setAlliance(String goalColor) {
        switch (goalColor) {
            case "blue":
                goalTarget = new Pose(4, 135);
                break;
            case "red":
                goalTarget = new Pose(140, 135);
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
}