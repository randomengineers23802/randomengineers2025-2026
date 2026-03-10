package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
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

import org.firstinspires.ftc.robotcore.external.function.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class robotControl {
    public DcMotorEx ShooterL;
    public DcMotorEx ShooterR;
    public Limelight3A limelight;
    private DcMotor intake;
    public DcMotorEx belt;
    private Servo BlueBoi;
    public Servo kickstand1;
    public Servo kickstand2;
    private Follower follower;
    private Pose goalTarget;
    public Supplier<PathChain> endgamePark;
    private ElapsedTime timer = new ElapsedTime();
    private Servo light;
    private double lastError = 0;

    private static final double flywheelMinSpeed = 940;
    private static final double flywheelMaxSpeed = 1250;
    private static final double scoreHeight = 26;
    private static final double passthroughPointRadius = 2;

    public double flywheelInchesPerSec;

    PIDFCoefficients shooterLPIDF = new PIDFCoefficients(120.0, 0.0, 0.0, 13.1);
    PIDFCoefficients shooterRPIDF = new PIDFCoefficients(120.0, 0.0, 0.0, 12.5);
    PIDFCoefficients beltPIDF = new PIDFCoefficients(0.0, 0.0, 0.0, 12.7);
    PIDFCoefficients aimPIDF = new PIDFCoefficients(1.2, 0.0, 0.1, 0.02); //may need to increase for shooting on the move

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
        belt = hardwareMap.get(DcMotorEx.class, "belt");
        belt.setDirection(DcMotor.Direction.REVERSE);
        belt.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        belt.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, beltPIDF);
        BlueBoi = hardwareMap.get(Servo.class, "BlueBoi");
        BlueBoi.setPosition(0.65);
        kickstand1 = hardwareMap.get(Servo.class, "kickstand1");
        kickstand2 = hardwareMap.get(Servo.class, "kickstand2");
        light = hardwareMap.get(Servo.class, "light");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        timer.reset();
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
            return new ShotParameters(shotParameters.flywheelTicks, Range.clip(aimPower, -0.8, 0.8));
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

    public void kickstandPosition(double position) {
        kickstand1.setPosition(position);
        kickstand2.setPosition(position);
    }

    public void setLightColor(double value) {
        light.setPosition(value);
    }

    public void kickstandUp() {
        kickstand1.setPosition(0.14);
        kickstand2.setPosition(0.88);
    }

    public void kickstandDown() {
        kickstand1.setPosition(0.55);
        kickstand2.setPosition(0.48);
    }

    public boolean relocalize() {
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D limelightPose = result.getBotpose_MT2();

            //Limelight meters to pedro inches
            double xInches = limelightPose.getPosition().x * 39.3701;
            double yInches = limelightPose.getPosition().y * 39.3701;

            //Limelight degrees to pedro radians
            double yawRadians = Math.toRadians(limelightPose.getOrientation().getYaw());

            Pose pedroPose = new Pose(xInches, yInches, yawRadians, FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            follower.setPose(pedroPose);
            return true;
        }
        else
            return false;
    }

//    public void setAlliance(String goalColor) {
//        switch (goalColor) {
//            case "blue":
//                goalTarget = new Pose(4, 140);
//                endgamePark = () -> follower.pathBuilder()
//                        .addPath(new Path(new BezierLine(follower::getPose, new Pose(112, 29))))
//                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.6))
//                        .build();
//                break;
//            case "red":
//                goalTarget = new Pose(140, 140);
//                endgamePark = () -> follower.pathBuilder()
//                        .addPath(new Path(new BezierLine(follower::getPose, new Pose(32, 29))))
//                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.6))
//                        .build();
//                break;
//        }
//    }

    public void setAlliance(Alliance alliance) {
        switch (alliance) {
            case BLUE:
                goalTarget = new Pose(4, 140);
                endgamePark = () -> follower.pathBuilder()
                        .addPath(new Path(new BezierLine(follower::getPose, new Pose(112, 29))))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.6))
                        .build();
                break;
            case RED:
                goalTarget = new Pose(140, 140);
                endgamePark = () -> follower.pathBuilder()
                        .addPath(new Path(new BezierLine(follower::getPose, new Pose(32, 29))))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.6))
                        .build();
                break;
        }
    }

    public Vector robotToGoalVector(Pose currentPose) {
        double dx = goalTarget.getX() - currentPose.getX();
        double dy = goalTarget.getY() - currentPose.getY();
        return new Vector(new Pose(dx, dy));
    }

    public static double getFlywheelTicksFromVelocity(double velocity) {
        return Range.clip((0.024369 * velocity * velocity) - (8.21129 * velocity) + 1632.4453, flywheelMinSpeed, flywheelMaxSpeed);
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

    public void beltOnShoot() { belt.setVelocity(1600); }
    public void beltOnIntake() { belt.setVelocity(2600); }
    public void beltOff() { belt.setPower(0.0); }
    public void intakeOn() { intake.setPower(1.0); }
    public void intakeOff() { intake.setPower(0.05); }
    public void blueBoiOpen() { BlueBoi.setPosition(1.0); }
    public void blueBoiClosed() { BlueBoi.setPosition(0.65); }
}