package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.function.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.BlueBoi;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kickstand;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class RobotControl {
    public Shooter shooter;
    public Intake intake;
    public Belt belt;
    public BlueBoi blueBoi;
    public Kickstand kickstand;
    public Limelight3A limelight;
    private Follower follower;
    private Pose goalTarget;
    public Supplier<PathChain> endgamePark;
    private ElapsedTime timer = new ElapsedTime();
    public Light light;
    private double lastError = 0;

    private static final double flywheelMinSpeed = 940;
    private static final double flywheelMaxSpeed = 1250;
    private static final double scoreHeight = 26;
    private static final double passthroughPointRadius = 2;

    public double flywheelInchesPerSec;
    public double teleOpHeadingOffset;

    PIDFCoefficients aimPIDF = new PIDFCoefficients(1.2, 0.0, 0.1, 0.02); //may need to increase for shooting on the move

    public RobotControl(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        this.shooter = new Shooter(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.belt = new Belt(hardwareMap);
        this.blueBoi = new BlueBoi(hardwareMap);
        this.kickstand = new Kickstand(hardwareMap);
        this.light = new Light(hardwareMap);
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
        shooter.setVelocity(targetVelocity);
    }

//    public void relocalize() {
//        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
//        LLResult result = limelight.getLatestResult();
//        if (result != null && result.isValid()) {
//            Pose3D limelightPose = result.getBotpose_MT2();
//            if (limelightPose != null) {
//                //Limelight meters to pedro inches
//                double xInches = limelightPose.getPosition().x * 39.3701;
//                double yInches = limelightPose.getPosition().y * 39.3701;
//
//                //Limelight degrees to pedro radians
//                //double yawRadians = Math.toRadians(limelightPose.getOrientation().getYaw(Angle));
//
//                pose pedroPose = new pose(xInches, yInches, limelightPose.getOrientation().getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE)
//                        .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
//                follower.setPose(pedroPose);
//            }
//        }
//    }

    public void relocalize() {
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D limelightPose = result.getBotpose_MT2();
            if (limelightPose != null) {
                //Limelight meters to pedro inches
                double xInches = limelightPose.getPosition().x * 39.3701 + 72;
                double yInches = limelightPose.getPosition().y * 39.3701 + 144 + 39.3701; //for some reason you have to add one meter to the y pose
                follower.setPose(new Pose(xInches, yInches, limelightPose.getOrientation().getYaw(AngleUnit.RADIANS)));
            }
        }
    }

    public Pose relocalizeConvert() {
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D limelightPose = result.getBotpose_MT2();
            if (limelightPose != null) {
                //Limelight meters to pedro inches
                double xInches = limelightPose.getPosition().x * 39.3701 + 72;
                double yInches = limelightPose.getPosition().y * 39.3701 + 144 + 39.3701;

                //Limelight degrees to pedro radians
                //double yawRadians = Math.toRadians(limelightPose.getOrientation().getYaw(Angle));

//                pose pedroPose = new pose(xInches, yInches, limelightPose.getOrientation().getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE)
//                        .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                return new Pose(xInches, yInches, limelightPose.getOrientation().getYaw(AngleUnit.RADIANS));
            }
            return new Pose(0,0,0);
        }
        return new Pose(0,0,0);
    }

    public String relocalizeTest() {
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D limelightPose = result.getBotpose_MT2();
            if (limelightPose != null) {
                return limelightPose.toString();
            }
        }
        return "didn't get a pose";
    }

    public void setAlliance(Alliance alliance) {
        switch (alliance) {
            case BLUE:
                goalTarget = new Pose(4, 140);
                endgamePark = () -> follower.pathBuilder()
                        .addPath(new Path(new BezierLine(follower::getPose, new Pose(112, 29))))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.6))
                        .build();
                teleOpHeadingOffset = Math.toRadians(0);
                break;
            case RED:
                goalTarget = new Pose(140, 140);
                endgamePark = () -> follower.pathBuilder()
                        .addPath(new Path(new BezierLine(follower::getPose, new Pose(32, 29))))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.6))
                        .build();
                teleOpHeadingOffset = Math.toRadians(180);
                break;
        }
        passthrough.alliance = alliance;
    }

    private Vector robotToGoalVector(Pose currentPose) {
        double dx = goalTarget.getX() - currentPose.getX();
        double dy = goalTarget.getY() - currentPose.getY();
        return new Vector(new Pose(dx, dy));
    }

    private static double getFlywheelTicksFromVelocity(double velocity) {
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
}