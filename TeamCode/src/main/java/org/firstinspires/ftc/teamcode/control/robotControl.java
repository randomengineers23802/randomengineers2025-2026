package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class robotControl {

    private Gamepad gamepad1;
    public DcMotorEx Shooter1;
    public DcMotorEx Shooter2;
    public Limelight3A limelight;
    private DcMotor intake;
    private DcMotor turret;
    private Servo BlueBoi;
    private Follower follower;
    private Pose targetPose = new Pose(0, 0);
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime turretTimer = new ElapsedTime();
    private double lastError = 0;
    private double lastEncoderAngle = 0;
    private AnalogInput analogEncoder;
    private int rotationCounter;
    private double gearRatio = 20.0 / 50.0;
    private static final double flywheelOffset = 0; //if shooting consistently to far/short
    private static final double flywheelMinSpeed = 1000;
    private static final double flywheelMaxSpeed = 1080;
    private static final double scoreHeight = 26; //distance from flywheel to goal
    private static final double scoreAngle = Math.toRadians(-30);
    private static final double passthroughPointRadius = 5;//radius that robot will aim, based of of targetPose
    private static final double hoodMinAngle = Math.toRadians(42); //ball exit angle
    private static final double hoodMaxAngle = Math.toRadians(72); //ball exit angle
    //will have to change what angles actually are, higher ball exit angle means smalled hood angle

    PIDFCoefficients shooterPIDF = new PIDFCoefficients(60.0, 0.0, 0.0, 11.875);
    //PIDFCoefficients aimPIDF = new PIDFCoefficients(1.2, 0.0, 0.1, 0.02);
    PIDFCoefficients aimTurretPIDF = new PIDFCoefficients(1.2, 0.0, 0.1, 0.02);


    public robotControl(HardwareMap hardwareMap, Follower follower, Gamepad gamepad1) {
        this.follower = follower;
        this.gamepad1 = gamepad1;
        Shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        Shooter1.setDirection(DcMotorEx.Direction.FORWARD);
        Shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Shooter1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterPIDF);
        Shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        Shooter2.setDirection(DcMotorEx.Direction.FORWARD);
        Shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Shooter2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterPIDF);
        intake = hardwareMap.get(DcMotor.class, "intake");
        turret = hardwareMap.get(DcMotor.class, "belt");
        turret.setDirection(DcMotor.Direction.FORWARD);
        BlueBoi = hardwareMap.get(Servo.class, "BlueBoi");
        BlueBoi.setPosition(0.65);
        analogEncoder = hardwareMap.get(AnalogInput.class, "analogEncoder");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        timer.reset();
    }

//    public double autoAim() {
//        Pose currentPose = follower.getPose();
//        double distanceX = targetGoalX - currentPose.getX();
//        double distanceY = targetGoalY - currentPose.getY();
//        double angleToGoal = Math.atan2(distanceY, distanceX) + Math.PI;
//
//        double error = angleToGoal - currentPose.getHeading();
//        while (error > Math.PI) error -= 2 * Math.PI;
//        while (error < -Math.PI) error += 2 * Math.PI;
//        double dt = timer.seconds();
//        timer.reset();
//
//        double derivative = 0;
//        if (dt > 0.001) {
//            derivative = (error - lastError) / dt;
//            lastError = error;
//        }
//        double feedForward = Math.signum(error) * aimPIDF.f;
//        double aimPower = (error * aimPIDF.p) + (derivative * aimPIDF.d) + feedForward;
//
//        if (Math.abs(error) < Math.toRadians(1.0)) //won't move if robot is within 1 degree
//            return 0;
//        else {
//            return Range.clip(aimPower, -1.0, 1.0);
//        }
//    }

        public void aimTurret() {
        Pose currentPose = follower.getPose();
        //double distanceX = targetPose.getX() - currentPose.getX();
        //double distanceY = targetPose.getY() - currentPose.getY();
        //double angleToGoal = Math.atan2(distanceY, distanceX);
        double angleToGoal = robotToGoalVector(currentPose).getTheta();
        double turretLocalTarget = angleToGoal - currentPose.getHeading();
        double currentTurretAngle = analogEncoder.getVoltage() / 3.3 * 360;
        double error = turretLocalTarget - currentTurretAngle;
        double dt = turretTimer.seconds();
        turretTimer.reset();
        double derivative = 0;
        if (dt > 0.001) {
            derivative = (error - lastError) / dt;
            lastError = error;
        }
        double feedforward = Math.signum(error) * aimTurretPIDF.f;
        double turretPower = (error * aimTurretPIDF.p) + (derivative * aimTurretPIDF.d) + feedforward;

        if (Math.abs(error) < Math.toRadians(1.0)) //won't move if turret is within 1 degree
            turret.setPower(0);
        else {
            turretPower = Range.clip(turretPower, -1.0, 1.0);
            turret.setPower(turretPower);
        }
    }

    public void aimTurretTest() {
        Pose currentPose = follower.getPose();
        double distanceX = targetPose.getX() - currentPose.getX();
        double distanceY = targetPose.getY() - currentPose.getY();
        double angleToGoal = Math.atan2(distanceY, distanceX);
        double turretLocalTarget = angleToGoal - currentPose.getHeading();
        double currentEncoderAngle = analogEncoder.getVoltage() / 3.3 * 360;
        if (lastEncoderAngle - currentEncoderAngle > 270) {
            rotationCounter += 1;
        }
        else if (lastEncoderAngle - currentEncoderAngle < -270) {
            rotationCounter -= 1;
        }
        double currentTurretAngle = ((rotationCounter * 360) + currentEncoderAngle) * gearRatio;


        lastEncoderAngle = currentEncoderAngle;
        double error = turretLocalTarget - currentTurretAngle;
        double dt = turretTimer.seconds();
        turretTimer.reset();
        double derivative = 0;
        if (dt > 0.001) {
            derivative = (error - lastError) / dt;
            lastError = error;
        }
        double feedforward = Math.signum(error) * aimTurretPIDF.f;
        double turretPower = (error * aimTurretPIDF.p) + (derivative * aimTurretPIDF.d) + feedforward;

        if (Math.abs(error) < Math.toRadians(1.0)) //won't move if turret is within 1 degree
            turret.setPower(0);
        else {
            turretPower = Range.clip(turretPower, -1.0, 1.0);
            turret.setPower(turretPower);
        }
    }

    public void moveTurret() {
        ShotParameters aim = calculateShotVectorAndUpdateTurret(follower.getPose());
        double currentTurretAngle = analogEncoder.getVoltage() / 3.3 * 360;
        double error = aim.turretAngle - currentTurretAngle;
        double dt = turretTimer.seconds();
        turretTimer.reset();
        double derivative = 0;
        if (dt > 0.001) {
            derivative = (error - lastError) / dt;
            lastError = error;
        }
        double feedforward = Math.signum(error) * aimTurretPIDF.f;
        double turretPower = (error * aimTurretPIDF.p) + (derivative * aimTurretPIDF.d) + feedforward;

        if (Math.abs(error) < Math.toRadians(1.0)) //won't move if turret is within 1 degree
            turret.setPower(0);
        else {
            turretPower = Range.clip(turretPower, -1.0, 1.0);
            turret.setPower(turretPower);
        }
    }


//    public void relocalize() {
//        LLResult result = limelight.getLatestResult();
//        if (result != null && result.isValid()) {
//            Pose3D botpose = result.getBotpose();
//            Pose2D ftcPose2d = new Pose2D(
//                    DistanceUnit.INCH, botpose.getPosition().x, botpose.getPosition().y,
//                    AngleUnit.DEGREES, botpose.getOrientation().getYaw()
//            );
//            Pose ftcStandard = PoseConverter.pose2DToPose(ftcPose2d, InvertedFTCCoordinates.INSTANCE);
//            Pose pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
//            follower.setPose(pedroPose);
//        }
//    }

    public void relocalize() {
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
            gamepad1.rumble(1, 1, 500);
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
        Shooter1.setVelocity(targetVelocity);
        Shooter2.setVelocity(targetVelocity);
    }

    public void shooterStop() {
        Shooter1.setPower(0);
        Shooter2.setPower(0);
    }

    public void setAlliance(String goalColor) {
        switch (goalColor) {
            case "blue":
                //targetPose = new Pose(4, 135);
                targetPose = new Pose(6, 138);
                break;
            case "red":
                //targetPose = new Pose(140, 135);
                targetPose = new Pose(138, 138);
                break;
        }
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

    private Vector robotToGoalVector(Pose currentPose) {
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        return new Vector(dx, dy);
    }

    public static double getFlywheelTicksFromVelocity(double velocity) {
        return Range.clip(94.501 * velocity / 12 - 187.96 + flywheelOffset, flywheelMinSpeed, flywheelMaxSpeed);
    }

    private static double getHoodTicksFromDegrees(double degrees) {
        return 0.0226 * degrees - 0.7443;
    }

    private ShotParameters calculateShotVectorAndUpdateTurret(Pose currentPose) {
        //constants
        Vector robotToGoalVector = robotToGoalVector(currentPose);
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - passthroughPointRadius;
        double y = scoreHeight;
        double a = scoreAngle;

        //calculate initial launch components
        double hoodAngle = Range.clip(Math.atan(2 * y / x - Math.tan(a)), hoodMaxAngle, hoodMinAngle);
        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        //get robot velocity and convert it into parallel and perpendicular components
        Vector robotVelocity = follower.getVelocity();
        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();
        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        //velocity compensation variables
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        //recalculate launch components
        hoodAngle = Range.clip(Math.atan(vz / nvr), hoodMaxAngle, hoodMinAngle);
        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

        //update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double turretAngle = Math.toDegrees(currentPose.getHeading() - robotToGoalVector.getTheta() + turretVelCompOffset);

        //return data
        return new ShotParameters(flywheelSpeed, hoodAngle, turretAngle);
    }
}