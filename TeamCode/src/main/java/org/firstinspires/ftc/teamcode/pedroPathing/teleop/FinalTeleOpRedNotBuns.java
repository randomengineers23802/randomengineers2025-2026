package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.passthrough;
import java.util.function.Supplier;

@Configurable
@Disabled
@TeleOp(name = "FinalTeleOpRedNotBuns", group = "TeleOp")
public class FinalTeleOpRedNotBuns extends OpMode {

    private Follower follower;
    private boolean automatedDrive;
    private Supplier<PathChain> farShoot;
    private Supplier<PathChain> endgamePark;
    private Supplier<PathChain> closeShoot;
    private TelemetryManager telemetryM;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;
    private boolean shooting = false;

    private DcMotorEx ShooterL = null;
    private DcMotorEx ShooterR = null;
    private DcMotor intake = null;
    private DcMotor belt = null;
    private Servo LinearServo = null;
    private Servo BlueBoi = null;

    private ElapsedTime timer = new ElapsedTime();

    private boolean downPressed = false;
    private boolean downPreviouslyPressed = false;

    private boolean rightTriggerPressed = false;
    private boolean rightTriggerWasPressed = false;
    private boolean prevRightTrigger = false;

    private boolean leftTriggerPressed = false;
    private Limelight3A limelight;
    private final double kP = 0.04;
    private final double kI = 0.0;
    private final double kD = 0.025;
    private double aimIntegral = 0;
    private double previousError = 0;

    @Override
    public void init() {
        ShooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        ShooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        belt = hardwareMap.get(DcMotor.class, "belt");

        ShooterL.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setDirection(DcMotor.Direction.REVERSE);

        ShooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ShooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        LinearServo = hardwareMap.get(Servo.class, "LinearServo");
        LinearServo.setPosition(0.1);

        BlueBoi = hardwareMap.get(Servo.class, "BlueBoi");
        BlueBoi.setPosition(0.65);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(passthrough.startPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        farShoot = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(84, 13))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(248.5), 0.8))
                .build();

        endgamePark = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(38.5, 33.5))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();

        closeShoot = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 81))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(233), 0.8))
                .build();
    }

    private void Shoot() {
        if (follower.isBusy()) {
            timer.reset();
        } else {
            double t = timer.seconds();
            if (t <= 0.5) {
                BlueBoi.setPosition(0.65);
            } else if (t <= 2.5) {
                BlueBoi.setPosition(1.0);
            } else {
                BlueBoi.setPosition(0.65);
                shooting = false;
            }
        }
    }

    private void autoAim() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            aimIntegral = 0;
            previousError = 0;
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false);
            return;
        }

        double error = result.getTx();
        double dt = timer.seconds();

        double derivative = 0.0;
        if (dt > 0) {
            derivative = (error - previousError) / dt;
        }

        if (Math.abs(error) < 1.0 && Math.abs(derivative) < 0.05) {
            aimIntegral = 0;
            previousError = 0;
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false);
            return;
        }

        aimIntegral += error * dt;

        double turnPower = kP * error + kI * aimIntegral + kD * derivative;
        turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

        previousError = error;
        timer.reset();

        follower.setTeleOpDrive(0.0, 0.0, -turnPower, false);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        intake.setPower(0.0);
        ShooterL.setVelocity(1000);
        ShooterR.setVelocity(1000);
        belt.setPower(0.5);
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        rightTriggerPressed = gamepad1.right_trigger > 0.2;
        rightTriggerWasPressed = rightTriggerPressed && !prevRightTrigger;

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        if (!automatedDrive) {
            if (!slowMode) {
                follower.setTeleOpDrive(y, x, turn, false);
            } else {
                follower.setTeleOpDrive(
                        y * slowModeMultiplier,
                        x * slowModeMultiplier,
                        turn * slowModeMultiplier,
                        false
                );
            }
        }

        if (gamepad1.dpad_left) {
            ShooterL.setVelocity(1200);
            ShooterR.setVelocity(1200);
        }
        if (gamepad1.dpad_up) {
            ShooterL.setVelocity(1000);
            ShooterR.setVelocity(1000);
        }
        if (gamepad1.dpad_right) {
            ShooterL.setVelocity(900);
            ShooterR.setVelocity(900);
        }

        if (gamepad1.xWasPressed() && !shooting && !automatedDrive) {
            follower.followPath(closeShoot.get());
            timer.reset();
            ShooterL.setVelocity(1000);
            ShooterR.setVelocity(1000);
            shooting = true;
            automatedDrive = true;
        }
        if (gamepad1.aWasPressed() && !shooting && !automatedDrive) {
            follower.followPath(farShoot.get());
            timer.reset();
            ShooterL.setVelocity(1200);
            ShooterR.setVelocity(1200);
            shooting = true;
            automatedDrive = true;
        }
        if (gamepad1.yWasPressed() && !shooting && !automatedDrive) {
            follower.followPath(endgamePark.get());
            automatedDrive = true;
        }

        if (shooting) Shoot();

        if (automatedDrive && gamepad1.bWasPressed()) {
            follower.startTeleopDrive();
            automatedDrive = false;
            shooting = false;
            BlueBoi.setPosition(0.65);
        }

        if (gamepad1.leftBumperWasPressed()) slowMode = !slowMode;

        intake.setPower(gamepad1.right_bumper ? 0.75 : 0.0);

        if (gamepad1.left_trigger > 0.2 && !shooting && !automatedDrive) autoAim();

        if (rightTriggerWasPressed && !shooting) {
            timer.reset();
            shooting = true;
        }

        downPressed = gamepad1.dpad_down;
        if (downPressed && !downPreviouslyPressed) {
            follower.setPose(new Pose(72, 81, Math.toRadians(233)));
            follower.update();
        }
        downPreviouslyPressed = downPressed;

        prevRightTrigger = rightTriggerPressed;

        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("LinearServo", LinearServo.getPosition());
        telemetryM.debug("busy", follower.isBusy());
        telemetryM.debug("BlueBoi", BlueBoi.getPosition());
        telemetryM.debug("ShooterLvelocity", ShooterL.getVelocity());
        telemetryM.debug("ShooterRvelocity", ShooterR.getVelocity());
        telemetryM.debug("shooting", shooting);
    }
}
