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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.datalogging.Datalog;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.passthrough;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "FinalTeleOpRed", group = "TeleOp")
public class FinalTeleOpRed extends OpMode {
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
    boolean downPressed = false;
    boolean downPreviouslyPressed;
    private boolean prevRightTrigger = false;

    Datalog datalog;
    int loopCounter = 0;
    VoltageSensor battery;

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
        battery = hardwareMap.voltageSensor.get("Control Hub");
        BlueBoi = hardwareMap.get(Servo.class, "BlueBoi");
        BlueBoi.setPosition(0.65);

        datalog = new Datalog("datalog_01");
        datalog.opModeStatus.set("RUNNING");
        datalog.battery.set(battery.getVoltage());
        datalog.writeLine();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(passthrough.startPose); //uses pose from auto
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        farShoot = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(84, 13))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(248.5), 0.8))
                .build();

        endgamePark = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(38.5, 33.5))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();

        closeShoot = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 81))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(233), 0.8))
                .build();
    }

    private void Shoot() {
        belt.setPower(0.8);
        if (follower.isBusy())
            timer.reset();
        else {
            double t = timer.seconds();
            if (t <= 2.5)
                BlueBoi.setPosition(1.0);
            else {
                BlueBoi.setPosition(0.65);
                belt.setPower(0.0);
                shooting = false;
            }
        }
    }

    private void autoAim() {
        Pose p = follower.getPose();
        double dx = 132 - p.getX();
        double dy = 137 - p.getY();
        double aimHeading = Math.PI + Math.atan2(dy, dx);
        follower.turnTo(aimHeading);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        intake.setPower(0.0);
        ShooterL.setVelocity(1040);
        ShooterR.setVelocity(1040);
        belt.setPower(0.0);
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
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
            ShooterL.setVelocity(1260);
            ShooterR.setVelocity(1260);
        }

        if (gamepad1.dpad_right) {
            ShooterL.setVelocity(1040);
            ShooterR.setVelocity(1040);
        }

//        if (gamepad1.xWasPressed() && !shooting && !automatedDrive) {
//            follower.followPath(closeShoot.get());
//            timer.reset();
//            ShooterL.setVelocity(1000);
//            ShooterR.setVelocity(1000);
//            shooting = true;
//            automatedDrive = true;
//        }
//
//        if (gamepad1.aWasPressed() && !shooting && !automatedDrive) {
//            follower.followPath(farShoot.get());
//            timer.reset();
//            ShooterL.setVelocity(1200);
//            ShooterR.setVelocity(1200);
//            shooting = true;
//            automatedDrive = true;
//        }
//
//        if (gamepad1.yWasPressed() && !shooting && !automatedDrive) {
//            follower.followPath(endgamePark.get());
//            automatedDrive = true;
//        }

        if (shooting)
            Shoot();

        if (automatedDrive && gamepad1.bWasPressed()) {
            follower.startTeleopDrive();
            automatedDrive = false;
            shooting = false;
            BlueBoi.setPosition(0.65);
        }

        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }

        if (gamepad1.right_bumper) {
            intake.setPower(0.75);
            belt.setPower(0.8);
        }
        else if (!shooting) {
            intake.setPower(0.0);
            belt.setPower(0.0);
        }

        if (gamepad1.left_trigger > 0.2 && !shooting && !automatedDrive) {
            autoAim();
            automatedDrive = true;
        }

        boolean rightTriggerPressed = gamepad1.right_trigger > 0.2;
        boolean rightTriggerWasPressed = rightTriggerPressed && !prevRightTrigger;
        prevRightTrigger = rightTriggerPressed;

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

        datalog.loopCounter.set(loopCounter);
        loopCounter++;
        datalog.battery.set(battery.getVoltage());
        datalog.shooterLvel.set(ShooterL.getVelocity());
        datalog.shooterRvel.set(ShooterR.getVelocity());

        telemetryM.debug("position",
                String.format("(%.2f, %.2f, %.2f)",
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("LinearServo", LinearServo.getPosition());
        telemetryM.debug("busy", follower.isBusy());
        telemetryM.debug("BlueBoi", BlueBoi.getPosition());
        telemetryM.debug("ShooterLvelocity", ShooterL.getVelocity());
        telemetryM.debug("ShooterRvelocity", ShooterR.getVelocity());
        telemetryM.debug("shooting", shooting);
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}
