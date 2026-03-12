package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Supplier;
import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.control.ShotParameters;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.control.passthrough;
import org.firstinspires.ftc.teamcode.control.robotControl;

@Disabled
@Configurable
@TeleOp(name = "teleOpBlueTest", group = "TeleOp")
public class teleOpBlueTest extends OpMode {
    private Follower follower;
    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.25;
    private boolean shooting = false;
    private robotControl robot;
    private ElapsedTime timer = new ElapsedTime();
    private boolean prevRightTrigger = false;
    private double flywheelTicks = 1000;
    private double kickstand1Position = 0.65;
    private double kickstand2Position = 0.65;
    private TelemetryManager panelsTelemetry;
    private Supplier<PathChain> endgameParkBlue;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        robot = new robotControl(hardwareMap, follower);
        robot.setAlliance(Alliance.BLUE);
        follower.setStartingPose(passthrough.startPose);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        robot.kickstandUp();

        endgameParkBlue = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(105.5, 33.5))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();
    }

    private void Shoot() {
        robot.beltOnShoot();
        robot.intakeOn();
        double t = timer.seconds();
        if (t <= 1.2)
            robot.blueBoiOpen();
        else {
            robot.blueBoiClosed();
            shooting = false;
        }
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        robot.intakeOff();
        robot.beltOff();
    }

    @Override
    public void loop() {
        follower.update();
        ShotParameters shotParameters = robot.updateShooting();

        if (gamepad1.dpadUpWasPressed())
            flywheelTicks += 10;
        else if (gamepad1.dpadDownWasPressed())
            flywheelTicks -= 10;

        robot.setShooterVelocity(flywheelTicks);

//        if (gamepad1.dpadLeftWasPressed()) {
//            robot.kickstandUp();
//            robot.setLightColor(1.0);
//        }
//        else if (gamepad1.dpadRightWasPressed()) {
//            robot.kickstandDown();
//            robot.setLightColor(0.52);
//        }

//        if (gamepad1.dpadUpWasPressed())
//            kickstand1Position += 0.01;
//        else if (gamepad1.dpadDownWasPressed())
//            kickstand1Position -= 0.01;
//
//        if (gamepad1.dpadRightWasPressed())
//            kickstand2Position += 0.01;
//        else if (gamepad1.dpadLeftWasPressed())
//            kickstand2Position -= 0.01;
//
//        robot.kickstand1.setPosition(kickstand1Position);
//        robot.kickstand2.setPosition(kickstand2Position);

        telemetry.addData("target ticks", flywheelTicks);
        telemetry.addData("ShooterL ticks", robot.ShooterL.getVelocity());
        telemetry.addData("ShooterR ticks", robot.ShooterR.getVelocity());
        telemetry.addData("flywheel inches/sec", robot.flywheelInchesPerSec);
        telemetry.addData("kickstand1 position", kickstand1Position);
        telemetry.addData("kickstand2 position", kickstand2Position);
        telemetry.addData("belt ticks", robot.belt.getVelocity());
        telemetry.update();

        panelsTelemetry.addData("ShooterL ticks", robot.ShooterL.getVelocity());
        panelsTelemetry.addData("ShooterR ticks", robot.ShooterR.getVelocity());
        panelsTelemetry.addData("flywheel inches/sec", robot.flywheelInchesPerSec);
        panelsTelemetry.update();

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        if (gamepad1.left_trigger > 0.2) {
            if (!slowMode) {
                follower.setTeleOpDrive(y, x, shotParameters.heading, false);
            } else {
                follower.setTeleOpDrive(
                        y * slowModeMultiplier,
                        x * slowModeMultiplier,
                        shotParameters.heading,
                        false
                );
            }
        }
        else {
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

        if (gamepad1.right_bumper && !shooting) {
            robot.intakeOn();
            robot.beltOnIntake();
        }
        else if (!shooting) {
            robot.intakeOff();
            robot.beltOff();
        }

        boolean rightTriggerPressed = gamepad1.right_trigger > 0.2;
        boolean rightTriggerWasPressed = rightTriggerPressed && !prevRightTrigger;

        if (rightTriggerWasPressed && !shooting) {
            timer.reset();
            shooting = true;
        }

        prevRightTrigger = rightTriggerPressed;

        if (shooting)
            Shoot();

        if (gamepad1.bWasPressed()) {
            shooting = false;
            robot.blueBoiClosed();
        }

        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }

        if (gamepad1.aWasPressed()) {
            follower.followPath(endgameParkBlue.get());
        }
    }

    @Override
    public void stop() {
        passthrough.startPose = follower.getPose();
    }
}