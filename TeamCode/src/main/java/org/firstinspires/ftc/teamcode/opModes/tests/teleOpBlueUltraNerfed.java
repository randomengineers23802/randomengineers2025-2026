package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.control.ShotParameters;
import org.firstinspires.ftc.teamcode.control.passthrough;
import org.firstinspires.ftc.teamcode.control.RobotControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Disabled
@TeleOp(name = "teleOpBlueUltraNerfed", group = "TeleOp")
public class teleOpBlueUltraNerfed extends OpMode {
    private Follower follower;
    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.25;
    private boolean shooting = false;
    private RobotControl robot;
    private ElapsedTime timer = new ElapsedTime();
    private boolean prevRightTrigger = false;
    private double flywheelSpeed = 1000;
    //private Supplier<PathChain> endgameParkBlue;
    private boolean endgame;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        robot = new RobotControl(hardwareMap, follower);
        robot.setAlliance(Alliance.BLUE);
        follower.setStartingPose(passthrough.pose);
        robot.kickstand.raise();
    }

    private void Shoot() {
        robot.belt.onShoot();
        robot.intake.on();
        double t = timer.seconds();
        if (t <= 1.0)
            robot.blueBoi.open();
        else {
            robot.blueBoi.close();
            shooting = false;
        }
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        robot.intake.off();
        robot.belt.off();
        robot.light.setColor(1.0);
    }

    @Override
    public void loop() {
        follower.update();
        ShotParameters shotParameters = robot.updateShooting();
        //if (!endgame) { robot.shooter.setVelocity(shotParameters.flywheelTicks + 20); }
        robot.shooter.setVelocity(flywheelSpeed); //No calculated shooting speed
        if (gamepad2.dpadUpWasPressed()) {flywheelSpeed += 20;}
        else if (gamepad2.dpadDownWasPressed()) {flywheelSpeed -= 20;}
        Pose currentPose = follower.getPose();
        telemetry.addData("pose x",currentPose.getX());
        telemetry.addData("pose y",currentPose.getY());
        telemetry.addData("heading", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("limelight raw pose", robot.relocalizeTest());
        telemetry.addData("limelihgt raw converted to pedro", robot.relocalizeConvert());
        telemetry.addData("flywheel speed", flywheelSpeed);
        telemetry.update();

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        if (!slowMode) {
            follower.setTeleOpDrive(y * 0.5, x * 0.5, turn * 0.5, true);
        } else {
            follower.setTeleOpDrive(
                    y * slowModeMultiplier,
                    x * slowModeMultiplier,
                    turn * slowModeMultiplier,
                    true
            );
        }

        if (!shooting) {
            if (gamepad1.right_bumper) {
                robot.intake.on();
                robot.belt.onIntake();
            }
            else {
                robot.intake.off();
                robot.belt.off();
            }
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
            robot.blueBoi.close();
            follower.startTeleopDrive();
        }

        if (gamepad1.xWasPressed() && follower.getVelocity().getMagnitude() < 1.5) {
            robot.relocalize();
        }

        if (gamepad1.yWasPressed()) {
            follower.followPath(robot.endgamePark.get());
            robot.shooter.off();
            slowMode = true;
            endgame = true;
        }

        if (gamepad1.dpadDownWasPressed()) {
            robot.kickstand.lower();
            robot.light.setColor(0.444);
        }
        else if (gamepad1.dpadUpWasPressed()) {
            robot.kickstand.raise();
            robot.light.setColor(1.0);
        }

        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }
    }

    @Override
    public void stop() {
        passthrough.pose = follower.getPose();
    }
}