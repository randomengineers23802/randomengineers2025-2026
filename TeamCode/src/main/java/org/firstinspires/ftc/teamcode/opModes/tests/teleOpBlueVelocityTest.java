package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.ShotParameters;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.control.passthrough;
import org.firstinspires.ftc.teamcode.control.robotControl;

@Configurable
@TeleOp(name = "teleOpBlueVelocityTest", group = "TeleOp")
public class teleOpBlueVelocityTest extends OpMode {
    private Follower follower;
    private boolean automatedDrive;
    private TelemetryManager panelsTelemetry;
    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.25;
    private boolean shooting = false;
    private Pose currentPose;
    private robotControl robot;
    private ElapsedTime timer = new ElapsedTime();
    private boolean prevRightTrigger = false;
    public double customTicks = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        robot = new robotControl(hardwareMap, follower);
        robot.setAlliance("blue");
        follower.setStartingPose(passthrough.startPose);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    private void Shoot() {
        robot.beltOn();
        robot.intakeOn();
        //follower.holdPoint(currentPose);
        double t = timer.seconds();
        if (t <= 1.0)
            robot.blueBoiOpen();
        else {
            robot.blueBoiClosed();
            robot.beltOff();
            shooting = false;
            automatedDrive = false;
            //follower.startTeleopDrive();
        }
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        robot.intakeOff();
        robot.setShooterVelocity("close");
        robot.beltOff();
    }

    @Override
    public void loop() {
        follower.update();
        ShotParameters shotParameters = robot.updateShooting();
//        if (gamepad1.dpadUpWasPressed())
//            customTicks += 20;
//        else if (gamepad1.dpadDownWasPressed())
//            customTicks -= 20;
        //robot.setShooterVelocity(customTicks);
        robot.setShooterVelocity(shotParameters.flywheelTicks);

        panelsTelemetry.addData("distance from goal", robot.robotToGoalVector(follower.getPose()).getMagnitude());
        panelsTelemetry.addData("Target ticks", shotParameters.flywheelTicks);
        panelsTelemetry.addData("ShooterL ticks", robot.ShooterL.getVelocity());
        panelsTelemetry.addData("ShooterR ticks", robot.ShooterR.getVelocity());
        panelsTelemetry.addData("calculated inches per second", robot.flywheelInchesPerSec);

        panelsTelemetry.update();

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        if (gamepad1.left_trigger > 0.2) {
            if (!automatedDrive) {
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
        }
        else {
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
        }

        if (gamepad1.right_bumper) {
            robot.intakeOn();
            robot.beltOn();
        }
        else if (!shooting) {
            robot.intakeOff();
            robot.beltOff();
        }

        boolean rightTriggerPressed = gamepad1.right_trigger > 0.2;
        boolean rightTriggerWasPressed = rightTriggerPressed && !prevRightTrigger;

        if (rightTriggerWasPressed && !shooting) {
            timer.reset();
            currentPose = follower.getPose();
            automatedDrive = true;
            shooting = true;
        }

        prevRightTrigger = rightTriggerPressed;

        if (shooting)
            Shoot();

        if (automatedDrive && (gamepad1.bWasPressed() || Math.abs(gamepad1.left_stick_x) > 0.4 || Math.abs(gamepad1.left_stick_y) > 0.4 || Math.abs(gamepad1.right_stick_x) > 0.4)) {
            follower.startTeleopDrive();
            automatedDrive = false;
            shooting = false;
            robot.blueBoiClosed();
        }

        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }
    }

    @Override
    public void stop() {
        passthrough.startPose = follower.getPose();
    }
}