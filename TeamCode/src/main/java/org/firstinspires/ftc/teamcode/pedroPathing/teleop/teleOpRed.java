package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.passthrough;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.robotControl;

@Configurable
@TeleOp(name = "teleOpRed", group = "TeleOp")
public class teleOpRed extends OpMode {
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

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        robot = new robotControl(hardwareMap, follower);
        robot.setAlliance("red");
        follower.setStartingPose(passthrough.startPose);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    private void Shoot() {
        robot.beltOn();
        robot.intakeOn();
        follower.holdPoint(currentPose);
        double t = timer.seconds();
        if (t <= 1.0)
            robot.blueBoiOpen();
        else {
            robot.blueBoiClosed();
            robot.beltOff();
            shooting = false;
            automatedDrive = false;
            follower.startTeleopDrive();
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
        panelsTelemetry.update();

        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        if (gamepad1.left_trigger > 0.2) {
            if (!automatedDrive) {
                if (!slowMode) {
                    follower.setTeleOpDrive(y, x, robot.autoAim(), false);
                } else {
                    follower.setTeleOpDrive(
                            y * slowModeMultiplier,
                            x * slowModeMultiplier,
                            robot.autoAim(),
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

        if (gamepad1.dpad_left) {
            robot.setShooterVelocity("far");
        }

        if (gamepad1.dpad_right) {
            robot.setShooterVelocity("close");
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
            shooting = true;
            automatedDrive = true;
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