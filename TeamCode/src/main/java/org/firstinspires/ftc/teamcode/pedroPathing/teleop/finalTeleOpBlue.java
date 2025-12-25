package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.passthrough;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.shooterControl;

@Configurable
@TeleOp(name = "finalTeleOpBlue", group = "TeleOp")
public class finalTeleOpBlue extends OpMode {
    private Follower follower;
    private boolean automatedDrive;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;
    private boolean shooting = false;
    private Pose currentPose;
    private shooterControl shooter;
    private DcMotor intake = null;
    private DcMotor belt = null;
    private Servo BlueBoi = null;
    private ElapsedTime timer = new ElapsedTime();
    boolean rightTriggerPressed;
    boolean rightTriggerWasPressed;
    boolean prevRightTrigger;

    @Override
    public void init() {
        shooter = new shooterControl(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        belt = hardwareMap.get(DcMotor.class, "belt");
        belt.setDirection(DcMotor.Direction.REVERSE);
        BlueBoi = hardwareMap.get(Servo.class, "BlueBoi");
        BlueBoi.setPosition(0.65);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(passthrough.startPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    private void Shoot() {
        belt.setPower(1.0);
        if (currentPose != null) {
            follower.holdPoint(currentPose);
        }
        double t = timer.seconds();
        if (t <= 1.0)
            BlueBoi.setPosition(1.0);
        else {
            BlueBoi.setPosition(0.65);
            belt.setPower(0.0);
            shooting = false;
            automatedDrive = false;
        }
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        intake.setPower(0.0);
        shooter.setShooterVelocity(1060);
        belt.setPower(0.0);
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

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
            shooter.setShooterVelocity(1300);
        }

        if (gamepad1.dpad_right) {
            shooter.setShooterVelocity(1060);
        }

        if (gamepad1.right_bumper) {
            intake.setPower(0.75);
            belt.setPower(1.0);
        }
        else if (!shooting) {
            intake.setPower(0.0);
            belt.setPower(0.0);
        }

        rightTriggerPressed = gamepad1.right_trigger > 0.2;
        rightTriggerWasPressed = rightTriggerPressed && !prevRightTrigger;

        if (rightTriggerWasPressed && !shooting) {
            timer.reset();
            currentPose = follower.getPose();
            automatedDrive = true;
            shooting = true;
        }

        prevRightTrigger = rightTriggerPressed;

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

        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("busy", follower.isBusy());
        telemetryM.debug("BlueBoi", BlueBoi.getPosition());
        telemetryM.debug("ShooterLvelocity", shooter.ShooterL.getVelocity());
        telemetryM.debug("ShooterRvelocity", shooter.ShooterR.getVelocity());
        telemetryM.debug("shooting", shooting);
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}