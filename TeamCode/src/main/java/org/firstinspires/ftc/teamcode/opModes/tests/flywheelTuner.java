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
import org.firstinspires.ftc.teamcode.control.passthrough;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.control.robotControl;

@Configurable
@TeleOp(name = "flywheelTuner", group = "TeleOp")
public class flywheelTuner extends OpMode {
    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private Pose currentPose;
    private robotControl robot;
    private final Pose startPose = new Pose(8.90625, 8.5625, Math.toRadians(90));
    public static double flyWheelTicks = 0;
    public static double hoodPosition = 0;
    public static double stopperPosition = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        robot = new robotControl(hardwareMap, follower, gamepad1);
        robot.setAlliance("blue");
        follower.setStartingPose(passthrough.startPose);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        robot.intakeOff();
    }

    @Override
    public void loop() {
        follower.update();
        currentPose = follower.getPose();
        ShotParameters shotParameters = robot.calculateShotVectorAndTurret(currentPose);

        //robot.updateTurretForTesting();
        telemetry.addData("Calculated Target Velocity In/Sec (was NaN)" , robot.flywheelVelocity);
        telemetry.addData("Unclamped hood angle", robot.hoodAngleViewTest);
        telemetry.addData("clamped hood angle", robot.clampedHoodAngleViewTest);
        telemetry.addData("Hood Servo Position", robot.hood.getPosition());
        telemetry.addData("Shooter Ticks 1", robot.Shooter1.getVelocity());
        telemetry.addData("Shooter Ticks 2", robot.Shooter2.getVelocity());
        panelsTelemetry.addData("Shooter Ticks 1", robot.Shooter1.getVelocity());
        panelsTelemetry.addData("Shooter Ticks 2", robot.Shooter2.getVelocity());
        telemetry.addData("Target flywheel ticks", flyWheelTicks);
        telemetry.addData("Distance to goal", robot.robotToGoalVector(currentPose).getMagnitude() - 3);
        telemetry.addData("stopper position", robot.stopper.getPosition());
        telemetry.addData("target turret angle (was NaN)", shotParameters.turretAngle);
        telemetry.addData("analog position", robot.analogEncoder.getVoltage() / 3.3 * 360);
        telemetry.update();
        panelsTelemetry.update();

        robot.hood.setPosition(shotParameters.hoodPosition);

        if (gamepad1.right_bumper)
            robot.intakeOn();
        else
            robot.intakeOff();

        if (gamepad1.dpadUpWasPressed()) {
            flyWheelTicks += 20;
        }
        else if (gamepad1.dpadDownWasPressed()) {
            flyWheelTicks -= 20;
        }

        if (gamepad1.aWasPressed()) {
            robot.stopperOpen();
        }
        if (gamepad1.bWasPressed()) {
            robot.stopperClosed();
        }

        robot.Shooter1.setVelocity(flyWheelTicks);
        robot.Shooter2.setVelocity(flyWheelTicks);
        //robot.hood.setPosition(hoodPosition);
        //robot.stopper.setPosition(stopperPosition);
    }

    @Override
    public void stop() {
        passthrough.startPose = follower.getPose();
    }
}