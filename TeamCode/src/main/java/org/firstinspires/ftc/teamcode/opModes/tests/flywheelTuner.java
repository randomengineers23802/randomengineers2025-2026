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
import org.firstinspires.ftc.teamcode.control.robotControl;

@Configurable
@TeleOp(name = "flywheelTuner", group = "TeleOp")
public class flywheelTuner extends OpMode {
    private Follower follower;
    private boolean automatedDrive;
    private TelemetryManager panelsTelemetry;
    private boolean shooting = false;
    private Pose currentPose;
    private robotControl robot;
    private ElapsedTime timer = new ElapsedTime();
    private boolean prevRightTrigger = false;
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
        follower.setStartingPose(startPose);
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
        panelsTelemetry.addData("Calculated Target Velocity In/Sec", robot.flywheelVelocity);
        panelsTelemetry.addData("Calculated hood angle", robot.hoodAngleViewTest);
        panelsTelemetry.addData("Hood Servo Position", robot.hood.getPosition());
        panelsTelemetry.addData("Motor Ticks 1", robot.Shooter1.getVelocity());
        panelsTelemetry.addData("Motor Ticks 2", robot.Shooter2.getVelocity());
        panelsTelemetry.addData("Distance to goal", robot.robotToGoalVector(currentPose).getMagnitude());
        panelsTelemetry.addData("stopper position", robot.stopper.getPosition());
        panelsTelemetry.update();

        robot.Shooter1.setVelocity(flyWheelTicks);
        robot.Shooter2.setVelocity(flyWheelTicks);
        robot.hood.setPosition(hoodPosition);
    }
}