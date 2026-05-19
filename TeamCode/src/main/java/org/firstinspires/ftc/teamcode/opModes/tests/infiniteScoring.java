package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.control.ShotParameters;
import org.firstinspires.ftc.teamcode.control.passthrough;
import org.firstinspires.ftc.teamcode.control.RobotControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "infiniteScoring", group = "Autonomous")
@Configurable
public class infiniteScoring extends OpMode {
    private RobotControl robot;
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime timer = new ElapsedTime();
    private boolean gateWait = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(24.500, 128.000, Math.toRadians(323.5)));
        paths = new Paths(follower);
        robot = new RobotControl(hardwareMap, follower);
        robot.setAlliance(Alliance.BLUE);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        robot.intake.on();
        robot.belt.onShoot();
        robot.blueBoi.close();
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();
        ShotParameters shotParameters = robot.updateShooting();
        robot.shooter.setVelocity(shotParameters.flywheelTicks + 10);
    }

    private void Shoot() {
        if (follower.isBusy())
            timer.reset();
        else {
            double t = timer.seconds();
            if (t <= 1.0) {
                robot.blueBoi.open();
            }
            else {
                robot.blueBoi.close();
                pathState = 2;
            }
        }
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.500, 128.000),
                                    new Pose(58.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(323.5), Math.toRadians(314))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.000, 80.000),
                                    new Pose(60.000, 69.000),
                                    new Pose(4.500, 56.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(314), Math.toRadians(155), 0.15)
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(4.500, 56.500),
                                    new Pose(40.000, 62.000),
                                    new Pose(58.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(314))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1, true);
                pathState++;
                break;
            case 1:
                Shoot();
                break;

            case 2:
                follower.followPath(paths.Path2, true);
                pathState++;
                break;
            case 3:
                if (!gateWait) {
                    gateWait = true;
                    timer.reset();
                }
                if (timer.seconds() > 4.0) {
                    follower.followPath(paths.Path3, true);
                    gateWait = false;
                    pathState++;
                }
                break;
            case 4:
                Shoot();
                break;
        }
        return pathState;
    }

    @Override
    public void stop() {
        passthrough.pose = follower.getPose();
    }
}