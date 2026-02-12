package org.firstinspires.ftc.teamcode.pedroPathing.auto.red;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.passthrough;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.robotControl;
import com.pedropathing.geometry.BezierCurve;

@Autonomous(name = "closeRed", group = "Autonomous")
@Configurable
@Disabled
public class closeRed extends OpMode {

    private robotControl robot;
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime timer = new ElapsedTime();
    private boolean pathStarted = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(119.5, 128.0, Math.toRadians(216.5)));
        paths = new Paths(follower);
        robot = new robotControl(hardwareMap, follower);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        robot.intakeOn();
        robot.setShooterVelocity("close");
        robot.beltOn();
        robot.blueBoiClosed();
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Current Path", pathState);
        panelsTelemetry.update(telemetry);
    }

    private void Shoot() {
        if (follower.isBusy())
            timer.reset();
        else {
            double t = timer.seconds();
            if (t <= 1.0) {
                robot.blueBoiOpen();
            } else {
                robot.blueBoiClosed();
                pathState++;
            }
        }
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(119.500, 128.000), new Pose(86.000, 80.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(216.5), Math.toRadians(225.0))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.000, 80.000), new Pose(95.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225.0), Math.toRadians(0.0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.000, 84.000), new Pose(128.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(128.000, 84.000),
                                    new Pose(107.000, 79.500),
                                    new Pose(128.000, 75.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(90.0))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(128.000, 75.000), new Pose(86.000, 80.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(225.0))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.000, 80.000), new Pose(95.000, 58.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225.0), Math.toRadians(0.0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.000, 58.000), new Pose(134.000, 58.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(134.000, 58.000), new Pose(86.000, 80.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(225.0))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.000, 80.000), new Pose(95.000, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225.0), Math.toRadians(0.0))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.000, 35.500), new Pose(134.000, 35.500))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(134.000, 35.500), new Pose(86.000, 80.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(225.0))
                    .build();

            Path12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.000, 80.000), new Pose(119.000, 70.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225.0), Math.toRadians(270.0))
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
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState++;
                }
                break;

            case 6:
                Shoot();
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    pathState++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    pathState++;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    pathState++;
                }
                break;

            case 10:
                Shoot();
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    pathState++;
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, true);
                    pathState++;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11, true);
                    pathState++;
                }
                break;

            case 14:
                Shoot();
                break;

            case 15:
                if (!pathStarted) {
                    follower.followPath(paths.Path12, true);
                    pathStarted = true;
                }
                if (!follower.isBusy())
                    pathState++;
                break;

            default:
                robot.shooterStop();
                robot.intakeOff();
                robot.beltOff();
                follower.breakFollowing();
                panelsTelemetry.debug("Status", "Autonomous Complete");
                panelsTelemetry.update(telemetry);
                break;
        }
        return pathState;
    }

    @Override
    public void stop() {
        passthrough.startPose = follower.getPose();
    }
}