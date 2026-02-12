package org.firstinspires.ftc.teamcode.pedroPathing.auto.red;

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

import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.passthrough;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.robotControl;

@Autonomous(name = "farRedFriendly", group = "Autonomous")
@Configurable
public class farRedFriendly extends OpMode {

    private robotControl robot;
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime timer = new ElapsedTime();
    private boolean wallWait = false;
    private boolean pathStarted = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(87.125, 8.5625, Math.toRadians(270)));
        paths = new Paths(follower);
        robot = new robotControl(hardwareMap, follower);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        robot.intakeOn();
        robot.setShooterVelocity("far");
        robot.beltOn();
        robot.blueBoiClosed();
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();
    }

    private void Shoot() {
        if (follower.isBusy())
            timer.reset();
        else {
            double t = timer.seconds();
            if (t <= 1.0) {
                robot.blueBoiOpen();
            }
            else {
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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.125, 8.563),
                                    new Pose(86.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(338))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.000, 15.000),
                                    new Pose(89.354, 40.691),
                                    new Pose(134.000, 34.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.000, 34.000),
                                    new Pose(86.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(82), Math.toRadians(338))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 15.000),
                                    new Pose(133.000, 20.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(338), Math.toRadians(40))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.000, 20.000),
                                    new Pose(136.000, 8.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.000, 8.000),
                                    new Pose(86.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(338))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 15.000),
                                    new Pose(133.000, 20.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(338), Math.toRadians(40))
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.000, 20.000),
                                    new Pose(136.000, 8.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.000, 8.000),
                                    new Pose(86.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(338))
                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 15.000),
                                    new Pose(108.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(338), Math.toRadians(90))
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
                Shoot();
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    pathState++;
                }
                break;
            case 6:
                if (!wallWait) {
                    wallWait = true;
                    timer.reset();
                }
                if (timer.seconds() > 3.0) {
                    follower.followPath(paths.Path5, true);
                    wallWait = false;
                    pathState++;
                }
                break;
            case 7:
                if (!wallWait) {
                    wallWait = true;
                    timer.reset();
                }
                if (timer.seconds() > 2.0) {
                    follower.followPath(paths.Path6, true);
                    wallWait = false;
                    pathState++;
                }
                break;
            case 8:
                Shoot();
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    pathState++;
                }
                break;
            case 10:
                if (!wallWait) {
                    wallWait = true;
                    timer.reset();
                }
                if (timer.seconds() > 3.0) {
                    follower.followPath(paths.Path8, true);
                    wallWait = false;
                    pathState++;
                }
                break;
            case 11:
                if (!wallWait) {
                    wallWait = true;
                    timer.reset();
                }
                if (timer.seconds() > 2.0) {
                    follower.followPath(paths.Path9, true);
                    wallWait = false;
                    pathState++;
                }
                break;
            case 12:
                Shoot();
                break;
            case 13:
                if (!pathStarted) {
                    follower.followPath(paths.Path10, true);
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