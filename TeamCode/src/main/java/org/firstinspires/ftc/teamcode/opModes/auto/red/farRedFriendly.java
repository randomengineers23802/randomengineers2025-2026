package org.firstinspires.ftc.teamcode.opModes.auto.red;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.control.passthrough;
import org.firstinspires.ftc.teamcode.control.robotControl;

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
        follower.setStartingPose(new Pose(87.125, 8.5625, Math.toRadians(180)));
        paths = new Paths(follower);
        robot = new robotControl(hardwareMap, follower);
        robot.setAlliance(Alliance.RED);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        robot.intakeOn();
        robot.beltOnShoot();
        robot.blueBoiClosed();
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();
        ShotParameters shotParameters = robot.updateShooting();
        robot.setShooterVelocity(shotParameters.flywheelTicks);
        telemetry.addData("heading", Math.toDegrees(follower.getHeading()));
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
        public PathChain Path11;
        public PathChain Path12;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.125, 8.563),
                                    new Pose(85.000, 22.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(253))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(85.000, 22.000),
                                    new Pose(89.354, 40.691),
                                    new Pose(133.000, 34.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.000, 34.000),
                                    new Pose(85.000, 22.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-8), Math.toRadians(250))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.000, 22.000),
                                    new Pose(128.000, 27.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.000, 27.000),
                                    new Pose(133.000, 8.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-90))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.000, 8.500),
                                    new Pose(133.000, 27.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-90))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.000, 27.000),
                                    new Pose(133.000, 8.500)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.000, 8.500),
                                    new Pose(85.000, 22.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(250))
                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.000, 22.000),
                                    new Pose(128.000, 27.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))
                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.000, 27.000),
                                    new Pose(133.000, 8.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-90))
                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.000, 8.500),
                                    new Pose(85.000, 22.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(250))
                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.000, 22.000),
                                    new Pose(100.000, 17.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))
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
                if (timer.seconds() > 2.3) {
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
                if (timer.seconds() > 1.3) {
                    follower.followPath(paths.Path6, true);
                    wallWait = false;
                    pathState++;
                }
                break;

            case 8:
                if (!wallWait) {
                    wallWait = true;
                    timer.reset();
                }
                if (timer.seconds() > 2.3) {
                    follower.followPath(paths.Path7, true);
                    wallWait = false;
                    pathState++;
                }
                break;

            case 9:
                if (!wallWait) {
                    wallWait = true;
                    timer.reset();
                }
                if (timer.seconds() > 2.3) {
                    follower.followPath(paths.Path8, true);
                    wallWait = false;
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
                if (!wallWait) {
                    wallWait = true;
                    timer.reset();
                }
                if (timer.seconds() > 2.3) {
                    follower.followPath(paths.Path10, true);
                    wallWait = false;
                    pathState++;
                }
                break;

            case 13:
                if (!wallWait) {
                    wallWait = true;
                    timer.reset();
                }
                if (timer.seconds() > 1.3) {
                    follower.followPath(paths.Path11, true);
                    wallWait = false;
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