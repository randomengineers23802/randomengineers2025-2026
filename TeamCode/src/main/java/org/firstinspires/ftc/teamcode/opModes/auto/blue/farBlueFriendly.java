package org.firstinspires.ftc.teamcode.opModes.auto.blue;

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

@Autonomous(name = "farBlueFriendly", group = "Autonomous")
@Configurable
public class farBlueFriendly extends OpMode {

    private robotControl robot;
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime timer = new ElapsedTime();
    private boolean wallWait = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.875, 8.5625, Math.toRadians(0)));
        paths = new Paths(follower);
        robot = new robotControl(hardwareMap, follower);
        robot.setAlliance(Alliance.BLUE);
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
        robot.setShooterVelocity(shotParameters.flywheelTicks + 30);
        telemetry.addData("target ticks", shotParameters.flywheelTicks + 20);
        telemetry.addData("shooter l ticks", robot.ShooterL.getVelocity());
        telemetry.addData("shooter r ticks", robot.ShooterR.getVelocity());
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
        public PathChain Path13;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.875, 8.563),

                                    new Pose(58.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(287))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.000, 15.000),
                                    new Pose(66.000, 40.000),
                                    new Pose(10.000, 37.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(287), Math.toRadians(180), 0.2)

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.000, 37.000),

                                    new Pose(58.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(290))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 15.000),

                                    new Pose(10.000, 17.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(210), 0.4)

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.000, 17.000),

                                    new Pose(21.000, 17.000)
                            )
                    ).setConstantHeadingInterpolation(180)

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.000, 17.000),

                                    new Pose(15.000, 15.000)
                            )
                    ).setConstantHeadingInterpolation(210)

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.000, 15.000),

                                    new Pose(8.000, 8.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(210))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.000, 8.500),

                                    new Pose(58.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(290))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 15.000),

                                    new Pose(9.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(180), 0.4)

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.000, 12.000),

                                    new Pose(58.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(290))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 15.000),

                                    new Pose(9.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(180), 0.4)

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.000, 12.000),

                                    new Pose(58.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(290))

                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 15.000),

                                    new Pose(36.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(180))

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
                if (follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.Path5, true);
                    pathState++;
                }
                break;

            case 7:
                if (follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.Path6, true);
                    pathState++;
                }
                break;

            case 8:
                if (!wallWait) {
                    wallWait = true;
                    timer.reset();
                }
                if (timer.seconds() > 1.3) {
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
                if (timer.seconds() > 1.3) {
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
                Shoot();
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11, true);
                    pathState++;
                }
                break;

            case 15:
                if (!wallWait) {
                    wallWait = true;
                    timer.reset();
                }
                if (timer.seconds() > 2.3) {
                    follower.followPath(paths.Path12, true);
                    wallWait = false;
                    pathState++;
                }
                break;

            case 16:
                Shoot();
                break;

            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path13, true);
                    pathState++;
                }
                break;

            default:
                robot.intakeOff();
                robot.beltOff();
                break;
        }
        return pathState;
    }

    @Override
    public void stop() {
        passthrough.startPose = follower.getPose();
    }
}