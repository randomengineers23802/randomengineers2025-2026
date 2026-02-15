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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.control.passthrough;
import org.firstinspires.ftc.teamcode.control.robotControl;

@Autonomous(name = "closeBlueTest2", group = "Autonomous")
@Configurable
public class closeBlueTest2 extends OpMode {

    private robotControl robot;
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime timer = new ElapsedTime();
    private boolean pathStarted = false;
    private boolean gateWait = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(24.500, 128.000, Math.toRadians(323.5)));
        paths = new Paths(follower);
        robot = new robotControl(hardwareMap, follower, gamepad1);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        robot.intakeOn();
        robot.setShooterVelocity("close");

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
                                    new Pose(24.500, 128.000),

                                    new Pose(58.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(323.5), Math.toRadians(316))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.000, 80.000),
                                    new Pose(55.000, 59.000),
                                    new Pose(10.000, 58.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(316), Math.toRadians(200), 0.15)

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10.000, 58.000),
                                    new Pose(39.000, 61.000),
                                    new Pose(58.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(316))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.000, 80.000),
                                    new Pose(60.000, 69.000),
                                    new Pose(7.000, 58.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(316), Math.toRadians(140), 0.15)

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(7.000, 58.000),
                                    new Pose(40.000, 62.000),
                                    new Pose(58.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(316))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 80.000),

                                    new Pose(18.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(316), Math.toRadians(175), 0.15)

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.000, 84.000),

                                    new Pose(58.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(316))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.000, 80.000),
                                    new Pose(85.000, 30.000),
                                    new Pose(10.000, 33.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(316), Math.toRadians(180), 0.15)

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.000, 33.000),

                                    new Pose(58.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 80.000),

                                    new Pose(25.000, 70.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(316), Math.toRadians(270))

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
                if (follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.Path2, true);
                    pathState++;
                }
                break;

            case 3:
                if (follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.Path3, true);
                    pathState++;
                }
                break;

            case 4:
                Shoot();
                break;

            case 5:
                if (follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.Path4, true);
                    pathState++;
                }
                break;

            case 6:
                if (!gateWait) {
                    gateWait = true;
                    timer.reset();
                }
                if (timer.seconds() > 4.0) {
                    follower.followPath(paths.Path5, true);
                    pathState++;
                }
                break;

            case 7:
                Shoot();
                break;

            case 8:
                if (follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.Path6, true);
                    pathState++;
                }
                break;

            case 9:
                if (follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.Path7, true);
                    pathState++;
                }
                break;

            case 10:
                Shoot();
                break;

            case 11:
                if (follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.Path8, true);
                    pathState++;
                }
                break;

            case 12:
                if (follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.Path9, true);
                    pathState++;
                }
                break;

            case 13:
                Shoot();
                break;

            case 14:
                if (follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.Path10, true);
                    pathStarted = true;
                }
                if (!follower.isBusy())
                    pathState++;
                break;


            default:
                robot.shooterStop();
                robot.intakeOff();

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