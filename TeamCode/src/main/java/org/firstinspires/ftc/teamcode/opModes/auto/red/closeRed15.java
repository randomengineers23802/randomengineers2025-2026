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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.control.passthrough;
import org.firstinspires.ftc.teamcode.control.robotControl;

@Autonomous(name = "closeRed15", group = "Autonomous")
@Configurable
public class closeRed15 extends OpMode {

    private robotControl robot;
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime timer = new ElapsedTime();
    private boolean gateWait = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(119.500, 128.000, Math.toRadians(216.5)));
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
                                    new Pose(119.500, 128.000),

                                    new Pose(86.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(216.5), Math.toRadians(224))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.000, 80.000),
                                    new Pose(89.000, 59.000),
                                    new Pose(134.000, 58.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(224), Math.toRadians(340), 0.15)

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.000, 58.000),
                                    new Pose(105.000, 61.000),
                                    new Pose(86.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(224))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.000, 80.000),
                                    new Pose(84.000, 69.000),
                                    new Pose(137.000, 58.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(224), Math.toRadians(400), 0.15)

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(137.000, 58.000),
                                    new Pose(104.000, 62.000),
                                    new Pose(86.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(400), Math.toRadians(224))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 80.000),

                                    new Pose(126.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(224), Math.toRadians(365), 0.15)

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.000, 84.000),

                                    new Pose(86.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(365), Math.toRadians(224))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.000, 80.000),
                                    new Pose(59.000, 30.000),
                                    new Pose(134.000, 33.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(224), Math.toRadians(360), 0.15)

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.000, 33.000),

                                    new Pose(86.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(224))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 80.000),

                                    new Pose(119.000, 70.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(224), Math.toRadians(270))

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
                if (follower.getCurrentTValue() > 0.95 && follower.getCurrentTValue() != 1.0) {
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
                if (follower.getCurrentTValue() > 0.95 && follower.getCurrentTValue() != 1.0) {
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
                if (follower.getCurrentTValue() > 0.95 && follower.getCurrentTValue() != 1.0) {
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