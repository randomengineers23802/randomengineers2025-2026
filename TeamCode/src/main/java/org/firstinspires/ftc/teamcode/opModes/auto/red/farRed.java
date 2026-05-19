package org.firstinspires.ftc.teamcode.opModes.auto.red;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.AutoOpMode;
import org.firstinspires.ftc.teamcode.control.Alliance;

@Autonomous
@Configurable
public class farRed extends AutoOpMode {
    private Paths paths;
    private ElapsedTime timer = new ElapsedTime();
    private boolean wallWait = false;

    @Override
    public void init() {
        super.init();
        follower.setStartingPose(new Pose(87.125, 8.5625, Math.toRadians(180)));
        paths = new Paths(follower);
        robot.setAlliance(Alliance.RED);
    }

    @Override
    public void loop() {
        super.loop();
        pathState = autonomousPathUpdate();
        robot.shooter.setVelocity(shotParameters.flywheelTicks + 30);
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.125, 8.563),
                                    new Pose(86.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(253))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.000, 15.000),
                                    new Pose(78.000, 40.000),
                                    new Pose(134.000, 37.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(253), Math.toRadians(0), 0.2)
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.000, 37.000),
                                    new Pose(86.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(250))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 15.000),
                                    new Pose(134.000, 17.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(330), 0.4)
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.000, 17.000),
                                    new Pose(123.000, 17.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.000, 17.000),
                                    new Pose(129.000, 15.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(330))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.000, 15.000),
                                    new Pose(136.000, 8.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(330))
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.000, 8.500),
                                    new Pose(86.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(250))
                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 15.000),
                                    new Pose(135.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0), 0.4)
                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.000, 12.000),
                                    new Pose(86.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(250))
                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 15.000),
                                    new Pose(135.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0), 0.4)
                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.000, 12.000),
                                    new Pose(86.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(250))
                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 15.000),
                                    new Pose(108.000, 15.000)
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
                follower.followPath(paths.Path2, true);
                pathState++;
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
                follower.followPath(paths.Path4, true);
                pathState++;
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
                follower.followPath(paths.Path9, true);
                pathState++;
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
                follower.followPath(paths.Path11, true);
                pathState++;
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
                follower.followPath(paths.Path13, true);
                pathState++;
                break;

            default:
                robot.intake.off();
                robot.belt.off();
                break;
        }
        return pathState;
    }
}