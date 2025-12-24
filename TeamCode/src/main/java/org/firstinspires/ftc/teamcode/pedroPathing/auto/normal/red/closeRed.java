package org.firstinspires.ftc.teamcode.pedroPathing.auto.normal.red;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.passthrough;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.shooterControl;

@Autonomous(name = "closeRed", group = "Autonomous")
@Configurable
public class closeRed extends OpMode {

    private shooterControl shooter;
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime timer = new ElapsedTime();

    private DcMotor intake = null;
    private DcMotor belt = null;
    private Servo BlueBoi = null;
    private boolean pathStarted = false;

    @Override
    public void init() {
        shooter = new shooterControl(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        belt = hardwareMap.get(DcMotor.class, "belt");
        belt.setDirection(DcMotor.Direction.REVERSE);
        BlueBoi = hardwareMap.get(Servo.class, "BlueBoi");

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(119.5, 128, Math.toRadians(216.5)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        intake.setPower(0.0);
        shooter.setShooterVelocity(1000);
        belt.setPower(0.5);
        BlueBoi.setPosition(0.65);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    private void Shoot() {
        if (follower.isBusy())
            timer.reset();
        else {
            double t = timer.seconds();
            if (t <= 1.0) {
                intake.setPower(0.75);
                BlueBoi.setPosition(1.0);
            }
            else {
                intake.setPower(0.0);
                BlueBoi.setPosition(0.65);
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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(119.500, 128.000), new Pose(84.000, 95.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(216.5), Math.toRadians(220))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(84.000, 95.000), new Pose(95.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(95.000, 84.000), new Pose(129.000, 84.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(129.000, 84.000), new Pose(84.000, 95.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(84.000, 95.000), new Pose(95.000, 57.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(95.000, 57.000), new Pose(132.000, 57.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(132.000, 57.000),
                            new Pose(84.000, 54.000),
                            new Pose(84.000, 95.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(84.000, 95.000), new Pose(95.000, 35.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
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
                    intake.setPower(0.0);
                    follower.followPath(paths.Path4, true);
                    pathState++;
                }
                break;

            case 5:
                Shoot();
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    pathState++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    intake.setPower(0.0);
                    follower.followPath(paths.Path7, true);
                    pathState++;
                }
                break;

            case 9:
                Shoot();
                break;

            case 10:
                if (!pathStarted) {
                    follower.followPath(paths.Path8, true);
                    pathStarted = true;
                }
                if (!follower.isBusy())
                    pathState++;
                break;

            default:
                shooter.shooterStop();
                intake.setPower(0);
                belt.setPower(0);
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
