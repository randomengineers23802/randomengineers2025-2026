package org.firstinspires.ftc.teamcode.pedroPathing.auto.normal.blue;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.passthrough;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.shooterControl;

@Autonomous(name = "closeBlue", group = "Autonomous")
@Configurable
public class closeBlue extends OpMode {

    private shooterControl shooter;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
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
        follower.setStartingPose(new Pose(24.500, 128.000, Math.toRadians(323.5)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        intake.setPower(0.0);
        shooter.setShooterVelocity(1060);
        belt.setPower(1.0);
        BlueBoi.setPosition(0.65);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
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
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.500, 128.000), new Pose(58.000, 80.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(323.5), Math.toRadians(311), 0.8)
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 80.000), new Pose(49.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(311), Math.toRadians(180), 0.8)
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(49.000, 84.000), new Pose(15.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 84.000), new Pose(58.000, 80.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(311), 0.8)
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 80.000), new Pose(49.000, 63.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(311), Math.toRadians(180), 0.8)
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(49.000, 63.000), new Pose(15.000, 63.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 63.000), new Pose(58.000, 80.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(311), 0.8)
                    .setReversed()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 80.000), new Pose(49.000, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(311), Math.toRadians(180), 0.8)
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(49.000, 35.500), new Pose(15.000, 35.500))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 35.500), new Pose(58.000, 80.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(311), 0.8)
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 80.000), new Pose(50.000, 128.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(311), Math.toRadians(180), 0.8)
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intake.setPower(0.0);
                follower.followPath(paths.Path1, true);
                pathState++;
                break;

            case 1:
                Shoot();
                break;

            case 2:
                intake.setPower(0.0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    intake.setPower(0.75);
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
                    intake.setPower(0.0);
                    follower.followPath(paths.Path5, true);
                    pathState++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    intake.setPower(0.75);
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
                if (!follower.isBusy()) {
                    intake.setPower(0.0);
                    follower.followPath(paths.Path8, true);
                    pathState++;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    intake.setPower(0.75);
                    follower.followPath(paths.Path9, true);
                    pathState++;
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    intake.setPower(0.0);
                    follower.followPath(paths.Path10, true);
                    pathState++;
                }
                break;

            case 13:
                Shoot();
                break;

            case 14:
                if (!pathStarted) {
                    intake.setPower(0.0);
                    follower.followPath(paths.Path11, true);
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
