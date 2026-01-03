package org.firstinspires.ftc.teamcode.pedroPathing.auto.normal.blue;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.passthrough;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.shooterControl;

@Autonomous(name = "farBlueShort", group = "Autonomous")
@Configurable
public class farBlueShort extends OpMode {

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
        intake = hardwareMap.get(DcMotor.class, "intake");
        belt = hardwareMap.get(DcMotor.class, "belt");
        belt.setDirection(DcMotor.Direction.REVERSE);

        BlueBoi = hardwareMap.get(Servo.class, "BlueBoi");
        BlueBoi.setPosition(0.65);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.875, 8.5625, Math.toRadians(90)));

        paths = new Paths(follower);

        shooter = new shooterControl(hardwareMap, follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        intake.setPower(1.0);
        shooter.setShooterVelocity("far");
        belt.setPower(0.8);
        BlueBoi.setPosition(0.65);
        timer.reset();
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
                BlueBoi.setPosition(1.0);
            }
            else {
                BlueBoi.setPosition(0.65);
                pathState++;
            }
        }
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.875, 8.563), new Pose(58.000, 15.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(292))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 15.000), new Pose(36.000, 15.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(292), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                double t = timer.seconds();
                if (t >= 2.5 && !follower.isBusy()) {
                    follower.followPath(paths.Path1, true);
                    pathState++;
                }
                break;

            case 1:
                Shoot();
                break;

            case 2:
                if (!pathStarted) {
                    follower.followPath(paths.Path2, true);
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
