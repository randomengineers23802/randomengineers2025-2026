package org.firstinspires.ftc.teamcode.pedroPathing.auto.random;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.passthrough;

@Autonomous(name = "farBlueCloseSpedatron", group = "Autonomous")
@Configurable
@Disabled
public class farBlueCloseSpedatron extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime timer = new ElapsedTime();
    private DcMotorEx ShooterL = null;
    private DcMotorEx ShooterR = null;
    private DcMotor intake = null;
    private DcMotor belt = null;
    private Servo LinearServo = null;
    private Servo BlueBoi = null;
    private boolean shooting = false;
    private boolean pathStarted = false;

    @Override
    public void init() {
        ShooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        ShooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        belt = hardwareMap.get(DcMotor.class, "belt");
        ShooterL.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setDirection(DcMotor.Direction.REVERSE);
        ShooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ShooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LinearServo = hardwareMap.get(Servo.class, "LinearServo");
        LinearServo.setPosition(0.1);
        BlueBoi = hardwareMap.get(Servo.class, "BlueBoi");
        BlueBoi.setPosition(0.65);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.875, 8.5625, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        intake.setPower(0.75);
        ShooterL.setVelocity(840);
        ShooterR.setVelocity(840);
        belt.setPower(0.5);
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
        panelsTelemetry.debug("Current Path", pathState);
        panelsTelemetry.update(telemetry);

        if (shooting)
            Shoot();
    }

    public void Shoot() {
        if (follower.isBusy())
            timer.reset();
        else {
            double t = timer.seconds();
            if (t <= 0.5)
                BlueBoi.setPosition(0.65);
            else if (t <= 2.5)
                BlueBoi.setPosition(1.0);
            else {
                BlueBoi.setPosition(0.65);
                shooting = false;
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

        public Paths(Follower follower) {

            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.875, 8.563),
                                    new Pose(60.000, 95.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(318)
                    )
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(72.000, 81.000),
                                    new Pose(12.000, 84.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(12.000, 84.000),
                                    new Pose(60.000, 95.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(318)
                    )
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 81.000),
                                    new Pose(67.000, 59.000),
                                    new Pose(9.000, 59.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(9.000, 59.000),
                                    new Pose(60.000, 95.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(318)
                    )
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(72.000, 81.000),
                                    new Pose(39.000, 15.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(318),
                            Math.toRadians(270)
                    )
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                double t = timer.seconds();
                if (t >= 3.5 && !follower.isBusy()) {
                    follower.followPath(paths.Path1, true);
                    pathState++;
                }
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
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState++;
                }
                break;

            case 7:
                Shoot();
                break;

            case 8:
                if (!pathStarted) {
                    follower.followPath(paths.Path6, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            default:
                ShooterL.setPower(0);
                ShooterR.setPower(0);
                intake.setPower(0);
                belt.setPower(0);
                follower.breakFollowing();
                panelsTelemetry.debug("Status", "Autonomous Complete");
                panelsTelemetry.update(telemetry);
                passthrough.startPose = follower.getPose();  //sets pose at end of auto
                panelsTelemetry.debug("heading", follower.getHeading());
                panelsTelemetry.debug("X", follower.getPose().getX());
                panelsTelemetry.debug("heading",follower.getPose().getY());
                panelsTelemetry.update(telemetry);
                break;
        }
        return pathState;
    }
}
