package org.firstinspires.ftc.teamcode.pedroPathing.customClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class shooterControl {

    public DcMotorEx shooterL;
    public DcMotorEx shooterR;

    public double targetVelocity;

    // Tune these
    PIDFCoefficients shooterPIDF = new PIDFCoefficients(200.0, 0.0, 10.0, 12.3);

    public shooterControl(HardwareMap hardwareMap) {

        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        shooterL.setDirection(DcMotorEx.Direction.REVERSE);
        shooterR.setDirection(DcMotorEx.Direction.FORWARD);
        shooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterPIDF);
    }

    public void setShooterVelocity(double targetVelocity) {
        shooterL.setVelocity(targetVelocity);
        shooterR.setVelocity(targetVelocity);

        double currentVelocityL = shooterL.getVelocity();
        double currentVelocityR = shooterR.getVelocity();

        double errorL = targetVelocity - currentVelocityL;
        double errorR = targetVelocity - currentVelocityR;
    }
}
