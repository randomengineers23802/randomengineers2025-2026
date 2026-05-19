package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    public DcMotorEx shooterL;
    public DcMotorEx shooterR;
    PIDFCoefficients shooterLPIDF = new PIDFCoefficients(120.0, 0.0, 0.0, 13.1);
    PIDFCoefficients shooterRPIDF = new PIDFCoefficients(120.0, 0.0, 0.0, 12.5);

    public Shooter(HardwareMap hardwareMap) {
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        shooterL.setDirection(DcMotorEx.Direction.REVERSE);
        shooterR.setDirection(DcMotorEx.Direction.FORWARD);
        shooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterLPIDF);
        shooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterRPIDF);
    }

    public void setVelocity(String range) {
        double targetVelocity = 0;
        switch (range) {
            case "close":
                targetVelocity = 1100;
                break;
            case "far":
                targetVelocity = 1180;
                break;
        }
        shooterL.setVelocity(targetVelocity);
        shooterR.setVelocity(targetVelocity);
    }

    public void setVelocity(double ticks) {
        shooterL.setVelocity(ticks);
        shooterR.setVelocity(ticks);
    }

    public void off() {
        shooterL.setPower(0);
        shooterR.setPower(0);
    }
}
