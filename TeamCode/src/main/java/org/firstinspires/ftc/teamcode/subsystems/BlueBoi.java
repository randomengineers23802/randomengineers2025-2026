package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class BlueBoi extends ServoImpl {

    public BlueBoi(HardwareMap hardwareMap) {
        this((ServoImpl) hardwareMap.get(Servo.class, "blueBoi"));
    }

    private BlueBoi(ServoImpl baseServo) {
        super(baseServo.getController(), baseServo.getPortNumber());
        this.setPosition(0.65);
    }

    public void open() { this.setPosition(1.0); }
    public void close() { this.setPosition(0.65); }
}