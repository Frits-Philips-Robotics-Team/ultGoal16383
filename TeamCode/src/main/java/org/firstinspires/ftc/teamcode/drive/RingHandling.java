package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RingHandling {
    DcMotorEx feeder;
    DcMotorEx shooter;

    public RingHandling(HardwareMap hardwareMap) {

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

    }

    public void setRPM(double rpm) {
        // unit conversion from the input, rpm, to setVelocity's desired input, encoder ticks per second
        double ticksPerSecond = rpm / 60 * 28;
        shooter.setVelocity(ticksPerSecond);
    }

    public double getRPM() {
        // unit conversion from the input, encoder ticks per second, to the desired output, rpm
        double ticksPerSecond = shooter.getVelocity();
        return ticksPerSecond / 28 * 60;
    }

}
