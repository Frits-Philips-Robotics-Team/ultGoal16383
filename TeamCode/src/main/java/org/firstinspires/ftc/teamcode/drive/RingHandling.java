package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;

public class RingHandling {
    DcMotorEx feeder;
    DcMotorEx shooter;
    DcMotorEx intakeChain;
    DcMotorEx intakeSingle;

    public RingHandling(@NotNull HardwareMap hardwareMap) {

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeChain = hardwareMap.get(DcMotorEx.class, "intakeChain");
        intakeSingle = hardwareMap.get(DcMotorEx.class, "intakeSingle");

        intakeChain.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setIntake(double power) {
        intakeSingle.setPower(power);
        intakeChain.setPower(power);
    }

    public double getIntake() {
        return intakeChain.getPower();
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
