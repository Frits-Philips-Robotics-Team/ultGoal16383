package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;

public class RingHandling {
    DcMotorEx feeder;
    DcMotorEx shooter;
    DcMotorEx intakeChain;
    DcMotorEx intakeSingle;
    Servo pusher;

    double pusherStart;

    public RingHandling(@NotNull HardwareMap hardwareMap) {

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeChain = hardwareMap.get(DcMotorEx.class, "intakeChain");
        intakeSingle = hardwareMap.get(DcMotorEx.class, "intakeSingle");
        pusher = hardwareMap.get(Servo.class, "pusher");

        pusher.setPosition(0.2);

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
        shooter.setVelocity(-ticksPerSecond);
    }

    public double getRPM() {
        // unit conversion from the input, encoder ticks per second, to the desired output, rpm
        double ticksPerSecond = shooter.getVelocity();
        return ticksPerSecond / 28 * 60;
    }

    public void triggerPusher(double startTime) {
        pusher.setPosition(0.4);
        pusherStart = startTime;
    }

    public void update(double time) {
        if (pusherStart >= 0) {
            if (time - pusherStart >= 800) {
                pusher.setPosition(0.2);
                pusherStart = -1;
            }
        }
    }
}
