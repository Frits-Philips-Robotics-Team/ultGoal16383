package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.stat.descriptive.moment.VectorialCovariance;
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

    public double shootGetHeading (Pose2d currentPose, String allianceColour) {
        Pose2d tower = new Pose2d();

        if (allianceColour.equals("red")) {
            tower = new Pose2d(72, -38);
        }
        else {
            tower = new Pose2d(72, 38);
        }

        currentPose = currentPose.minus(tower);
        Vector2d vector = new Vector2d(currentPose.getX(), currentPose.getY());
        return vector.rotated(Math.PI - Math.toRadians(9)).angle();
    }

    public double shootGetRPM (Pose2d currentPose, String allianceColour) {
        double goalHeight = 42;
        Pose2d tower = new Pose2d();

        if (allianceColour.equals("red")) {
            tower = new Pose2d(72, -38);
        }
        else {
            tower = new Pose2d(72, 38);
        }

        currentPose = currentPose.minus(tower);
        Vector2d vector = new Vector2d(currentPose.getX(), currentPose.getY());
        double distance = 2.54 * vector.norm() - 20; // Distance to goal converted from inches to cm

        return (Math.sqrt((490*Math.pow(distance, 2))/(distance*Math.tan(Math.toRadians(28))-goalHeight)))/(0.15) - Math.pow(0.03 * distance, 2.5);
    }

    public double getDistance (Pose2d currentPose, String allianceColour) {
        Pose2d tower = new Pose2d();

        if (allianceColour.equals("red")) {
            tower = new Pose2d(72, -38);
        }
        else {
            tower = new Pose2d(72, 38);
        }

        currentPose = currentPose.minus(tower);
        Vector2d vector = new Vector2d(currentPose.getX(), currentPose.getY());
        return 2.54 * vector.norm(); // Distance to goal converted from inches to cm
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
