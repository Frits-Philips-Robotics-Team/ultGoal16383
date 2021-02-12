package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.stat.descriptive.moment.VectorialCovariance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.NotNull;

import java.nio.channels.Selector;
import java.util.concurrent.ThreadPoolExecutor;



public class RingHandling {
    DcMotorEx feeder;
    public DcMotorEx shooter;
    DcMotorEx intakeChain;
    DcMotorEx intakeSingle;
    Servo pusher;
    DistanceSensor distance;

    PIDFCoefficients pidf = new PIDFCoefficients(100, 0, 2, 15.6);

    double pusherStart;
    double shooterTime;

    shooterStates state_s;

    public int calcRPM;

    enum shooterStates {
        INITIALIZE, CHECKRPM, WAIT, NOTHING
    }

    public RingHandling(@NotNull HardwareMap hardwareMap) {

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeChain = hardwareMap.get(DcMotorEx.class, "intakeChain");
        intakeSingle = hardwareMap.get(DcMotorEx.class, "intakeSingle");
        pusher = hardwareMap.get(Servo.class, "pusher");
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        //shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        pusher.setPosition(0.2);

        state_s = shooterStates.NOTHING;

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
        return ticksPerSecond / 28 * -60;
    }

    public double shootGetHeading (Pose2d currentPose, String allianceColour) {
        Pose2d tower;

        if (allianceColour.equals("red")) {
            tower = new Pose2d(72, -24);
        }
        else {
            tower = new Pose2d(72, 38);
        }

        currentPose = currentPose.minus(tower);
        Vector2d vector = new Vector2d(currentPose.getX(), currentPose.getY());
        return vector.rotated(Math.PI - Math.toRadians(6)).angle();
    }

    public double shootGetRPM (Pose2d currentPose, String allianceColour) {
        double goalHeight = 30; //43
        Pose2d tower = new Pose2d();

        if (allianceColour.equals("red")) {
            tower = new Pose2d(72, -24);
        }
        else {
            tower = new Pose2d(72, 36);
        }

        currentPose = currentPose.minus(tower);
        Vector2d vector = new Vector2d(currentPose.getX(), currentPose.getY());
        double distance = 2.54 * vector.norm() - 20; // Distance to goal converted from inches to cm

        return (Math.sqrt((490*Math.pow(distance, 2))/(distance*Math.tan(Math.toRadians(28))-goalHeight)))/(0.15) - Math.pow(0.03 * distance, 2.5);
    }

    public double getDistance (Pose2d currentPose, String allianceColour) {
        Pose2d tower;

        if (allianceColour.equals("red")) {
            tower = new Pose2d(72, -38);
        }
        else {
            tower = new Pose2d(72, 36);
        }

        currentPose = currentPose.minus(tower);
        Vector2d vector = new Vector2d(currentPose.getX(), currentPose.getY());
        return 2.54 * vector.norm(); // Distance to goal converted from inches to cm
    }

    public void triggerPusher(double startTime) {
        pusher.setPosition(0.4);
        pusherStart = startTime;
    }

    public double getDistanceSensor() {
        return distance.getDistance(DistanceUnit.MM);
    }

    public int getRingNumber() {
        if (getDistanceSensor() < 40) {
            return 3;
        }
        else if (getDistanceSensor() < 60) {
            return 2;
        }
        else if (getDistanceSensor() < 83) {
            return 1;
        }
        else {
            return 0;
        }
    }

    public void shoot() {
        state_s = shooterStates.INITIALIZE;
    }

    public void update(double time, Pose2d currentPose, String allianceColour) {
        if (pusherStart >= 0) {
            if (time - pusherStart >= 600) {
                pusher.setPosition(0.2);
                pusherStart = -1;
            }
        }

        switch (state_s) {
            case INITIALIZE:
                if (getRingNumber() == 0) {
                    state_s = shooterStates.NOTHING;
                    break;
                }
                calcRPM = (int) shootGetRPM(currentPose, allianceColour);
                setRPM(calcRPM);
                state_s = shooterStates.CHECKRPM;
                break;
            case CHECKRPM:
                if (getRPM() > (calcRPM - 50) && getRPM() < (calcRPM + 50)) {
                    if (shooterTime == -1) {
                        shooterTime = time;
                    }
                    else if (time - shooterTime >= 300) {
                        triggerPusher(time);
                        shooterTime = -1;
                        state_s = shooterStates.WAIT;
                    }
                }
                else {
                    shooterTime = -1;
                }
                break;
            case WAIT:
                if (pusherStart != -1) {
                    break;
                }
                else if (getRingNumber() != 0) {
                    state_s = shooterStates.CHECKRPM;
                }
                else {
                    setRPM(0);
                    state_s = shooterStates.NOTHING;
                }
                break;
            case NOTHING:
                break;
        }
    }
}
