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



public class GrabberHandling {
    Servo arm;
    Servo gripper;

    public GrabberHandling(@NotNull HardwareMap hardwareMap) {
        arm = hardwareMap.get(Servo.class, "arm");
        gripper = hardwareMap.get(Servo.class, "gripper");
    }


    public void moveLeftGrabber(String armPos, String gripperPos) {
        final double upBlockValue = 0.58;
        final double upEmptyValue = 0.49;
        final double downValue = 0.81;
        final double openValue = 0.45;
        final double closedValue = 0.85;
        final double initialGripperValue = 1;

        switch (armPos) {
            case "upBlock": arm.setPosition(upBlockValue);
                break;
            case "upEmpty": arm.setPosition(upEmptyValue);
                break;
            case "down":    arm.setPosition(downValue);
                break;
            default: break;
        }
        switch (gripperPos) {
            case "open":    gripper.setPosition(openValue);
                break;
            case "closed":  gripper.setPosition(closedValue);
                break;
            case "initial": gripper.setPosition(initialGripperValue);
                break;
            default: break;
        }
    }
}
