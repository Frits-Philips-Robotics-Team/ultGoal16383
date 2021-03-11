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


    public void moveGrabber(String armPos, String gripperPos) {
        final double upHalfValue = 0.5;
        final double inSizeValue = 0.84;
        final double downValue = 0.15;
        final double openValue = 1;
        final double closedValue = 0.55;

        switch (armPos) {
            case "upHalf": arm.setPosition(upHalfValue);
                break;
            case "inSize": arm.setPosition(inSizeValue);
                break;
            case "down":   arm.setPosition(downValue);
                break;
            default: break;
        }
        switch (gripperPos) {
            case "open":    gripper.setPosition(openValue);
                break;
            case "closed":  gripper.setPosition(closedValue);
                break;
            default: break;
        }
    }
}
