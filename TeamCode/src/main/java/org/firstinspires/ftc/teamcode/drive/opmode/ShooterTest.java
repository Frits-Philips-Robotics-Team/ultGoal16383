/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RingHandling;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PersistentStorage;

@Config
@TeleOp(name="ShooterTest", group="basic")
public class ShooterTest extends OpMode
{
    SampleMecanumDrive drive;
    RingHandling rings;
    ElapsedTime rotateTimer = new ElapsedTime();
    ElapsedTime gameTimer = new ElapsedTime();

    boolean wasRotating;
    double rotationSetpoint;
    double currentHeading;
    double rotationError;
    double rotate;
    final double adjustmentSpeed = 0.4;
    ElapsedTime matchTimer = new ElapsedTime();

    Pose2d poseOffset;

    double rpmSetpoint;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Declare OpMode members.
        drive = new SampleMecanumDrive(hardwareMap);
        rings = new RingHandling(hardwareMap);

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(PersistentStorage.currentPose);

        wasRotating = false;
        rotationSetpoint = currentHeading = PersistentStorage.currentPose.getHeading();
        poseOffset = new Pose2d(0, 0, 0);
        rpmSetpoint = 6000;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        drive.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        matchTimer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from gamepad x/y, then rotate it by current robot heading.
        // If applicable, use offset to change field centric orientation.
        Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).times(0.3 + 0.7 * gamepad1.right_trigger)
                .rotated((-poseEstimate.getHeading()) - poseOffset.getHeading());

        if (Math.abs(rotationSetpoint - currentHeading) < Math.abs(rotationSetpoint - (currentHeading - (2 * Math.PI)))) {
            rotationError = rotationSetpoint - currentHeading;
        }
        else {
            rotationError = rotationSetpoint - (currentHeading - (2 * Math.PI));
        }

        currentHeading = poseEstimate.getHeading();

        if (gamepad1.right_stick_x == 0 && !wasRotating) {
            rotate = adjustmentSpeed * (rotationError);
            if (rotate >= 0) {
                rotate = Math.pow(rotate, 0.6);
            }
            else {
                rotate = -Math.pow(-rotate, 0.6);
            }
        } else if (wasRotating && gamepad1.right_stick_x == 0) {
            if (rotateTimer.milliseconds() > 300) {
                wasRotating = false;
            }
            rotationSetpoint = currentHeading;
            rotate = 0;
        } else if (!wasRotating) {
            wasRotating = true;
            rotate = -gamepad1.right_stick_x;
            rotateTimer.reset();
        } else {
            rotate = -gamepad1.right_stick_x;
            rotateTimer.reset();
        }

        if (Math.abs(rotate) < 0.05) {
            rotate = 0;
        }
        // Pass rotated input + right stick value for rotation to drive function
        drive.setDrivePower(new Pose2d(input.getX(), input.getY(), rotate));

        // Sets an offset so that the current rotation is used as field perspective for field centric drive
        if (gamepad1.right_bumper) {
            poseOffset = new Pose2d(0, 0, poseEstimate.getHeading());
        }

        if (gamepad1.a) {
            rings.setIntake(1);
        }
        else if (gamepad1.y) {
            rings.setIntake(0);
        }
        else if (gamepad1.x) {
            rings.setIntake(-1);
        }

        // Broadly setting rpm with trigger and finetuning using dpad
        if (gamepad2.right_bumper) {
            rpmSetpoint = gamepad2.right_trigger * 6000;
        }
        else if (gamepad2.dpad_down) {
            rpmSetpoint -= 10;
        }
        else if (gamepad2.dpad_up) {
            rpmSetpoint += 10;
        }

        // Shooter disable and enable
        if (gamepad2.y) {
            rings.setRPM(0);
        }
        else if (gamepad2.x) {
            rings.setRPM(rpmSetpoint);
        }

        if (gamepad2.left_bumper) {
            rings.triggerPusher(matchTimer.milliseconds());
        }

        // updates everything. Localizer, drive functions, etc.
        drive.update();
        rings.update(matchTimer.milliseconds());
        // This is what the shooter test is all about
        telemetry.addData("Set RPM: ", (int) rpmSetpoint);
        telemetry.addData("Current RPM: ", (int) rings.getRPM());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
