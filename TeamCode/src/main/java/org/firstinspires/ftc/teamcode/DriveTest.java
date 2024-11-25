/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Drive Test", group="Linear OpMode")

public class DriveTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftArmSlidesM = null;
    private DcMotor rightArmSlidesM = null;
    private DcMotor armAxelM = null;
    Servo clawS;

    private double slide_pos = 0;

    private double axel_pos = 0;

    private int get_arm_pos(){
            return leftArmSlidesM.getCurrentPosition();
    }

    private int get_axel_pos(){
            return armAxelM.getCurrentPosition();
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // M = Motor
        // S = Servo

        leftArmSlidesM = hardwareMap.get(DcMotor.class, "left_arm_slides");
        rightArmSlidesM = hardwareMap.get(DcMotor.class, "right_arm_slides");

        armAxelM = hardwareMap.get(DcMotor.class, "arm_axel");

        clawS = hardwareMap.get(Servo.class, "claw");

        leftArmSlidesM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armAxelM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // 3
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);  // 2
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);  // 1
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);  // 0
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;
            double yaw =  gamepad1.left_stick_x;
            double lateral  = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

            // Right Front and Right back have switched lateral signs because of wire swap
            double leftFrontPower  = axial - lateral + yaw;
            double rightFrontPower = axial + lateral - yaw;
            double leftBackPower   = axial + lateral + yaw;
            double rightBackPower  = axial - lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Sends Calculated Power to Wheels
            leftFrontDrive.setPower(leftFrontPower/1.5);
            rightFrontDrive.setPower(rightFrontPower/1.5);
            leftBackDrive.setPower(leftBackPower/1.5);
            rightBackDrive.setPower(rightBackPower/1.5);


            if (gamepad1.right_trigger > 0 && axel_pos > (4000*0.2698) && slide_pos < 2000) {   // 1079.2 for axel pos
                leftArmSlidesM.setPower(0.8);  // Slides EXTEND
                rightArmSlidesM.setPower(-0.8);
            }
            else if(gamepad1.right_trigger > 0 && axel_pos <= (4000*0.2698) && slide_pos <= 3000){   // 1079.2 for axel pos
                leftArmSlidesM.setPower(0.8);  // Slides EXTEND
                rightArmSlidesM.setPower(-0.8);
            }

            else if (gamepad1.left_trigger > 0){
                    leftArmSlidesM.setPower(-0.8); // Slides DESCEND
                    rightArmSlidesM.setPower(0.8);
            }
            else if (axel_pos > (4000*0.2698) && slide_pos > 2200) { // Pulls out of illegal zone
                leftArmSlidesM.setPower(-0.8); // Slides DESCEND
                rightArmSlidesM.setPower(0.8);
            }
            else {
                    leftArmSlidesM.setPower(0); // Slides DON'T MOVE
                    rightArmSlidesM.setPower(0);
            }



            telemetry.addData("Slide pos", slide_pos);
            telemetry.addData("Axel pos", axel_pos);
            telemetry.update();

            if(gamepad1.dpad_down && axel_pos < (4000*0.2698)) {
                armAxelM.setPower(0.6);  // Axel Rotates UP
            }
            else if (gamepad1.dpad_down && axel_pos >= (4000*0.2698)) {
                armAxelM.setPower(0.3);  // Axel Rotates UP, slowly near the ground
            }
            else if(gamepad1.dpad_up){
                armAxelM.setPower(-0.6);  // Axel Rotates DOWN
            }
            else{
                armAxelM.setPower(0.0);
            }


            if(gamepad1.x){
                clawS.setPosition(0.6); // CLOSE Claw
            }
            else if (gamepad1.a){
                clawS.setPosition(0.0);  // OPEN Claw
            }

            slide_pos = get_arm_pos();
            axel_pos = get_axel_pos();


            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //telemetry.addData("Yaw / Lateral / Axial", "%4.2f, %4.2f, %4.2f", yaw, lateral, axial);
            telemetry.update();
        }
    }}
