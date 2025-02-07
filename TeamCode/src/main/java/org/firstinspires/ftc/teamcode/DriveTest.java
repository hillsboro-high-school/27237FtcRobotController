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

import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

import java.util.Objects;

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

    TouchSensor resetSlide;
    TouchSensor resetAxel;
    private double slide_pos = 0;

    private double axel_pos = 0;
    private boolean axel_stop = false;

    public double getSlidePos(){
        return (leftArmSlidesM.getCurrentPosition() - slide_pos);
    }
    public double getAxelPos(){
            return (armAxelM.getCurrentPosition() - axel_pos);
    }
    public void resetSlidePos(){
        slide_pos = leftArmSlidesM.getCurrentPosition();
    }
    public void resetAxelPos(){
        axel_pos = armAxelM.getCurrentPosition();
    }

    private double lastClawPos = 1;

    // Color Sensor

    ColorSensor teamColorSensor = null;
    String allianceColor;
    ColorSensor sampleColorSensor = null;

    double redR = 189.0;
    double redG = 119.5;
    double redB = 67;

    double blueR = 54.5;
    double blueG = 94.5;
    double blueB = 130.5;

    double yellowR = 239.5;
    double yellowG = 305.5;
    double yellowB = 92.7;

    int tileR;
    int tileG;
    int tileB;
    boolean full;

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

        resetSlide = hardwareMap.get(TouchSensor.class, "slide_touch");
        resetAxel = hardwareMap.get(TouchSensor.class, "axel_touch");

        leftArmSlidesM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armAxelM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        teamColorSensor = hardwareMap.get(ColorSensor.class, "team_color_sensor");
        sampleColorSensor = hardwareMap.get(ColorSensor.class, "sample_color_sensor");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // 3
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);  // 2
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);  // 1
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);  // 0


        // Coasting Code
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        getTileColors();

        teamColor();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Team", allianceColor);
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = gamepad1.left_stick_y;
            double yaw =  -gamepad1.left_stick_x;
            double lateral  = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

            // Right Front and Right back have switched lateral signs because of wire swap
            double leftFrontPower  =   axial - lateral + yaw;
            double rightFrontPower =   axial + lateral + yaw;
            double leftBackPower   =   -axial + lateral + yaw;
            double rightBackPower  =   -axial - lateral + yaw;

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
            if (!isStopRequested()) {
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }
            else{
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                break;
            }


                if (gamepad1.right_trigger > 0 && getAxelPos() > (1600) && getSlidePos() < 2250) {
                // Manual Extension                            ^~250
                leftArmSlidesM.setPower(0.8);  // Slides EXTEND
                rightArmSlidesM.setPower(-0.8);
            }
            else if(gamepad1.right_trigger > 0 && getAxelPos() <= (1600) && getSlidePos() <= 3900){
                // Vertical Limiter                             ^~250
                leftArmSlidesM.setPower(0.8);  // Slides EXTEND
                rightArmSlidesM.setPower(-0.8);
            }

            else if (gamepad1.left_trigger > 0){
                // Manual Takedown
                    leftArmSlidesM.setPower(-0.8); // Slides DESCEND
                    rightArmSlidesM.setPower(0.8);
            }                  // V~250
            else if (getAxelPos() > (1600) && getSlidePos() > 2350) { // Pulls out of illegal zone
                // Horizontal LimiterPL
                leftArmSlidesM.setPower(-0.8); // Slides DESCEND
                rightArmSlidesM.setPower(0.8);
            }
            else {
                    leftArmSlidesM.setPower(0); // Slides DON'T MOVE
                    rightArmSlidesM.setPower(0);
            }



            //telemetry.addData("Slide pos", getSlidePos());
            //telemetry.addData("Axel pos", getAxelPos());
            //telemetry.update();

            if(gamepad1.dpad_down) {armAxelM.setPower(0.6);}  // Axel Rotates FORWARD
            else if(gamepad1.dpad_up && !axel_stop) {armAxelM.setPower(-1.0);}  // Axel Rotates BACKWARD
            else {armAxelM.setPower(0.0);}


            if(gamepad1.x && lastClawPos == 1){
                lastClawPos = 0; // OPEN Claw
            }
            else if (gamepad1.a && lastClawPos == 0){
                lastClawPos = 1;  // CLOSE Claw
            }

            clawS.setPosition(lastClawPos);

            if((Objects.equals(sampleColors(), "yellow")) || (Objects.equals(sampleColors(), allianceColor))){
                lastClawPos = 1;  // CLOSE Claw
            }

            if(!(resetSlide.isPressed())){resetSlidePos();}
            if(!(resetAxel.isPressed())){resetAxelPos(); axel_stop = true;}
            else{axel_stop = false;}

            if (gamepad1.dpad_left){
                getTileColors();
            }

            telemetry.addData("Color", sampleColors());
            telemetry.update();
        }
    }
    public String sampleColors(){
        int inputRed = sampleColorSensor.red();
        int inputGreen = sampleColorSensor.green();
        int inputBlue = sampleColorSensor.blue();

        double RI;
        double BI;
        double YI;
        double EI;

        double Cry;
        double Crb;
        double Cre;

        double Cbr;
        double Cby;
        double Cbe;

        double Cye;
        double Cyr;
        double Cyb;

        double Cer;
        double Ceb;
        double Cey;

        double Cr;
        double Cb;
        double Cy;
        double Ce;

        RI = Math.sqrt( Math.pow((inputRed-redR), 2) + Math.pow((inputGreen-redG), 2) + Math.pow((inputBlue-redB), 2) );
        BI = Math.sqrt( Math.pow((inputRed-blueR), 2) + Math.pow((inputGreen-blueG), 2) + Math.pow((inputBlue-blueB), 2) );
        YI = Math.sqrt( Math.pow((inputRed-yellowR), 2) + Math.pow((inputGreen-yellowG), 2) + Math.pow((inputBlue-yellowB), 2) );
        EI = Math.sqrt( Math.pow((inputRed-tileR), 2) + Math.pow((inputGreen-tileG), 2) + Math.pow((inputBlue-tileB), 2) );

        Cry = 1 - (RI/(RI+YI));
        Crb = 1 - (RI/(RI+BI));
        Cre = 1 - (RI/(RI+EI));

        Cbr = 1 - (BI/(BI+RI));
        Cby = 1 - (BI/(BI+YI));
        Cbe = 1 - (BI/(BI+EI));

        Cye = 1 - (YI/(YI+EI));
        Cyr = 1 - (YI/(YI+RI));
        Cyb = 1 - (YI/(YI+BI));

        Cey = 1 - (EI/(EI+YI));
        Cer = 1 - (EI/(EI+RI));
        Ceb = 1 - (EI/(EI+BI));

        Cr = (Crb+Cry+Cre)/3;
        Cb = (Cbr+Cby+Cbe)/3;
        Cy = (Cyb+Cyr+Cye)/3;
        Ce = (Cer+Ceb+Cey)/3;

        if(Cr > Cb && Cr > Cy && Cr > Ce){
            return "red";
        }
        else if(Cb > Cy && Cb > Ce){
            return "blue";
        }
        else if(Cy > Ce){
            return "yellow";
        }
        else{
            return "empty";
        }
    }

    public void getTileColors(){
        tileR = sampleColorSensor.red();
        tileG = sampleColorSensor.green();
        tileB = sampleColorSensor.blue();
    }

    public void teamColor() {
        int inputRed = teamColorSensor.red();
        int inputGreen = teamColorSensor.green();
        int inputBlue = teamColorSensor.blue();

        double RI;
        double BI;

        double Cb;
        double Cr;

        RI = Math.sqrt( Math.pow((inputRed-1398), 2) + Math.pow((inputGreen-831), 2) + Math.pow((inputBlue-482), 2) );
        BI = Math.sqrt( Math.pow((inputRed-285), 2) + Math.pow((inputGreen-650), 2) + Math.pow((inputBlue-1333), 2) );

        Cr = 1 - (RI/(RI+BI));
        Cb = 1 - (BI/(BI+RI));

        if(Cr > Cb){
            allianceColor = "red";
        }
        else{
            allianceColor = "blue";
        }



    }
}

