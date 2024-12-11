package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="LeftAuto", group="Linear OpMode")

public class LeftAuto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;

    private DcMotor leftArmSlidesM = null;
    private DcMotor rightArmSlidesM = null;
    private DcMotor armAxelM = null;

    NormalizedColorSensor teamColorSensor = null;
    Servo clawS;
    private DcMotor leftEncoderMotor = null;
    private double leftEncoderPos = 0.0;
    private double deltaLeftEncoder = 0.0;

    private DcMotor rightEncoderMotor = null;
    private double rightEncoderPos = 0.0;
    private double deltaRightEncoder = 0.0;

    private DcMotor centerEncoderMotor = null;
    private double centerEncoderPos = 0;
    private double deltaCenterEncoder = 0;

    private boolean rightStop = false;
    private boolean leftStop = false;

    private double tileMatLength = 12*2;  // Inches

    double localTargetTick;
    double slideTarget;
    // Calculates the circumference for the Odometry pods
    // Divides by 25.4 to change mm to inches
    double OPcircumference = 2.0*Math.PI*(16.0/25.4);

    double yaw;

    double curAngle = 0;
    IMU imu;

    // COLOR SENSOR SECTION GUYS!!!!!

    View relativeLayout;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FL = hardwareMap.get(DcMotor.class, "left_front_drive");
        BL = hardwareMap.get(DcMotor.class, "left_back_drive");
        FR = hardwareMap.get(DcMotor.class, "right_front_drive");
        BR = hardwareMap.get(DcMotor.class, "right_back_drive");

        centerEncoderMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftEncoderMotor = hardwareMap.get(DcMotor.class, "right_back_drive");
        rightEncoderMotor = hardwareMap.get(DcMotor.class, "left_front_drive");

        leftArmSlidesM = hardwareMap.get(DcMotor.class, "left_arm_slides");
        rightArmSlidesM = hardwareMap.get(DcMotor.class, "right_arm_slides");
        armAxelM = hardwareMap.get(DcMotor.class, "arm_axel");
        clawS = hardwareMap.get(Servo.class, "claw");
        teamColorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        imu = hardwareMap.get(IMU.class, "imu");

        leftEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);  // Directions taken from BlackBoxBot.java
        rightEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        centerEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw();

        resetTicks();
        setNormalDrive();  // Sets all motors to correct forward/reverse

        // Send telemetry message to signify robot waiting;
        telemIMUOrientation(orientation, yaw);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetTicks();

        // COLOR SENSOR SECTION GUYS!!!!!

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        try {
            runSample(); // actually execute the sample
        } finally {
            // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
            // as pure white, but it's too much work to dig out what actually was used, and this is good
            // enough to at least make the screen reasonable again.
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }


        while (opModeIsActive()) {

            /*
            // Turn Testing
            //doesnt turn left and right back to back
            curAngle = turnLeft(-0.3, 2, 90, orientation, curAngle);
            curAngle = turnRight(-0.3, 2, 180, orientation, curAngle);
            curAngle = turnLeft(-0.3, 2, 270, orientation, curAngle);
            curAngle = turnRight(-0.3, 2, 359, orientation,curAngle);
            curAngle = turnLeft(-0.3, 2, 180, orientation, curAngle);  // 360 and 0 degrees don't work with IMU
            curAngle = turnRight(-0.3, 2, 45, orientation, curAngle);

            telemIMUOrientation(orientation, yaw);

             */


        // MEET 2 CODE
            localTargetTick = InchesToTicks(tileMatLength);
            driveForward(localTargetTick, -0.4, 1);  // Drives up to rung

            // place starting specimen
            slideTarget = 4000;  // placeholder value
            ascendSlides(slideTarget);
            sleep(1000);
            descendSlides(slideTarget);  // we want to descend the slides the same amount we ascend them
            openClaw();

            // repeat this n times or encase in while loop
            axelDown(200);  // also placeholder value. We might need to calculate a circle arc as we dont have an angle...
            // begin sample search algorithm
            axelUp(200);

            // go to observation zone
            localTargetTick = InchesToTicks(tileMatLength*0.5);
            driveBackward(localTargetTick, -0.4, 1);
            localTargetTick = InchesToTicks(tileMatLength*2);
            strafeRight(localTargetTick, -0.4, 1);
            // turn 180
            openClaw();

            closeClaw();
            // turn 180
            strafeLeft(localTargetTick, -0.4, 1);
            localTargetTick = InchesToTicks(tileMatLength*0.5);
            driveForward(localTargetTick, -0.4, 1);

            // encase in while loop




            /*

            ** using n as i dont know how many we can cycle **

            cycle 5 samples to obs zone
            hang all 5 specimens on high rung

            maybe check time every hang and park if time is low?
            or continue cycling and not care
             */


        /* Meet 0 Code
        localTargetTick = InchesToTicks(tileMatLength*0.12);
        strafeRight(localTargetTick, -0.4, 1);

        localTargetTick = (InchesToTicks(tileMatLength*0.8));
        driveForward(localTargetTick, -0.5, 1);

        localTargetTick = (InchesToTicks(tileMatLength*0.8));
        driveBackward(localTargetTick, -0.5, 1);

        localTargetTick = InchesToTicks(tileMatLength*0.8);
        strafeRight(localTargetTick, -0.4, 1);

        localTargetTick = (InchesToTicks(tileMatLength*3.5));
        driveBackward(localTargetTick, -0.5, 1);

        localTargetTick = InchesToTicks(tileMatLength*0.6);
        strafeLeft(localTargetTick, -0.4, 1);
         */


            telemAllTicks("None");
            break;
        }
    }

    public void driveForward(double targetTicks, double power, long sleep) {
        resetTicks();
        setAllPower(power);
        telemAllTicks("Forward");

        while (!(rightStop && leftStop)) {
            if (getRightTicks() >= targetTicks) {
                rightStop = true;
                stopRightPower();
            }
            if (getLeftTicks() >= targetTicks) {
                leftStop = true;
                stopLeftPower();
            }
            telemAllTicks("Forward");
        }

        telemAllTicks("Forward");

        rightStop = false;
        leftStop = false;

        stopAllPower();
        resetTicks();

        sleep(500*sleep);
    }

    public void driveBackward(double targetTicks, double power, long sleep) {
        resetTicks();
        setAllPower(-power);

        telemAllTicks("Backward");

        while (!(rightStop && leftStop)) {
            if (Math.abs(getRightTicks()) >= targetTicks) {
                rightStop = true;
                stopRightPower();
            }
            if (Math.abs(getLeftTicks()) >= targetTicks) {
                leftStop = true;
                stopLeftPower();
            }
            telemAllTicks("Backward");
        }

        telemAllTicks("Backward");

        rightStop = false;
        leftStop = false;
        stopAllPower();
        resetTicks();

        sleep(500*sleep);
    }

    public void strafeRight(double targetTicks, double power, long sleep) {
        setStrafingDrive();
        resetTicks();
        setAllPower(power);

        telemAllTicks("Right");

        while (getCenterTicks() < targetTicks){
            telemAllTicks("Right");
        }

        telemAllTicks("Right");

        stopAllPower();
        resetTicks();
        setNormalDrive();

        sleep(500*sleep);
    }

    public void strafeLeft(double targetTicks, double power, long sleep) {
        setStrafingDrive();
        resetTicks();
        setAllPower(-power);

        telemAllTicks("Left");

        while (Math.abs(getCenterTicks()) < targetTicks){
            telemAllTicks("Left");
        }

        telemAllTicks("Left");

        stopAllPower();
        resetTicks();
        setNormalDrive();

        sleep(500*sleep);
    }


    // Turns in one direction but doesn't turn back to back
    public double turnLeft(double power, long sleep, double angle, YawPitchRollAngles orientation, double curAngle){
        setRightPower(power);
        setLeftPower(-power);

        double yaw = orientation.getYaw();
        double targetAngle = curAngle + angle;

        while(leftYawConversion(yaw) <= targetAngle){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();
            telemIMUOrientation(orientation, yaw);
        }

        stopAllPower();
        resetTicks();

        telemIMUOrientation(orientation, yaw);

        sleep(500*sleep);

        return ((curAngle+yaw)%360);
    }

    public double turnRight(double power, long sleep, double angle, YawPitchRollAngles orientation, double curAngle){
        setLeftPower(power);
        setRightPower(-power);

        double yaw = orientation.getYaw();
        double targetAngle = curAngle - angle;

        while(rightYawConversion(yaw) <= targetAngle){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();
            telemIMUOrientation(orientation, yaw);
        }

        stopAllPower();
        resetTicks();

        telemIMUOrientation(orientation, yaw);

        sleep(500*sleep);

        return((curAngle + yaw) % 360);
    }

    public void openClaw(){
        clawS.setPosition(0.0);
    }

    public void closeClaw(){
        clawS.setPosition(0.6);
    }

    public void ascendSlides(double target){
        while (leftArmSlidesM.getCurrentPosition() < target) {
            leftArmSlidesM.setPower(0.8);
            rightArmSlidesM.setPower(-0.8);
        }
        stopSlides();
    }

    public void descendSlides(double target) {
        while (leftArmSlidesM.getCurrentPosition() > target){
            leftArmSlidesM.setPower(-0.8);
            rightArmSlidesM.setPower(0.8);
        }
       stopSlides();
    }

    public void stopSlides(){
        leftArmSlidesM.setPower(0.0);
        rightArmSlidesM.setPower(0.0);
    }

    public void axelDown(double target){
        while (armAxelM.getCurrentPosition() > target) {
            armAxelM.setPower(-0.8);
        }
    }

    public void axelUp(double target){
        while (armAxelM.getCurrentPosition() < target) {
            armAxelM.setPower(0.8);
        }
    }

    public void stopAxel(){
        armAxelM.setPower(0.0);
    }



    public void setNormalDrive(){
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setStrafingDrive(){
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public void resetTicks(){
        resetLeftTicks();
        resetRightTicks();
        resetCenterTicks();
    }

    public void setAllPower(double p){
        FL.setPower(p);
        FR.setPower(p);
        BL.setPower(p);
        BR.setPower(p);
    }

    public void stopAllPower(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void setLeftPower(double p){
        FL.setPower(p);
        BL.setPower(p);
    }

    public void setRightPower(double p){
        FR.setPower(p);
        BR.setPower(p);
    }

    public void stopLeftPower(){
        FL.setPower(0);
        BL.setPower(0);
    }

    public void stopRightPower(){
        FR.setPower(0);
        BR.setPower(0);
    }

    public void resetLeftTicks(){
        leftEncoderPos = leftEncoderMotor.getCurrentPosition();
    }

    public double getLeftTicks(){
        return (-(leftEncoderMotor.getCurrentPosition() - leftEncoderPos));
    }

    public void resetRightTicks(){
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }

    public double getRightTicks(){
        return (rightEncoderMotor.getCurrentPosition() - rightEncoderPos);
    }

    public void resetCenterTicks(){
        centerEncoderPos = centerEncoderMotor.getCurrentPosition();
    }

    public double getCenterTicks(){
        return (centerEncoderMotor.getCurrentPosition() - centerEncoderPos);
    }

    public void telemAllTicks(String dir){
        telemetry.addData("Direction", dir);
        telemetry.addData("Left pos", getLeftTicks());
        telemetry.addData("Right pos", getRightTicks());
        telemetry.addData("Center pos", getCenterTicks());
        telemetry.update();
    }

    public void telemIMUOrientation(YawPitchRollAngles orientation, double yaw){
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Var Yaw", yaw);
        telemetry.update();
    }

    public double rightYawConversion(double yaw){
        if (yaw <= -1){
            return (Math.abs(yaw));
        }
        else{
            return (360 - yaw);
        }
    }

    public double leftYawConversion(double yaw){
        if (yaw <= -1){
            return (yaw+360);
        }
        else{
            return yaw;
        }
    }

    public double TicksToInches(double ticks){
        double rev = (double)ticks/2000;
        double inches = OPcircumference * rev;
        return inches;
    }
    public double InchesToTicks(double inches){
        double rev = inches/OPcircumference;
        double tick = 2000*rev;
        return tick;
    }


    // MORE COLOR SENSOR LOGIC!!!!!

    public void runSample() {
        // Use a higher value of gain in dark situations, but lower in high. >= 1, lower values are safer
        float gain = 2;

        // Once per loop, we will update this hsvValues array. Element 0 is H, 1 is S, 2 is V.
        final float[] hsvValues = new float[3];

        /*
        // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
        // state of the X button on the gamepad
        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;
         */

        // Map sensor. Normalized is used to ensure values between 0 and 1.
        teamColorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (teamColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) teamColorSensor).enableLight(true);
        }

        // Wait for the start button to be pressed.
        waitForStart();

        // Loop until we are asked to stop
        while (opModeIsActive()) {
            // Explain basic gain information via telemetry
            // telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");
            telemetry.addData("Gain: ", gain); // Show the gain value via telemetry

            // Get the normalized colors from the sensor
            NormalizedRGBA teamColor = teamColorSensor.getNormalizedColors();

            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(teamColor.toColor(), hsvValues);

            telemetry.addLine()
                    .addData("Red", "%.3f", teamColor.red)
                    .addData("Green", "%.3f", teamColor.green)
                    .addData("Blue", "%.3f", teamColor.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", teamColor.alpha);

            /* If this color sensor also has a distance sensor, display the measured distance.
             * Note that the reported distance is only useful at very close range, and is impacted by
             * ambient light and surface reflectivity. */
            if (teamColorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) teamColorSensor).getDistance(DistanceUnit.CM));
            }

            telemetry.update();

            // Change the Robot Controller's background color to match the color detected by the color sensor.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
                }
            });
        }
    }
}