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

    YawPitchRollAngles orientation;
    IMU imu;

    // COLOR SENSOR SECTION GUYS!!!!!

    View relativeLayout;
    NormalizedColorSensor teamColorSensor = null;
    NormalizedColorSensor sampleColorSensor = null;
    String allianceColor;

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

        // M = Motor | S = servo
        leftArmSlidesM = hardwareMap.get(DcMotor.class, "left_arm_slides");
        rightArmSlidesM = hardwareMap.get(DcMotor.class, "right_arm_slides");
        armAxelM = hardwareMap.get(DcMotor.class, "arm_axel");
        clawS = hardwareMap.get(Servo.class, "claw");

        teamColorSensor = hardwareMap.get(NormalizedColorSensor.class, "team_color_sensor");
        sampleColorSensor = hardwareMap.get(NormalizedColorSensor.class, "sample_color_sensor");

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
        telemIMUOrientation(orientation, yaw);

        // sets up color sensor
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
    }

    protected void runSample() {
        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).
        float gain = 2;

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        final float[] hsvValues = new float[3];

        // Team Alliance Color
        if (teamColorSensor instanceof SwitchableLight) {
            ((SwitchableLight)teamColorSensor).enableLight(true);
        }
        NormalizedRGBA teamColors = teamColorSensor.getNormalizedColors();

        // Team Sample Color
        if (sampleColorSensor instanceof SwitchableLight) {
            ((SwitchableLight)sampleColorSensor).enableLight(true);
        }

        Color.colorToHSV(teamColors.toColor(), hsvValues);

        if (teamColors.blue > teamColors.red){
            allianceColor = "blue";
        }
        else{
            allianceColor = "red";
        }

        telemetry.addData("Alliance Color", allianceColor);
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetTicks();

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
            float starting_pos = leftEncoderMotor.getCurrentPosition();
            closeClaw();

            localTargetTick = InchesToTicks(tileMatLength*0.5);
            driveForward(localTargetTick, -0.4, 1);

            // place starting sample in basket
            slideTarget = 3700;  // placeholder value
            ascendSlides(slideTarget);
            sleep(1000);
            openClaw();
            sleep(600);
            driveBackward(localTargetTick, -0.4, 1);

            descendSlides(starting_pos);  // we want to descend the slides the same amount we ascend them

            // Go to ascent zone to park
            localTargetTick = InchesToTicks(tileMatLength*2.3);
            strafeRight(localTargetTick, -0.5, 1);

            curAngle = turnLeft(-0.3, 2, 180, orientation, curAngle);

            localTargetTick = InchesToTicks(tileMatLength*0.2);
            driveForward(localTargetTick, -0.4, 1);

            axelDown(5000);

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
        clawS.setPosition(1.0);
    }

    public void closeClaw(){
        clawS.setPosition(0);
    }

    public void ascendSlides(double target){
        while ((leftArmSlidesM.getCurrentPosition() + 200) <= target) {
            leftArmSlidesM.setPower(0.8);
            rightArmSlidesM.setPower(-0.8);
        }
        stopSlides();
    }

    public void descendSlides(double target) {
        while ((leftArmSlidesM.getCurrentPosition() - 200) >= target){
            leftArmSlidesM.setPower(-0.8);
            rightArmSlidesM.setPower(0.8);
        }
       stopSlides();
    }

    public void stopSlides(){
        leftArmSlidesM.setPower(0.0);
        rightArmSlidesM.setPower(0.0);
    }

    public void axelUp(double target){
        while ((armAxelM.getCurrentPosition() - 100) > target) {
            armAxelM.setPower(-0.8);
        }
        stopAxel();
    }

    public void axelDown(double target){
        while ((armAxelM.getCurrentPosition() + 100) < target) {
            armAxelM.setPower(0.8);
        }
        stopAxel();
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
    public double InchesToTicks(double inches) {
        double rev = inches / OPcircumference;
        double tick = 2000 * rev;
        return tick;
    }
}