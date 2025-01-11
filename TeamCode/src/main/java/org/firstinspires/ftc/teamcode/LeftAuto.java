package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.GestureDetector;
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
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Objects;

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

    double curAngle;

    YawPitchRollAngles orientation;
    IMU imu;

    // COLOR SENSOR SECTION GUYS!!!!!

    ColorSensor teamColorSensor = null;
    ColorSensor sampleColorSensor = null;

    String allianceColor;

    double redR = 189.0;
    double redG = 119.5;
    double redB = 67;

    double blueR = 54.5;
    double blueG = 94.5;
    double blueB = 130.5;

    double yellowR = 239.5;
    double yellowG = 305.5;
    double yellowB = 92.7;

    double tileR;
    double tileG;
    double tileB;

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

        teamColorSensor = hardwareMap.get(ColorSensor.class, "team_color_sensor");
        sampleColorSensor = hardwareMap.get(ColorSensor.class, "sample_color_sensor");

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
        curAngle = 0;

        resetTicks();
        setNormalDrive();  // Sets all motors to correct forward/reverse
        telemIMUOrientation(orientation);

        teamColor();

        telemetry.addData("CurAngle", curAngle);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Team", allianceColor);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetTicks();

        while (opModeIsActive()) {


            // Turn Testing
            //doesnt turn left and right back to back
            //curAngle = turnLeft(-0.3, 6, 90, orientation, curAngle);

            curAngle = turnRight(-0.3, 3, 90, orientation, curAngle);
            curAngle = turnRight(-0.3, 3, 90, orientation, curAngle);

            //curAngle = turnLeft(-0.3, 10, 270, orientation, curAngle);
            //curAngle = turnRight(-0.3, 10, 359, orientation,curAngle);
            //curAngle = turnLeft(-0.3, 10, 180, orientation, curAngle);  // 360 and 0 degrees don't work with IMU
            //curAngle = turnRight(-0.3, 10, 45, orientation, curAngle);

            telemIMUOrientation(orientation);



            /*
            // MEET 3 CODE
            closeClaw();

            // place starting sample in basket
            int startingSlidePos = leftArmSlidesM.getCurrentPosition();
            slideTarget = 3500;  // placeholder value
            ascendSlides(slideTarget);
            openClaw();
            sleep(500);
            descendSlides(startingSlidePos);

            axelDown(900);

            telemetry.addData("Axel", armAxelM.getCurrentPosition());
            telemetry.update();

            getTileColors();

            localTargetTick = InchesToTicks(tileMatLength*0.5);
            strafeRight(localTargetTick, -0.4, 1);
            curAngle = turnLeft(-0.4, 1, 270, orientation, curAngle);

            localTargetTick = InchesToTicks(tileMatLength);
            while(true){
                strafeLeft(localTargetTick, -0.4, 1);
                if (!(Objects.equals(sampleColors(), "empty"))){
                    break;
                }
                else{
                    localTargetTick = localTargetTick/2;
                }

                strafeRight(localTargetTick, -0.4, 1);
                if (!(Objects.equals(sampleColors(), "empty"))){
                    break;
                }
                else{
                    localTargetTick = localTargetTick/2;
                }
            }
            curAngle = turnLeft(-0.4, 1, 90, orientation, curAngle);
            localTargetTick = InchesToTicks(tileMatLength*0.5);
            strafeLeft(localTargetTick, -0.4, 1);

             */


        }
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

    public String sampleColors(){
        int inputRed = sampleColorSensor.red();
        int inputGreen = sampleColorSensor.green();
        int inputBlue = sampleColorSensor.blue();

        // Red/Blue/Yellow/Empty Input
        double RI;
        double BI;
        double YI;
        double EI;

        // Cry: C = confidence. Red to Yellow Ratio
        // R = red, Y = Yellow, G = Green, E = empty (when the claw has nothing)
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

        // Final Confidence
        double Cr;
        double Cb;
        double Cy;
        double Ce;

        String finalColor;

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

        telemetry.addData("Confidence Red", Cr);
        telemetry.addData("Confidence Blue", Cb);
        telemetry.addData("Confidence Yellow", Cy);
        telemetry.addData("Confidence Empty", Ce);
        telemetry.update();

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

        while (getCenterTicks() > -targetTicks){
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
        double yaw = orientation.getYaw();
        double targetAngle = ((curAngle + angle) % 360);

        telemetry.addData("Target", targetAngle);
        telemetry.addData("leftYaw C", leftYawConversion(yaw));
        telemetry.addData("Yaw", yaw);
        telemetry.update();

        setRightPower(power);
        setLeftPower(-power);

        while(leftYawConversion(yaw) <= targetAngle){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();
            telemetry.addData("Target", targetAngle);
            telemetry.addData("leftYaw C", leftYawConversion(yaw));
            telemetry.update();
            //telemIMUOrientation(orientation, yaw);
        }

        stopAllPower();
        resetTicks();

        //telemIMUOrientation(orientation, yaw);

        sleep(1000*sleep);

        return ((curAngle + yaw) % 360);
    }

    public double turnRight(double power, long sleep, double targetAngle, YawPitchRollAngles orientation, double curAngle){
        double yaw = orientation.getYaw();
        targetAngle = -targetAngle;
        double newTarget = targetAngle - curAngle;

        setLeftPower(power);
        setRightPower(-power);

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Target", targetAngle);
        telemetry.addData("New Target", newTarget);
        telemetry.update();

        while(yaw > newTarget){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();

            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetAngle);
            telemetry.addData("New Target", newTarget);
            telemetry.update();
            //telemIMUOrientation(orientation, yaw);
        }

        stopAllPower();
        resetTicks();

        //telemIMUOrientation(orientation, yaw);

        sleep(1000*sleep);

        return(yaw);
    }

    public void openClaw(){
        clawS.setPosition(1.0);
    }

    public void closeClaw(){
        clawS.setPosition(0);
    }

    public void ascendSlides(double target){
        while ((leftArmSlidesM.getCurrentPosition() + 100) <= target) {
            leftArmSlidesM.setPower(0.8);
            rightArmSlidesM.setPower(-0.8);
        }
        stopSlides();
    }

    public void descendSlides(double target) {
        while ((leftArmSlidesM.getCurrentPosition() - 100) >= target){
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
        while ((armAxelM.getCurrentPosition() - 80) > target) {
            armAxelM.setPower(-0.8);
        }
        stopAxel();
    }

    public void axelDown(double target){
        while ((armAxelM.getCurrentPosition() + 80) < target) {
            armAxelM.setPower(0.8);
        }
        stopAxel();
    }

    public void stopAxel(){
        armAxelM.setPower(0.0);
    }



    public void setNormalDrive(){
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setStrafingDrive(){
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void telemIMUOrientation(YawPitchRollAngles orientation){
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.update();
    }

    public double rightYawConversion(double yaw){
        if (yaw < 0){
            return (Math.abs(yaw));
        }
        else{
            return (360 - yaw);
        }
    }

    public double leftYawConversion(double yaw){
        if (yaw < 0){
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