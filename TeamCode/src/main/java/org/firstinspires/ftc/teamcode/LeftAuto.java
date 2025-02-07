package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    final double ptTileMat = Math.sqrt( (Math.pow(tileMatLength, 2)) + Math.pow(tileMatLength,2) );

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
        imu.resetYaw();
        //curAngle = 0;

        resetTicks();
        setNormalDrive();  // Sets all motors to correct forward/reverse
        telemIMUOrientation(orientation);

        teamColor();
        //getYellowColors();

        telemetry.addData("CurAngle", getRuntime());
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Team", allianceColor);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetTicks();

        while (opModeIsActive()) {

            /*
            ALL POWER MUST BE NEGATIVE
            Sleep is used when testing, set to 0 for max time

             */
            int startingSlidePos = leftArmSlidesM.getCurrentPosition();
            axelUp(300); // This value only works if robot is POWERED ON when slides are VERTICAL

            ascendSlides(1850);

            localTargetTick = inchesToTicks(tileMatLength*1.3);
            driveForward(localTargetTick, -0.5,1);

            descendSlidesAndAxel(startingSlidePos+100, 1000);  // aTarget = Axel Target = Placeholder

            localTargetTick = inchesToTicks(tileMatLength*0.3);
            driveBackward(localTargetTick, -0.5, 1);

            localTargetTick = inchesToTicks(tileMatLength*1.3);
            strafeLeft(localTargetTick, -0.5, 1);

            axelDown(2350);
            ascendSlides(1900);
            closeClaw();
            descendSlides(1900);
            axelUp(300);

            turnLeft(-0.5, 90, orientation);
            turnLeft(-0.5, 45, orientation);

            localTargetTick = inchesToTicks(ptTileMat);
            driveForward(ptTileMat, -0.5, 1);

            // Places first sample in high basket
            ascendSlides(3700);
            axelDown(400);
            openClaw();
            sleep(300);
            axelUp(300);
            sleep(300);  // Sleeps are for safety, can remove to speed up

            turnRight(-0.5, 90, orientation);
            turnRight(-0.5, 45, orientation);

            descendSlidesAndAxel(startingSlidePos+100, 2350);
            ascendSlides(1900);
            closeClaw();
            descendSlides(1900);
            axelUp(300);

            turnLeft(-0.5, 90, orientation);
            turnLeft(-0.5, 45, orientation);

            // Places second sample in high basket
            ascendSlides(3700);
            axelDown(400);
            openClaw();
            sleep(300);
            axelUp(300);
            sleep(300);  // Sleeps are for safety, can remove to speed up

            // Idk if we should go for the third sample or park

            /* Code for third sample

            turnRight(-0.5, 45, orientation);
            descendSlides(startingSlidePos+100);
            localTargetTick = inchesToTicks(tileMatLength*1.5);
            strafeRight(localTargetTick, -0.5, 1);

            descendSlidesAndAxel(startingSlidePos+100, 2350);
            ascendSlides(1900);
            closeClaw();
            descendSlides(1900);
            axelUp(300);

            localTargetTick = inchesToTicks(tileMatLength*1.5);
            strafeLeft(localTargetTick, -0.5, 1);

            turnLeft(-0.5, 45, orientation);

            // Places second sample in high basket
            ascendSlides(3700);
            axelDown(400);
            openClaw();
            sleep(300);
            axelUp(300);
            sleep(300);  // Sleeps are for safety, can remove to speed up
             */

            // Goes to sub
            turnRight(-0.5, 45, orientation);

            descendSlides(startingSlidePos+100);

            localTargetTick = inchesToTicks(tileMatLength*2);
            strafeRight(localTargetTick, -0.5, 1);

            turnRight(-0.5, 90, orientation);
            turnRight(-0.5, 45, orientation);

            localTargetTick = inchesToTicks(tileMatLength*1.3);
            driveForward(localTargetTick, -0.5, 1);

            axelDown(1000);
            getTileColors();
            ascendSlides(1850);

            //Search Sub CODE

            break;

            // Total Points With 2 Samples WITH PARK: 10+(8*2)+3 = 29 at end of auto -> 52 in end game
            // Total Points With 3 Samples WITH PARK: 10+(8*3)+3 = 37 at end of auto -> 68 in end game

            // Total points With 2 Samples NO PARK: 10+(8*2)+2= 28 at end of auto -> 56 in end game
            // Total points With 3 Samples NO PARK: 10+(8*3)+2= 36 at end of auto -> 72 in end game


            /* MEET 4 CODE
            closeClaw();

            // place starting sample in basket
            int startingSlidePos = leftArmSlidesM.getCurrentPosition();
            axelUp(300);

            localTargetTick = InchesToTicks(tileMatLength*0.2);
            strafeRight(localTargetTick, -0.5, 0);

            localTargetTick = InchesToTicks(tileMatLength);
            driveForward(localTargetTick, -0.5, 0);

            turnLeft(-0.3, 45, orientation);

            slideTarget = 3700;
            ascendSlides(slideTarget);

            axelDown(400);
            openClaw();
            sleep(300);
            axelUp(300);
            sleep(300);

            turnRight(-0.4, 45, orientation);
            turnRight(-0.4, 90, orientation);

            descendSlides(startingSlidePos+100);

            localTargetTick = InchesToTicks(tileMatLength*0.1);
            driveForward(localTargetTick, -0.3, 0);

            axelDown(2350);
            slideTarget = 1900;
            ascendSlides(slideTarget);
            closeClaw();
            sleep(500);
            descendSlides(startingSlidePos+300);
            axelUp(300);

            driveBackward(localTargetTick, -0.3, 0);

            turnLeft(-0.4, 90, orientation);
            turnLeft(-0.4 , 44, orientation);

            slideTarget = 3700;
            ascendSlides(slideTarget);

            axelDown(400);
            openClaw();
            sleep(300);
            axelUp(300);
            sleep(300);

            turnLeft(-0.4, 5, orientation);

            localTargetTick = InchesToTicks(tileMatLength*1.25);
            driveBackward(localTargetTick, -0.6, 1);

            descendSlides(475);

            localTargetTick = InchesToTicks(tileMatLength*1.25);
            driveBackward(localTargetTick, -0.6, 1);

            descendSlides(475);

            stopAllPower();
            break;

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
            telemetry.addData("Color", "Red");
            telemetry.update();
            return "red";
        }
        else if(Cb > Cy && Cb > Ce){
            telemetry.addData("Color", "Blue");
            telemetry.update();
            return "blue";
        }
        else if(Cy > Ce){
            telemetry.addData("Color", "Yellow");
            telemetry.update();
            return "yellow";
        }
        else{
            telemetry.addData("Color", "Empty");
            telemetry.update();
            return "empty";
        }
    }

    public void getTileColors(){
        tileR = sampleColorSensor.red();
        tileG = sampleColorSensor.green();
        tileB = sampleColorSensor.blue();
    }

    public void getYellowColors(){
        yellowR = sampleColorSensor.red();
        yellowG = sampleColorSensor.green();
        yellowB = sampleColorSensor.blue();
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
    public void turnLeft(double power, double targetAngle, YawPitchRollAngles orientation){
        double yaw = orientation.getYaw();

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Target", targetAngle);
        telemetry.update();

        setRightPower(power);
        setLeftPower(-power);

        while(yaw < targetAngle){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetAngle);
            telemetry.update();
            //telemIMUOrientation(orientation, yaw);
        }

        stopAllPower();
        resetTicks();
        imu.resetYaw();

        //telemIMUOrientation(orientation, yaw);
    }

    public void turnRight(double power, double targetAngle, YawPitchRollAngles orientation){
        double yaw = orientation.getYaw();
        targetAngle = -targetAngle;

        setLeftPower(power);
        setRightPower(-power);

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Target", targetAngle);
        telemetry.update();

        while(yaw > targetAngle){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();

            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetAngle);
            telemetry.update();
            //telemIMUOrientation(orientation, yaw);
        }

        stopAllPower();
        resetTicks();
        imu.resetYaw();

        //telemIMUOrientation(orientation, yaw);
    }

    public void openClaw(){
        clawS.setPosition(0);
    }

    public void closeClaw(){
        clawS.setPosition(1.0);
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

    public void descendSlidesAndAxel(double sTarget, double aTarget) {
        boolean slideStop = false;
        boolean axelStop = false;
        while (!(slideStop && axelStop)){
            leftArmSlidesM.setPower(-0.8);
            rightArmSlidesM.setPower(0.8);
            armAxelM.setPower(0.9);
            if (leftArmSlidesM.getCurrentPosition() == sTarget){
                slideStop = true;
                stopSlides();
            }
            if (armAxelM.getCurrentPosition() == aTarget){
                axelStop = true;
                stopAxel();
            }


        }
        stopSlides();
    }

    public void stopSlides(){
        leftArmSlidesM.setPower(0.0);
        rightArmSlidesM.setPower(0.0);
    }

    public void axelUp(double target){
        while (armAxelM.getCurrentPosition() > target) {
            armAxelM.setPower(-0.8);
        }
        stopAxel();
    }

    public void axelDown(double target){
        while (armAxelM.getCurrentPosition() < target) {
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
    public double inchesToTicks(double inches) {
        double rev = inches / OPcircumference;
        double tick = 2000 * rev;
        return tick;
    }
}