package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Odometry Pods", group="Linear OpMode")

public class OdometryPods extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;

    private DcMotor leftEncoderMotor = null;
    private int leftEncoderPos = 0;
    private double deltaLeftEncoder = 0;

    private DcMotor rightEncoderMotor = null;
    private int rightEncoderPos = 0;
    private double deltaRightEncoder = 0;

    private DcMotor centerEncoderMotor = null;
    private int centerEncoderPos = 0;
    private double deltaCenterEncoder = 0;

    private double theta = 0;

    private boolean rightStop = false;
    private boolean leftStop = false;

    // Calculates the circumference for the Odometry pods
    // Divides by 25.4 to change mm to inches
    double OPcircumference = 2*Math.PI*(16/25.4);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FL = hardwareMap.get(DcMotor.class, "left_front_drive");
        BL = hardwareMap.get(DcMotor.class, "left_back_drive");
        FR = hardwareMap.get(DcMotor.class, "right_front_drive");
        BR = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftEncoderMotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightEncoderMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        centerEncoderMotor = hardwareMap.get(DcMotor.class, "left_back_drive");

        leftEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);  // Directions taken from BlackBoxBot.java
        rightEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        centerEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetTicks();

        int targetInches = 12*10;
        double targetTicks = InchesToPulse(targetInches);

        setLeftPower(-0.5);
        setRightPower(-0.5);

        while (!(rightStop && leftStop)){
            if (getRightTicks() >= targetTicks){
                rightStop = true;
                stopRightPower();
            }
            if (getLeftTicks() >= targetTicks){
                leftStop = true;
                stopLeftPower();
            }
        }
        telemetry.addData("Target Tick", targetTicks);
        telemetry.addData("Left pos", getLeftTicks());
        telemetry.addData("Right pos", getLeftTicks());
        telemetry.update();

        sleep(10000);
    }
    public void resetTicks(){
        resetLeftTicks();
        resetRightTicks();
        resetCenterTicks();
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

    public int getLeftTicks(){
        return (leftEncoderMotor.getCurrentPosition() - leftEncoderPos)*-1;
    }

    public void resetRightTicks(){
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }

    public int getRightTicks(){
        return rightEncoderMotor.getCurrentPosition() - rightEncoderPos;
    }

    public void resetCenterTicks(){
        centerEncoderPos = centerEncoderMotor.getCurrentPosition();
    }

    public int getCenterTicks(){
        return (centerEncoderMotor.getCurrentPosition() - centerEncoderPos)*-1;
    }

    public double PulseToInches(int pulses){
        double rev = (double)pulses/2000;
        double inches = OPcircumference * rev;
        return inches;
    }
    public double InchesToPulse(int inches){
        double rev = inches/OPcircumference;
        double pulses = 2000*rev;
        return pulses;
    }
}
