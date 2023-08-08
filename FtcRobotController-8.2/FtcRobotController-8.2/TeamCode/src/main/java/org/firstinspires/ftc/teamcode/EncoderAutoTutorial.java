package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderAutoTutorial extends LinearOpMode {

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private ElapsedTime runtime;
    private Servo claw;

    private final double TICKS_PER_REV = 537.7;
    private final double GEAR_RATIO = 1.0 / 1.0;
    private final double WHEEL_CIRCUMFERENCE = (96 / 25.4) * Math.PI;


    @Override
    public void runOpMode() {

        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        claw = hardwareMap.get(Servo.class, "claw");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        runtime = new ElapsedTime();

        runtime.reset();

        waitForStart();
        if(opModeIsActive() && !isStopRequested()) {
            encoderStrafeRight(0.5, 24, 2.0);
            claw.setPosition(0.82);
            encoderForward(0.5, 24, 2.0);
        }
    }

    /*
    public double ticksToInches(double ticks) {
        return ticks * (1.0 / TICKS_PER_REV) * GEAR_RATIO * WHEEL_CIRCUMFERENCE;
    } */

    public double inchesToTicks(double inches) {
        return (inches * TICKS_PER_REV) / (GEAR_RATIO * WHEEL_CIRCUMFERENCE);
    }

    public void encoderStrafeRight(double speed, double distance, double timeout) {
        int fLTarget;
        int fRTarget;
        int bLTarget;
        int bRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            fLTarget = (int)inchesToTicks(distance);
            fRTarget = -(int)inchesToTicks(distance);
            bLTarget = -(int)inchesToTicks(distance);
            bRTarget = (int)inchesToTicks(distance);
            front_left.setTargetPosition(fLTarget);
            front_right.setTargetPosition(fRTarget);
            back_left.setTargetPosition(bLTarget);
            back_right.setTargetPosition(bRTarget);

            // Turn On RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            back_left.setPower(Math.abs(speed));
            back_right.setPower(Math.abs(speed));
            front_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));

            //timeout checker
            while(opModeIsActive() &&  (runtime.seconds() < timeout) && front_left.isBusy() && front_right.isBusy()
                    && back_left.isBusy() && back_right.isBusy()) {
                //use telemetry to display
                telemetry.addData("front_left: ", front_left.getCurrentPosition());
                telemetry.addData("front_right: ", front_right.getCurrentPosition());
                telemetry.addData("back_left: ", back_left.getCurrentPosition());
                telemetry.addData("back_right: ", back_right.getCurrentPosition());
                telemetry.update();
            }

            //stop motion
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);

            //turn off RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);

        }
    }

    public void encoderForward(double speed, double distance, double timeout) {
        int fLTarget;
        int fRTarget;
        int bLTarget;
        int bRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            fLTarget = (int)inchesToTicks(distance);
            fRTarget = (int)inchesToTicks(distance);
            bLTarget = (int)inchesToTicks(distance);
            bRTarget = (int)inchesToTicks(distance);
            front_left.setTargetPosition(fLTarget);
            front_right.setTargetPosition(fRTarget);
            back_left.setTargetPosition(bLTarget);
            back_right.setTargetPosition(bRTarget);

            // Turn On RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            back_left.setPower(Math.abs(speed));
            back_right.setPower(Math.abs(speed));
            front_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));

            //timeout checker
            while(opModeIsActive() &&  (runtime.seconds() < timeout) && front_left.isBusy() && front_right.isBusy()
                    && back_left.isBusy() && back_right.isBusy()) {
                //use telemetry to display
                telemetry.addData("front_left: ", front_left.getCurrentPosition());
                telemetry.addData("front_right: ", front_right.getCurrentPosition());
                telemetry.addData("back_left: ", back_left.getCurrentPosition());
                telemetry.addData("back_right: ", back_right.getCurrentPosition());
                telemetry.update();
            }

            //stop motion
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);

            //turn off RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);

        }
    }

}
