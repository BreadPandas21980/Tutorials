package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class SimpleTeleOpTutorial extends LinearOpMode {

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor slide_left;
    private DcMotor slide_right;
    private Servo claw;
    private CRServo arm_left;
    private CRServo arm_right;

    final double OPEN_CLAW = 0.82;
    final double CLOSED_CLAW = 1.0;

    @Override
    public void runOpMode() {

        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        slide_left = hardwareMap.get(DcMotor.class, "slide_left");
        slide_right = hardwareMap.get(DcMotor.class, "slide_right");
        claw = hardwareMap.get(Servo.class, "claw");
        arm_left = hardwareMap.get(CRServo.class, "arm_left");
        arm_right = hardwareMap.get(CRServo.class, "arm_right");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        arm_right.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_left.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            double arms = -gamepad2.left_stick_y;
            double slides = -gamepad2.right_stick_y;

            double fLPower = forward + rotate + strafe;
            double fRPower = forward - rotate - strafe;
            double bLPower = forward + rotate - strafe;
            double bRPower = forward - rotate + strafe;

            setDrivePowers(fLPower, fRPower, bLPower, bRPower);

            //claw code
            if(gamepad2.left_trigger > 0.1 || gamepad2.b) {
                claw.setPosition(OPEN_CLAW);
            } else if (gamepad2.right_trigger > 0.1 || gamepad2.x) {
                claw.setPosition(CLOSED_CLAW);
            }

            //arm code
            if(arms > 0.2) {
                arm_left.setPower(1);
                arm_right.setPower(1);
            } else if (arms < -0.2) {
                arm_left.setPower(-1);
                arm_right.setPower(-1);
            } else {
                arm_left.setPower(0);
                arm_right.setPower(0);
            }

            //slide code
            if (slides > 0.2) {
                slide_left.setPower(0.9);
                slide_right.setPower(0.9);
            } else if (slides < -0.2) {
                slide_left.setPower(-0.775);
                slide_right.setPower(-0.775);
            } else {
                slide_left.setPower(0);
                slide_right.setPower(0);
            }

            telemetry.addData("clawPos: ", claw.getPosition());
            telemetry.addData("arms: ", arm_left.getPower());
            telemetry.addData("slides: ", slide_left.getPower());
            telemetry.update();
        }
    }

    private void setDrivePowers(double fLPower, double fRPower, double bLPower, double bRPower){

        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(fLPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(fRPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(bLPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(bRPower));

        fLPower /= maxSpeed;
        fRPower /= maxSpeed;
        bLPower /= maxSpeed;
        bRPower /= maxSpeed;

        front_left.setPower(fLPower);
        front_right.setPower(fRPower);
        back_left.setPower(bLPower);
        back_right.setPower(bRPower);

        telemetry.addData("fL: ", fLPower);
        telemetry.addData("fR: ", fLPower);
        telemetry.addData("bL: ", fLPower);
        telemetry.addData("bR: ", fLPower);

    }
}
