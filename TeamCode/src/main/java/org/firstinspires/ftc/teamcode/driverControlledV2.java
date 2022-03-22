// MAY NEED TO RECONFIGURE CODE (left stick on gamepad2 is arm movement and left and right trigger is intake)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class driverControlledV2 extends LinearOpMode {
    //this compiles on AS so use this exact process when defining motor objects
    /*DcMotor motorLF = hardwareMap.dcMotor.get("motorFrontLeft");
    DcMotor motorLB = hardwareMap.dcMotor.get("motorBackLeft");
    DcMotor motorRF = hardwareMap.dcMotor.get("motorFrontRight");
    DcMotor motorRB = hardwareMap.dcMotor.get("MotorBackRight"); */

    //this don't compile on AS when it works on onBot...

    @Override
    public void runOpMode() {
        DcMotor motorLF = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorLB = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorRF = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorRB = hardwareMap.get(DcMotor.class, "motorBackRight");

        DcMotor Arm = hardwareMap.get(DcMotor.class, "arm");
        DcMotor Turret = hardwareMap.get(DcMotor.class, "Turret");
        DcMotor Carousel = hardwareMap.get(DcMotor.class, "Carousel");
        DcMotor Intake = hardwareMap.get(DcMotor.class, "poggy");
        telemetry.addData("POGGY", " NO CRASH");
        telemetry.update();


        // Servo shipping = hardwareMap.get(Servo.class, "ship");

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {
            double y, x, rx;

            // carousel
            if (gamepad2.right_bumper && gamepad2.left_bumper) {
                Carousel.setPower(-0.65/2);
            }
            else if (gamepad2.right_bumper) {
                Carousel.setPower(-0.65);
            }
            else {
                Carousel.setPower(0);
            }

            // intake
            Intake.setPower(gamepad2.right_stick_y);

            // arm
            Arm.setPower(gamepad2.left_stick_y);

            // halve speed
            if (gamepad1.right_bumper) {
                y = -gamepad1.left_stick_y / 2;
                x = (gamepad1.left_stick_x * 1.1) / 2;
                rx = gamepad1.right_stick_x / 2;
            }

            // move speed
            else {
                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x * 1.1;
                rx = gamepad1.right_stick_x;
            }

            // dpad to move forward
            if (gamepad1.dpad_up) {
                motorLF.setPower(1);
                motorLB.setPower(1);
                motorRF.setPower(1);
                motorRB.setPower(1);
            }

            // dpad to move backward
            if (gamepad1.dpad_down) {
                motorLF.setPower(-1);
                motorLB.setPower(-1);
                motorRF.setPower(-1);
                motorRB.setPower(-1);
            }

            // dpad to strafe left
            if (gamepad1.dpad_left) {
                motorLF.setPower(-1);
                motorLB.setPower(1);
                motorRF.setPower(1);
                motorRB.setPower(-1);
            }

            // dpad to strafe right
            if (gamepad1.dpad_right) {
                motorLF.setPower(1);
                motorLB.setPower(-1);
                motorRF.setPower(-1);
                motorRB.setPower(1);
            }

            // turret rotation
            /*
            if (gamepad2.left_trigger>0) Turret.setPower(gamepad2.left_trigger);
            if (gamepad2.right_trigger>0) Turret.setPower(-gamepad2.right_trigger);
            else Turret.setPower(0);
             */

            if (Math.abs(gamepad2.left_stick_x)>0.2) Turret.setPower(-gamepad2.left_stick_x);
            else if (gamepad2.left_stick_x==0) Turret.setPower(0);

            // driving via joystick
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;

            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorLF.setPower(frontLeftPower);
            motorLB.setPower(backLeftPower);
            motorRF.setPower(frontRightPower);
            motorRB.setPower(backRightPower);

            telemetry.addData("LF Power:", motorLF.getPower());
            telemetry.addData("LB Power:", motorLB.getPower());
            telemetry.addData("RF Power:", motorRF.getPower());
            telemetry.addData("RB Power:", motorRB.getPower());
            telemetry.addData( "\nIntake Power:", Intake.getPower());
            telemetry.addData( "Arm Power: ", Arm.getPower());
            telemetry.addData("Carousel Turner", Carousel.getPower());
            telemetry.update();
        }
    }
}

