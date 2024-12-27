package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "CyberSalam2 - TeleOp")

public class CyberSalam2 extends LinearOpMode {

    private DcMotor intake;
    private DcMotor armtop;
    private DcMotor armbottom;
    private DcMotor moterLB;
    private DcMotor motorLF;
    private DcMotor motorRB;
    private DcMotor motorRF;
    private Servo servoclaw;

    int limit_reached = 0;
    float ultraPower = 0;
    double fIntake = 0;
    float fArmTop = 0;
    float bPower = 0;
    double fPower = 0;
    double sPower = 0;
    double dPower = 0;
    int armbottomposition = 0;
    int armtopposition = 0;

    @Override
    public void runOpMode() {

        moterLB = hardwareMap.get(DcMotor.class, "moterLB");
        motorLF = hardwareMap.get(DcMotor.class, "motorLF");
        motorRB = hardwareMap.get(DcMotor.class, "motorRB");
        motorRF = hardwareMap.get(DcMotor.class, "motorRF");
        armtop = hardwareMap.get(DcMotor.class, "armtop");
        armbottom = hardwareMap.get(DcMotor.class, "armbottom");
        servoclaw = hardwareMap.get(Servo.class, "servoclaw");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Makes robot brake when controller isn't giving any input.
        moterLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armtop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armbottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders.
        MOTORS_STOP_AND_RESET_ENCODERS();
        MOTORS_RUN_WITHOUT_ENCODERS();

        telemetry.addData("Status", "Initialize ... DONE!");
        telemetry.update();

        // Put initialization blocks here.
        servoclaw.setDirection(Servo.Direction.REVERSE);
        servoclaw.setPosition(0.0);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Put loop blocks here.

                if (0 < gamepad2.left_stick_y) {
                    fIntake = gamepad2.left_stick_y * 1;
                    intake.setPower(fIntake);
                } else if (0 > gamepad2.left_stick_y) {
                    fIntake = gamepad2.left_stick_y * 0.5;
                    intake.setPower(fIntake);
                } else if (0 == gamepad2.left_stick_y) {
                    fIntake = 0;
                    intake.setPower(fIntake);
                }

                if ( (gamepad2.right_stick_y > 0 ) || (gamepad2.right_stick_y < 0) ) {
                    fArmTop = -(gamepad2.right_stick_y);
                    armtop.setPower(fArmTop);
                } else {
                    fArmTop = 0;
                    armtop.setPower(0);
                }

                if (0 != gamepad2.right_trigger && limit_reached == 0 ) {
                    ultraPower = gamepad2.right_trigger;
                    armbottom.setPower(ultraPower);
                } else if (0 != gamepad2.left_trigger) {
                    ultraPower = -gamepad2.left_trigger;
                    armbottom.setPower(ultraPower);
                } else {
                    armbottom.setPower(0);
                    ultraPower = 0;
                }

                SOFTWARE_LIMIT();

                if (0 != gamepad1.left_trigger) {
                    // Back Code
                    bPower = gamepad1.left_trigger;
                    moterLB.setPower(bPower);
                    motorLF.setPower(bPower);
                    motorRB.setPower(-bPower);
                    motorRF.setPower(-bPower);
                } else if (0 != gamepad1.right_trigger) {
                    // Front Code
                    fPower = -(gamepad1.right_trigger * 0.8);
                    moterLB.setPower(fPower);
                    motorLF.setPower(fPower);
                    motorRB.setPower(-fPower);
                    motorRF.setPower(-fPower);
                } else if (0 != gamepad1.right_stick_x) {
                    // Start diagonal movement (shifting)
                    sPower = -(gamepad1.right_stick_x * 0.8);
                    moterLB.setPower(-sPower);
                    motorLF.setPower(sPower);
                    motorRB.setPower(-sPower);
                    motorRF.setPower(sPower);
                } else if (0 != gamepad1.left_stick_x) {
                    // Start diagonal movement (shifting)
                    dPower = -(gamepad1.left_stick_x * 0.8);
                    moterLB.setPower(dPower);
                    motorLF.setPower(dPower);
                    motorRB.setPower(dPower);
                    motorRF.setPower(dPower);
                } else {
                    // Back Code
                    bPower = 0;
                    moterLB.setPower(0);
                    motorLF.setPower(0);
                    motorRB.setPower(0);
                    motorRF.setPower(0);
                }

                if (gamepad2.right_bumper) {
                    CLAW_OPEN();
                }
                if (gamepad2.left_bumper) {
                    CLAW_CLOSE();
                }

//                // PS/Home button: Reset robot configuration
//                if (gamepad2.ps) {
//                    ultraPower = (float) 0.8;
//                    armtopposition = 0;
//                    ARM_TOP();
//                    while ( armtop.isBusy() ) {
//                        sleep(10);
//                    }
//                    armbottomposition = 0;
//                    ARM_BOTTOM();
//                    while ( armbottom.isBusy() ) {
//                        sleep(10);
//                    }
//                    // Rest runMode to RUN_WITHOUT_ENCODERS
//                    MOTORS_RUN_WITHOUT_ENCODERS();
//                }
//                // L1 : Perimeter Position
//                if (gamepad2.left_bumper) {
//                    ultraPower = (float) 0.8;
//                    armbottomposition = 1400;
//                    ARM_BOTTOM();
//                    while (armbottom.isBusy()) {
//                        sleep(10);
//                    }
//                    // Rest runMode to RUN_WITHOUT_ENCODERS
//                    MOTORS_RUN_WITHOUT_ENCODERS();
//                }
//                // R1: Specimen hanging position
//                if (gamepad2.right_bumper) {
//                    ultraPower = (float) 0.8;
//                    armbottomposition = 3200;
//                    ARM_BOTTOM();
//                    while (armbottom.isBusy()) {
//                        sleep(10);
//                    }
//                    // Rest runMode to RUN_WITHOUT_ENCODERS
//                    MOTORS_RUN_WITHOUT_ENCODERS();
//                }
//                if (gamepad2.circle) {
//                    CLAW_OPEN();
//                }
//                if (gamepad2.cross) {
//                    CLAW_CLOSE();
//                }
//                // Square : Climb position (90 degrees)
//                if (gamepad2.square) {
//                    ultraPower = (float) 0.8;
//                    armbottomposition = 2700;
//                    ARM_BOTTOM();
//                    while (armbottom.isBusy()) {
//                        sleep(10);
//                    }
//                    armtopposition = 350;
//                    ARM_TOP();
//                    while (armtop.isBusy()) {
//                        sleep(10);
//                    }
//                    // Rest runMode to RUN_WITHOUT_ENCODERS
//                    MOTORS_RUN_WITHOUT_ENCODERS();
//                }
//                // Triangle/Y: Bucket position
//                if (gamepad2.triangle) {
//                    ultraPower = (float) 0.8;
//                    armbottomposition = 2750;
//                    ARM_BOTTOM();
//                    while (armbottom.isBusy()) {
//                        sleep(10);
//                    }
//                    armtopposition = 350;
//                    ARM_TOP();
//                    while (armtop.isBusy()) {
//                        sleep(10);
//                    }
//                    // Rest runMode to RUN_WITHOUT_ENCODERS
//                    MOTORS_RUN_WITHOUT_ENCODERS();
//                }

                telemetry.addData("armbottom", armbottom.getCurrentPosition());
                telemetry.addData("armtop", armtop.getCurrentPosition());
                telemetry.addData("limit_reached", limit_reached);
                telemetry.addData("F1", fPower);
                telemetry.addData("B1", bPower);
                telemetry.addData("S1", sPower);
                telemetry.addData("D1", dPower);
                telemetry.addData("ultraPower", ultraPower);
                telemetry.addData("Intake", fIntake);
                telemetry.addData("Claw", servoclaw.getPosition());
                telemetry.addData("gamepad2.right_trigger", gamepad2.right_trigger);
                telemetry.update();
            }

            // Put run blocks here.

        }

    }

    private void MOTORS_STOP_AND_RESET_ENCODERS() {
        armbottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armtop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void MOTORS_RUN_WITHOUT_ENCODERS() {
        armbottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armtop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void SOFTWARE_LIMIT() {
        if (armbottom.getCurrentPosition() > 3500) {
            limit_reached = 1;
        } else {
            limit_reached = 0;
        }
    }

    /**
     * Describe this function...
     */
    private void ARM_BOTTOM() {
        armbottom.setPower((4 * ultraPower));
        armbottom.setTargetPosition(armbottomposition);
        armbottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void ARM_TOP() {
        armtop.setPower((3 * ultraPower));
        armtop.setTargetPosition(armtopposition);
        armtop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void CLAW_OPEN() {
        servoclaw.setDirection(Servo.Direction.REVERSE);
        servoclaw.setPosition(0.15);
    }

    /**
     * Describe this function...
     */
    private void CLAW_CLOSE() {
        servoclaw.setDirection(Servo.Direction.REVERSE);
        servoclaw.setPosition(0);
    }
}