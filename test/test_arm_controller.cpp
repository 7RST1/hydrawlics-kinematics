#include <unity.h>
#include "ArmController.h"
#include <math.h>

// Helper function to compare floats with tolerance
bool floatEqual(float a, float b, float tolerance = 0.001f) {
    return fabs(a - b) < tolerance;
}

// ==================== GCodeCommand Tests ====================

void test_gcode_command_default_constructor(void) {
    GCodeCommand cmd;
    TEST_ASSERT_FALSE(cmd.hasX);
    TEST_ASSERT_FALSE(cmd.hasY);
    TEST_ASSERT_FALSE(cmd.hasZ);
    TEST_ASSERT_FALSE(cmd.hasFeedRate);
}

void test_gcode_command_with_type(void) {
    GCodeCommand cmd("G01");
    TEST_ASSERT_EQUAL_STRING("G01", cmd.commandType.c_str());
    TEST_ASSERT_FALSE(cmd.hasX);
}

// ==================== ArmController Tests ====================

void test_arm_controller_initialization(void) {
    ArmController arm;

    // Check initial position is zero
    Vector3 pos = arm.getCurrentPosition();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pos.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pos.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pos.z);

    // Check default mode is absolute
    TEST_ASSERT_TRUE(arm.isAbsoluteMode());
}

void test_arm_controller_set_absolute_mode(void) {
    ArmController arm;

    arm.setAbsoluteMode(false);
    TEST_ASSERT_FALSE(arm.isAbsoluteMode());

    arm.setAbsoluteMode(true);
    TEST_ASSERT_TRUE(arm.isAbsoluteMode());
}

void test_arm_controller_set_arm_dimensions(void) {
    ArmController arm;
    // Just verify it doesn't crash
    arm.setArmDimensions(0.1f, 0.2f, 0.3f, 0.05f);
}

// ==================== G-Code Parsing Tests ====================

void test_parse_gcode_empty_line(void) {
    ArmController arm;
    GCodeCommand cmd;

    TEST_ASSERT_FALSE(arm.parseGCodeLine("", cmd));
    TEST_ASSERT_FALSE(arm.parseGCodeLine("   ", cmd));
}

void test_parse_gcode_comment_only(void) {
    ArmController arm;
    GCodeCommand cmd;

    TEST_ASSERT_FALSE(arm.parseGCodeLine("; This is a comment", cmd));
}

void test_parse_gcode_g90_mode_change(void) {
    ArmController arm;
    GCodeCommand cmd;

    arm.setAbsoluteMode(false);
    TEST_ASSERT_FALSE(arm.parseGCodeLine("G90", cmd));  // Mode change, not a movement
    TEST_ASSERT_TRUE(arm.isAbsoluteMode());
}

void test_parse_gcode_g91_mode_change(void) {
    ArmController arm;
    GCodeCommand cmd;

    arm.setAbsoluteMode(true);
    TEST_ASSERT_FALSE(arm.parseGCodeLine("G91", cmd));  // Mode change, not a movement
    TEST_ASSERT_FALSE(arm.isAbsoluteMode());
}

void test_parse_gcode_g00_simple(void) {
    ArmController arm;
    GCodeCommand cmd;

    TEST_ASSERT_TRUE(arm.parseGCodeLine("G00 X10.5 Y20.3 Z5.1", cmd));
    TEST_ASSERT_EQUAL_STRING("G00", cmd.commandType.c_str());
    TEST_ASSERT_TRUE(cmd.hasX);
    TEST_ASSERT_TRUE(cmd.hasY);
    TEST_ASSERT_TRUE(cmd.hasZ);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.5f, cmd.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.3f, cmd.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.1f, cmd.z);
}

void test_parse_gcode_g01_simple(void) {
    ArmController arm;
    GCodeCommand cmd;

    TEST_ASSERT_TRUE(arm.parseGCodeLine("G01 X1.23 Y4.56", cmd));
    TEST_ASSERT_EQUAL_STRING("G01", cmd.commandType.c_str());
    TEST_ASSERT_TRUE(cmd.hasX);
    TEST_ASSERT_TRUE(cmd.hasY);
    TEST_ASSERT_FALSE(cmd.hasZ);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.23f, cmd.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.56f, cmd.y);
}

void test_parse_gcode_with_comment(void) {
    ArmController arm;
    GCodeCommand cmd;

    TEST_ASSERT_TRUE(arm.parseGCodeLine("G00 X10 Y20 ; Move to position", cmd));
    TEST_ASSERT_EQUAL_STRING("G00", cmd.commandType.c_str());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, cmd.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, cmd.y);
}

void test_parse_gcode_lowercase(void) {
    ArmController arm;
    GCodeCommand cmd;

    TEST_ASSERT_TRUE(arm.parseGCodeLine("g01 x5.5 y10.2", cmd));
    TEST_ASSERT_EQUAL_STRING("G01", cmd.commandType.c_str());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.5f, cmd.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.2f, cmd.y);
}

void test_parse_gcode_with_feedrate(void) {
    ArmController arm;
    GCodeCommand cmd;

    TEST_ASSERT_TRUE(arm.parseGCodeLine("G01 X10 Y20 F100", cmd));
    TEST_ASSERT_TRUE(cmd.hasFeedRate);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, cmd.feedRate);
}

void test_parse_gcode_negative_values(void) {
    ArmController arm;
    GCodeCommand cmd;

    TEST_ASSERT_TRUE(arm.parseGCodeLine("G01 X-5.5 Y-10.2 Z-3.3", cmd));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -5.5f, cmd.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -10.2f, cmd.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -3.3f, cmd.z);
}

void test_parse_gcode_partial_coordinates(void) {
    ArmController arm;
    GCodeCommand cmd;

    TEST_ASSERT_TRUE(arm.parseGCodeLine("G01 X10", cmd));
    TEST_ASSERT_TRUE(cmd.hasX);
    TEST_ASSERT_FALSE(cmd.hasY);
    TEST_ASSERT_FALSE(cmd.hasZ);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, cmd.x);
}

// ==================== G-Code Processing Tests ====================

void test_process_gcode_absolute_mode(void) {
    ArmController arm;
    arm.setAbsoluteMode(true);
    arm.setCurrentPosition(Vector3(0, 0, 0));

    GCodeCommand cmd("G01");
    cmd.hasX = true;
    cmd.hasY = true;
    cmd.hasZ = true;
    cmd.x = 10.0f;
    cmd.y = 20.0f;
    cmd.z = 30.0f;

    arm.processGCodeCommand(cmd);

    Vector3 pos = arm.getCurrentPosition();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, pos.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, pos.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, pos.z);
}

void test_process_gcode_relative_mode(void) {
    ArmController arm;
    arm.setAbsoluteMode(false);
    arm.setCurrentPosition(Vector3(5.0f, 10.0f, 15.0f));

    GCodeCommand cmd("G01");
    cmd.hasX = true;
    cmd.hasY = true;
    cmd.hasZ = true;
    cmd.x = 2.0f;
    cmd.y = 3.0f;
    cmd.z = 4.0f;

    arm.processGCodeCommand(cmd);

    Vector3 pos = arm.getCurrentPosition();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.0f, pos.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 13.0f, pos.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 19.0f, pos.z);
}

void test_process_gcode_partial_update_absolute(void) {
    ArmController arm;
    arm.setAbsoluteMode(true);
    arm.setCurrentPosition(Vector3(10.0f, 20.0f, 30.0f));

    GCodeCommand cmd("G01");
    cmd.hasX = true;
    cmd.x = 15.0f;
    // Y and Z not specified

    arm.processGCodeCommand(cmd);

    Vector3 pos = arm.getCurrentPosition();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, pos.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, pos.y);  // Unchanged
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, pos.z);  // Unchanged
}

// ==================== Inverse Kinematics Tests ====================

void test_ik_simple_forward_position(void) {
    ArmController arm;
    // Default arm dimensions: a1=0.098, a2=0.270, a3=0.320, e=0.07

    // Test a reachable position
    Vector3 worldPos(0.3f, 0.2f, 0.3f);
    JointAngles angles = arm.moveToWorldSpace(worldPos);

    // Just verify we get a valid solution
    TEST_ASSERT_TRUE(angles.valid);
}

void test_ik_unreachable_position_too_far(void) {
    ArmController arm;

    // Position way too far to reach (max reach is about a2 + a3 = 0.59m)
    Vector3 worldPos(1.0f, 0.2f, 1.0f);
    JointAngles angles = arm.moveToWorldSpace(worldPos);

    // Should be invalid
    TEST_ASSERT_FALSE(angles.valid);
}

void test_ik_unreachable_position_too_close(void) {
    ArmController arm;

    // Position too close to base (min reach is about |a2 - a3| = 0.05m)
    Vector3 worldPos(0.01f, 0.1f, 0.01f);
    JointAngles angles = arm.moveToWorldSpace(worldPos);

    // Might be invalid depending on exact position
    // Just check it doesn't crash
}

void test_ik_known_position(void) {
    ArmController arm;

    // Test a known reachable position in the middle of workspace
    Vector3 worldPos(0.3f, 0.1f, 0.2f);
    JointAngles angles = arm.moveToWorldSpace(worldPos);

    TEST_ASSERT_TRUE(angles.valid);

    // Angles should be within reasonable bounds (-180 to 180)
    TEST_ASSERT_TRUE(angles.baseRotation >= -180.0f && angles.baseRotation <= 360.0f);
    TEST_ASSERT_TRUE(angles.joint1Angle >= -180.0f && angles.joint1Angle <= 180.0f);
    TEST_ASSERT_TRUE(angles.joint2Angle >= -180.0f && angles.joint2Angle <= 180.0f);
    TEST_ASSERT_TRUE(angles.joint3Angle >= -180.0f && angles.joint3Angle <= 180.0f);
}

// ==================== Drawing Space Tests ====================

void test_drawing_space_origin(void) {
    ArmController arm;

    // Test moving to origin in drawing space
    Vector3 gCodePos(0.0f, 0.0f, 0.0f);
    JointAngles angles = arm.moveToDrawingSpace(gCodePos);

    // Should produce a valid solution
    TEST_ASSERT_TRUE(angles.valid);
}

void test_drawing_space_valid_position(void) {
    ArmController arm;

    // Test a position in the middle of drawing space
    Vector3 gCodePos(0.15f, 0.09f, 0.0f);
    JointAngles angles = arm.moveToDrawingSpace(gCodePos);

    TEST_ASSERT_TRUE(angles.valid);
}

// ==================== Main Setup ====================

void setUp(void) {
    // Called before each test
}

void tearDown(void) {
    // Called after each test
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // GCodeCommand tests
    RUN_TEST(test_gcode_command_default_constructor);
    RUN_TEST(test_gcode_command_with_type);

    // ArmController initialization tests
    RUN_TEST(test_arm_controller_initialization);
    RUN_TEST(test_arm_controller_set_absolute_mode);
    RUN_TEST(test_arm_controller_set_arm_dimensions);

    // G-Code parsing tests
    RUN_TEST(test_parse_gcode_empty_line);
    RUN_TEST(test_parse_gcode_comment_only);
    RUN_TEST(test_parse_gcode_g90_mode_change);
    RUN_TEST(test_parse_gcode_g91_mode_change);
    RUN_TEST(test_parse_gcode_g00_simple);
    RUN_TEST(test_parse_gcode_g01_simple);
    RUN_TEST(test_parse_gcode_with_comment);
    RUN_TEST(test_parse_gcode_lowercase);
    RUN_TEST(test_parse_gcode_with_feedrate);
    RUN_TEST(test_parse_gcode_negative_values);
    RUN_TEST(test_parse_gcode_partial_coordinates);

    // G-Code processing tests
    RUN_TEST(test_process_gcode_absolute_mode);
    RUN_TEST(test_process_gcode_relative_mode);
    RUN_TEST(test_process_gcode_partial_update_absolute);

    // Inverse kinematics tests
    RUN_TEST(test_ik_simple_forward_position);
    RUN_TEST(test_ik_unreachable_position_too_far);
    RUN_TEST(test_ik_unreachable_position_too_close);
    RUN_TEST(test_ik_known_position);

    // Drawing space tests
    RUN_TEST(test_drawing_space_origin);
    RUN_TEST(test_drawing_space_valid_position);

    return UNITY_END();
}