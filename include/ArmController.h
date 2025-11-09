#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#ifdef NATIVE_TEST
    // Native testing mode - use standard C++ libraries
    #include <string>
    #include <cmath>
    #include <cstdlib>
    #include <cstring>
    #include <cctype>
    #include <cstdio>

    // Use std::string for native testing
    typedef std::string String;

    // Mock Serial for native testing
    class MockSerial {
    public:
        void begin(unsigned long) {}
        void println(const std::string& s) { printf("%s\n", s.c_str()); }
        void println(const char* s) { printf("%s\n", s); }
    };
    extern MockSerial Serial;
#else
    // Arduino mode - use Arduino libraries
    #include <Arduino.h>
#endif

// Simple 3D vector structure for positions
struct Vector3 {
    float x;
    float y;
    float z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }

    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }

    Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    float magnitude() const {
        return sqrt(x * x + y * y + z * z);
    }

    Vector3 normalized() const {
        float mag = magnitude();
        if (mag > 0.0001f) {
            return Vector3(x / mag, y / mag, z / mag);
        }
        return Vector3(0, 0, 0);
    }
};

// Structure representing a single G-Code command
struct GCodeCommand {
    String commandType;  // G00, G01, G02, G03, etc.
    bool hasX;
    bool hasY;
    bool hasZ;
    bool hasFeedRate;
    float x;
    float y;
    float z;
    float feedRate;

    GCodeCommand() : hasX(false), hasY(false), hasZ(false), hasFeedRate(false),
                     x(0), y(0), z(0), feedRate(0) {}

    GCodeCommand(const String& type) : commandType(type), hasX(false), hasY(false),
                                       hasZ(false), hasFeedRate(false),
                                       x(0), y(0), z(0), feedRate(0) {}
};

// Result structure for joint angles
struct JointAngles {
    float baseRotation;  // theta_1 (rotation around vertical axis)
    float joint1Angle;   // theta_2 (first arm segment)
    float joint2Angle;   // theta_3 (second arm segment)
    float joint3Angle;   // end effector angle
    bool valid;          // whether IK solution is valid

    JointAngles() : baseRotation(0), joint1Angle(0), joint2Angle(0),
                    joint3Angle(0), valid(false) {}
};

class ArmController {
public:
    // Constructor
    ArmController();

    // Configuration
    void setArmDimensions(float a1, float a2, float a3, float endEffectorLength);
    void setDrawingSpaceOffset(Vector3 offset);
    void setJointAngleTolerance(float tolerance);

    // G-Code processing
    bool parseGCodeLine(const String& line, GCodeCommand& outCommand);
    void processGCodeCommand(const GCodeCommand& cmd);
    void setAbsoluteMode(bool absolute);
    bool isAbsoluteMode() const { return absoluteMode; }

    // Movement functions
    JointAngles calculateJointAngles(const Vector3& position);
    JointAngles moveToDrawingSpace(const Vector3& gCodePos);
    JointAngles moveToWorldSpace(const Vector3& worldPos);

    // State getters
    Vector3 getCurrentPosition() const { return currentPosition; }
    void setCurrentPosition(const Vector3& pos) { currentPosition = pos; }

    // Debug/logging
    void enableDebug(bool enable) { debugEnabled = enable; }

private:
    // Arm dimensions (in meters)
    float a1;  // Height of first joint above base
    float a2;  // Length of first arm segment
    float a3;  // Length of second arm segment
    float endEffectorMagnitude;  // Length of end effector

    // Drawing space configuration
    Vector3 drawSpaceOffset;

    // G-Code state
    Vector3 currentPosition;
    bool absoluteMode;  // true = G90 (absolute), false = G91 (relative)

    // Control parameters
    float jointAngleTolerance;

    // Debug
    bool debugEnabled;

    // Helper functions
    JointAngles calculateInverseKinematics(const Vector3& endEffectorOriginPos);
    Vector3 adjustForEndEffector(const Vector3& tipPosition, const Vector3& basePosition);
    Vector3 translateToWorldSpace(const Vector3& gCodePos);

    // Utility
    void debugLog(const String& message);
    float parseFloat(const String& str, bool& success);
    String floatToString(float value, int decimals);
};

#endif // ARM_CONTROLLER_H