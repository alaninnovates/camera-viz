#include <frc/geometry/Transform2d.h>

namespace Constants
{
    const frc::Transform2d LEFT_CAMERA_TRANSFORM = {
        frc::Translation2d{0.355_m, 0.135_m},
        frc::Rotation2d{0_deg}};
    const frc::Transform2d RIGHT_CAMERA_TRANSFORM = {
        frc::Translation2d{0.355_m, -0.135_m},
        frc::Rotation2d{0_deg}};
    const double CONTROLLER_DEADBAND = 0.1;
}