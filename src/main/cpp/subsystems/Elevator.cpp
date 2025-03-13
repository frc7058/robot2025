#include "subsystems/Elevator.h"
#include "constants/ElevatorConstants.h"

Elevator::Elevator() {
    m_feedforward = std::make_unique<frc::ElevatorFeedforward>(
        constants::elevator::feedforward::staticFriction,
        constants::elevator::feedforward::gravity,
        constants::elevator::feedforward::velocity,
        constants::elevator::feedforward::acceleration);

    m_pid = std::make_unique<frc::ProfiledPIDController<units::meters>>(
        constants::elevator::pid::p,
        constants::elevator::pid::i,
        constants::elevator::pid::d,
        frc::TrapezoidProfile<units::meters>::Constraints(
            constants::elevator::pid::maxVelocity,
            constants::elevator::pid::maxAcceleration
        ));
}

void Elevator::Periodic() {}