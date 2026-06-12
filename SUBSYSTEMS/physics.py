from __future__ import annotations

from dataclasses import dataclass
import math

from CONFIG.config import (
    BRAKE_FORCE_N,
    EMERGENCY_FORCE_N,
    ROTATING_MASS_FACTOR,
    RUNNING_RESIST_A_MS2,
    RUNNING_RESIST_B_S1,
    RUNNING_RESIST_C_INV_M,
    TRAIN_MASS_KG,
    TRACTION_FORCE_N,
)


G = 9.81


@dataclass(frozen=True)
class LongitudinalForces:
    """Resolved longitudinal force balance for one simulation sample."""

    traction_force_n: float
    brake_force_n: float
    resistance_force_n: float
    grade_force_n: float
    net_force_n: float
    acceleration_ms2: float
    traction_work_j: float = 0.0
    brake_work_j: float = 0.0


@dataclass(frozen=True)
class RuntimeDynamicsResult:
    """Runtime dynamics sample resolved through the shared force primitive."""

    acceleration_ms2: float
    forces: LongitudinalForces
    traction_command: float
    brake_command: float
    brake_mode: str
    legacy_acceleration_ms2: float
    used_fallback: bool = False


def kmh_to_ms(kmh: float) -> float:
    return kmh / 3.6


def ms_to_kmh(ms: float) -> float:
    return ms * 3.6


def braking_distance_m(speed_ms: float, a_emergency: float) -> float:
    if a_emergency <= 0:
        return float("inf")
    return (speed_ms * speed_ms) / (2.0 * a_emergency)


def allowed_speed_for_distance(distance_m: float, a_emergency: float) -> float:
    if distance_m <= 0:
        return 0.0
    return math.sqrt(2.0 * a_emergency * distance_m)


def equivalent_mass_kg(mass_kg: float) -> float:
    """Return train mass including equivalent rotating mass allowance."""
    return max(0.0, mass_kg) * max(1.0, ROTATING_MASS_FACTOR)


def davis_resistance_accel_ms2(speed_ms: float) -> float:
    """Return Davis running resistance A + Bv + Cv^2 as a deceleration magnitude."""
    speed = max(0.0, speed_ms)
    if speed <= 1e-6:
        return 0.0
    return (
        RUNNING_RESIST_A_MS2
        + RUNNING_RESIST_B_S1 * speed
        + RUNNING_RESIST_C_INV_M * speed * speed
    )


def davis_resistance_force_n(speed_ms: float, mass_kg: float = TRAIN_MASS_KG) -> float:
    """Return Davis running resistance force in N, opposing forward motion."""
    return davis_resistance_accel_ms2(speed_ms) * equivalent_mass_kg(mass_kg)


def traction_acceleration_ms2(speed_ms: float) -> float:
    """Estimate traction acceleration capability as a realistic speed-dependent profile."""
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= 50.0:
        base_accel = max(0.9, 1.05 - 0.003 * speed_kmh)
        return base_accel / max(1.0, ROTATING_MASS_FACTOR)
    if speed_kmh <= 100.0:
        base_accel = max(0.5, 0.9 - 0.008 * (speed_kmh - 50.0))
        return base_accel / max(1.0, ROTATING_MASS_FACTOR)
    return 0.5 / max(1.0, ROTATING_MASS_FACTOR)


def traction_force_n(speed_ms: float, mass_kg: float = TRAIN_MASS_KG, command: float = 1.0) -> float:
    """Return available commanded traction force at the current speed.

    The speed profile preserves the legacy traction acceleration shape, while
    exposing it as force so new physics can resolve acceleration from total
    force divided by equivalent mass.
    """
    command = max(0.0, min(1.0, command))
    available_force = traction_acceleration_ms2(speed_ms) * equivalent_mass_kg(mass_kg)
    return command * min(TRACTION_FORCE_N, available_force)


def brake_force_n(mode: str = "service", command: float = 1.0) -> float:
    """Return commanded brake force in N for service or emergency braking."""
    command = max(0.0, min(1.0, command))
    mode_key = str(mode).strip().lower()
    if mode_key in {"emergency", "ebi", "eb"}:
        return command * EMERGENCY_FORCE_N
    if mode_key in {"service", "sbi", "sb"}:
        return command * BRAKE_FORCE_N
    if mode_key in {"none", "coast", "off"}:
        return 0.0
    raise ValueError(f"Unsupported brake mode: {mode}")


def grade_resistance_force_n(mass_kg: float, gradient: float) -> float:
    """Return grade force in N, positive uphill and negative downhill."""
    return max(0.0, mass_kg) * G * gradient


def longitudinal_force_balance(
    speed_ms: float,
    mass_kg: float = TRAIN_MASS_KG,
    traction_command: float = 0.0,
    brake_command: float = 0.0,
    brake_mode: str = "service",
    gradient: float = 0.0,
    distance_m: float = 0.0,
) -> LongitudinalForces:
    """Resolve train acceleration from traction, brake, Davis resistance and grade.

    Positive acceleration is in the forward running direction. Positive
    gradient means uphill, so it reduces acceleration; negative gradient means
    downhill, so it assists forward motion.
    """
    traction_n = traction_force_n(speed_ms, mass_kg, traction_command)
    brake_n = brake_force_n(brake_mode, brake_command)
    resistance_n = davis_resistance_force_n(speed_ms, mass_kg)
    grade_n = grade_resistance_force_n(mass_kg, gradient)
    net_n = traction_n - brake_n - resistance_n - grade_n
    accel = net_n / max(equivalent_mass_kg(mass_kg), 1e-9)
    travel = max(0.0, distance_m)
    return LongitudinalForces(
        traction_force_n=traction_n,
        brake_force_n=brake_n,
        resistance_force_n=resistance_n,
        grade_force_n=grade_n,
        net_force_n=net_n,
        acceleration_ms2=accel,
        traction_work_j=max(0.0, traction_n * travel),
        brake_work_j=max(0.0, brake_n * travel),
    )


def legacy_runtime_acceleration_ms2(
    speed_ms: float,
    commanded_accel_ms2: float,
    gradient: float = 0.0,
) -> float:
    """Legacy runtime acceleration path kept as fallback for regression isolation."""
    accel = commanded_accel_ms2
    if speed_ms > 1e-6 or accel > 0.0:
        accel -= running_resistance_accel_ms2(speed_ms)
    accel -= G * gradient
    return accel


def resolve_runtime_dynamics(
    speed_ms: float,
    mass_kg: float,
    commanded_accel_ms2: float,
    gradient: float = 0.0,
    traction_command_hint: float = 0.0,
    brake_command_hint: float = 0.0,
    brake_mode_hint: str = "service",
    force_balance_enabled: bool = True,
) -> RuntimeDynamicsResult:
    """Resolve Train.step acceleration through force balance with legacy fallback.

    ``commanded_accel_ms2`` is the existing controller acceleration request before
    running resistance and grade. The adapter maps it into traction/brake command
    fractions, then uses ``longitudinal_force_balance`` as the shared primitive.
    """
    legacy_accel = legacy_runtime_acceleration_ms2(speed_ms, commanded_accel_ms2, gradient)
    if not force_balance_enabled:
        empty = LongitudinalForces(0.0, 0.0, 0.0, 0.0, 0.0, legacy_accel)
        return RuntimeDynamicsResult(
            acceleration_ms2=legacy_accel,
            forces=empty,
            traction_command=0.0,
            brake_command=0.0,
            brake_mode="legacy",
            legacy_acceleration_ms2=legacy_accel,
            used_fallback=True,
        )

    traction_hint = max(0.0, min(1.0, traction_command_hint))
    brake_hint = max(0.0, min(1.0, brake_command_hint))
    brake_mode = str(brake_mode_hint or "service").lower()
    if brake_mode not in {"service", "sbi", "sb", "emergency", "ebi", "eb", "none", "coast", "off"}:
        brake_mode = "service"

    traction_command = 0.0
    brake_command = 0.0
    if commanded_accel_ms2 > 0.0:
        max_traction_accel = max(traction_acceleration_ms2(speed_ms), 1e-9)
        command_from_accel = commanded_accel_ms2 / max_traction_accel
        traction_command = min(command_from_accel, traction_hint) if traction_hint > 0.0 else command_from_accel
    elif commanded_accel_ms2 < 0.0:
        if brake_mode in {"none", "coast", "off"} and brake_hint <= 0.0:
            brake_command = 0.0
        else:
            full_brake_force = brake_force_n(brake_mode, 1.0)
            if full_brake_force <= 0.0:
                full_brake_force = brake_force_n("service", 1.0)
                brake_mode = "service"
            command_from_accel = abs(commanded_accel_ms2) * equivalent_mass_kg(mass_kg) / max(full_brake_force, 1e-9)
            brake_command = max(brake_hint, command_from_accel)
    elif brake_hint > 0.0:
        brake_command = brake_hint

    traction_command = max(0.0, min(1.0, traction_command))
    brake_command = max(0.0, min(1.0, brake_command))
    if brake_command <= 0.0:
        brake_mode = "none"

    forces = longitudinal_force_balance(
        speed_ms=speed_ms,
        mass_kg=mass_kg,
        traction_command=traction_command,
        brake_command=brake_command,
        brake_mode=brake_mode,
        gradient=gradient,
    )
    return RuntimeDynamicsResult(
        acceleration_ms2=forces.acceleration_ms2,
        forces=forces,
        traction_command=traction_command,
        brake_command=brake_command,
        brake_mode=brake_mode,
        legacy_acceleration_ms2=legacy_accel,
        used_fallback=False,
    )


def acceleration_from_forces_ms2(
    speed_ms: float,
    mass_kg: float = TRAIN_MASS_KG,
    traction_command: float = 0.0,
    brake_command: float = 0.0,
    brake_mode: str = "service",
    gradient: float = 0.0,
) -> float:
    """Convenience wrapper returning only acceleration from the force balance."""
    return longitudinal_force_balance(
        speed_ms=speed_ms,
        mass_kg=mass_kg,
        traction_command=traction_command,
        brake_command=brake_command,
        brake_mode=brake_mode,
        gradient=gradient,
    ).acceleration_ms2


def braking_distance_from_force_m(
    speed_ms: float,
    mass_kg: float = TRAIN_MASS_KG,
    brake_mode: str = "emergency",
    gradient: float = 0.0,
) -> float:
    """Estimate stopping distance using initial-speed force balance.

    This is a compact physics sanity helper, not a replacement for ATP's
    build-up/reaction-delay brake curves.
    """
    if speed_ms <= 0.0:
        return 0.0
    accel = acceleration_from_forces_ms2(
        speed_ms=speed_ms,
        mass_kg=mass_kg,
        traction_command=0.0,
        brake_command=1.0,
        brake_mode=brake_mode,
        gradient=gradient,
    )
    decel = -accel
    if decel <= 0.0:
        return float("inf")
    return braking_distance_m(speed_ms, decel)


def running_resistance_accel_ms2(speed_ms: float) -> float:
    """Return train running resistance as a positive deceleration magnitude."""
    return davis_resistance_accel_ms2(speed_ms)


def equivalent_mass_adjusted_accel(accel_ms2: float) -> float:
    """Apply equivalent rotating mass to commanded longitudinal acceleration."""
    return accel_ms2 / max(1.0, ROTATING_MASS_FACTOR)


def limit_jerk(previous_accel: float, target_accel: float, max_jerk: float, dt: float) -> float:
    """Limit acceleration changes so longitudinal jerk stays within comfort limits."""
    if dt <= 0.0:
        return target_accel
    delta = target_accel - previous_accel
    max_delta = max_jerk * dt
    if delta > max_delta:
        return previous_accel + max_delta
    if delta < -max_delta:
        return previous_accel - max_delta
    return target_accel

