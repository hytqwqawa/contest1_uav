#!/usr/bin/env python3
"""Pure geometric helpers for Demo11 RViz schematic. Never used for flight validation.

Demo coordinates (F,L,H) are converted using the current DISARMED Frame Guard
session. Physical straight-wall limits are read from the existing corridor YAML;
the 90-degree round bend is only an *illustration* of the reference arc.
"""
import math


def world_point(session, forward, left, height):
    """World xyz via orthonormal frame from session_frame.yaml."""
    origin = session['origin_world']
    fw = session['forward_world']
    lw = session['left_world']
    values = (
        origin['x'], origin['y'], origin['z'],
        fw['x'], fw['y'], lw['x'], lw['y'],
        forward, left, height,
    )
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError('non-finite transform')
    return (
        float(origin['x']) + float(fw['x']) * forward + float(lw['x']) * left,
        float(origin['y']) + float(fw['y']) * forward + float(lw['y']) * left,
        float(origin['z']) + height,
    )


def quarter_arc(center_f=3.0, center_l=-1.5, radius=1.5, samples=72):
    """Clockwise 90-degree reference arc: (3,0) -> (4.5,-1.5)."""
    if not (radius > 0.0 and 2 <= int(samples) <= 2000):
        raise ValueError('invalid arc radius/samples')
    result = []
    for i in range(int(samples) + 1):
        theta = (math.pi / 2.0) * (1.0 - i / float(samples))
        result.append((center_f + radius * math.cos(theta),
                       center_l + radius * math.sin(theta)))
    return result


def round_tube_ring(center_f, center_l, bend_radius, tube_radius,
                    theta_deg, flight_height, samples=24):
    """One circular 3-D rib, normal to an arc tangent; F/L/H triples."""
    if not (bend_radius > tube_radius > 0.0 and samples >= 8):
        raise ValueError('invalid round-tube cross-section')
    theta = math.radians(theta_deg)
    result = []
    for i in range(samples + 1):
        phi = 2.0 * math.pi * (i / float(samples))
        r = bend_radius + tube_radius * math.cos(phi)
        result.append((center_f + r * math.cos(theta),
                       center_l + r * math.sin(theta),
                       flight_height + tube_radius * math.sin(phi)))
    return result


def static_scene_definitions(corridor, arc):
    """Serialize the two independent schematic geometries for tests/diagnosis."""
    return {
        'straight_physical_wall_F': (corridor.f_start, corridor.f_end),
        'straight_physical_wall_L': (corridor.l_min, corridor.l_max),
        'straight_protected_center_F': (corridor.f_guard_min, corridor.f_guard_max),
        'straight_protected_center_L': (corridor.center_l_min, corridor.center_l_max),
        'arc_reference_center': (float(arc['center_f']), float(arc['center_l'])),
        'arc_reference_radius': float(arc['radius_m']),
        'arc_is_physical_safety_boundary': False,
    }
