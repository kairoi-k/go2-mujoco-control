#!/usr/bin/env python3
"""Independent standard-library RK4 and cone-vertex oracle for C0 synthetic packets.
Consumes exported physical inputs, never solver Jacobians or success booleans.
No simulator, controller, B0/B1 analyzer or external optimization dependency.
"""
import argparse
import itertools
import json
import math
from pathlib import Path


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def add(a, b):
    return [x+y for x, y in zip(a, b)]


def sub(a, b):
    return [x-y for x, y in zip(a, b)]


def scale(a, b):
    return [x*b for x in a]


def derivative(x, forces, contacts, mass, gravity):
    total = [sum(f[a] for f in forces) for a in range(3)]
    torques = [cross(sub(c['foot'], x[:3]), f) for c, f in zip(contacts, forces)]
    return x[3:6] + [total[a]/mass - (gravity if a == 2 else 0) for a in range(3)] + [sum(t[a] for t in torques) for a in range(3)]


def rk4(x, forces, contacts, mass, gravity, dt):
    for _ in range(32):
        h = dt/32
        fn = lambda v: derivative(v, forces, contacts, mass, gravity)
        a = fn(x)
        b = fn(add(x, scale(a, h/2)))
        c = fn(add(x, scale(b, h/2)))
        d = fn(add(x, scale(c, h)))
        x = add(x, scale(add(add(a, scale(b, 2)), add(scale(c, 2), d)), h/6))
    return x


def verify_separation(case):
    p, w = case['input'], case['separation']
    assert w['valid']
    k, d = w['interval'], w['direction']
    dt = p['grid'][k+1]-p['grid'][k]
    lo = p['initial'] if k == 0 else p['lower'][k]
    hi = p['initial'] if k == 0 else p['upper'][k]
    low = [p['mass']*((p['lower'][k+1][a+3]-hi[a+3])/dt + (p['gravity'] if a == 2 else 0)) for a in range(3)]
    high = [p['mass']*((p['upper'][k+1][a+3]-lo[a+3])/dt + (p['gravity'] if a == 2 else 0)) for a in range(3)]
    low += [(p['lower'][k+1][a]-hi[a])/dt for a in range(6, 9)]
    high += [(p['upper'][k+1][a]-lo[a])/dt for a in range(6, 9)]
    required = sum(a*(l if a >= 0 else h) for a, l, h in zip(d, low, high))
    available = 0
    for c in p['intervals'][k]:
        if not c['contact']:
            continue
        values = []
        for n, s, t in itertools.product((c['min'], c['max']), (-1, 1), (-1, 1)):
            local = [s*n*c['mu']/math.sqrt(2), t*n*c['mu']/math.sqrt(2), n]
            f = [dot(row, local) for row in c['basis']]
            # Independently integrate a single point force; force self moment
            # vanishes. The initial COM/velocity only enters angular k0 proofs.
            delta = sub(rk4(p['initial'][:], [f], [c], p['mass'], p['gravity'], dt), p['initial'])
            values.append(dot(d[:3], f) + dot(d[3:], scale(delta[6:9], 1/dt)))
        available += max(values)
    assert abs(required-w['required']) < 1e-8, (required, w)
    assert abs(available-w['available']) < 1e-8, (available, w)
    assert required > available+1e-6
    return {'name': case['name'], 'verified_infeasible': True, 'separation_gap': required-available}


def verify(case):
    if case['failure'] == 'dynamics_infeasible':
        return verify_separation(case)
    assert case['failure'] == 'none', case['failure']
    p, states, forces = case['input'], case['states'], case['forces']
    assert len(states) == len(p['grid']) and len(forces) == len(states)-1
    x = p['initial'][:]
    maximum = [0., 0., 0.]
    force_violation = 0.
    for k, fs in enumerate(forces):
        for c, f in zip(p['intervals'][k], fs):
            assert all(math.isfinite(a) for a in f)
            if not c['contact']:
                assert f == [0, 0, 0], f
                continue
            local = [sum(c['basis'][a][b]*f[a] for a in range(3)) for b in range(3)]
            force_violation = max(force_violation, c['min']-local[2], local[2]-c['max'], abs(local[0])-c['mu']/math.sqrt(2)*local[2], abs(local[1])-c['mu']/math.sqrt(2)*local[2])
        x = rk4(x, fs, p['intervals'][k], p['mass'], p['gravity'], p['grid'][k+1]-p['grid'][k])
        error = [abs(a-b) for a, b in zip(x, states[k+1][1:])]
        for group in range(3):
            maximum[group] = max(maximum[group], *error[3*group:3*group+3])
    assert max(maximum) < 2e-7, maximum
    assert force_violation < 2e-5, force_violation
    if case['name'] == 'rest':
        assert all(abs(f[l][2]-49.05) < 2e-5 for f in forces for l in (0, 3))
    if case['name'] == 'aerial':
        for state in states:
            t = state[0]-p['grid'][0]
            assert abs(state[3]-(.4-4.905*t*t)) < 1e-12
    return {'name': case['name'], 'verified_feasible': True, 'rk4_position_m': maximum[0], 'rk4_velocity_mps': maximum[1], 'rk4_momentum_nms': maximum[2], 'force_violation_n': force_violation}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('packet', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    packet = json.loads(args.packet.read_text())
    results = [verify(c) for c in packet['cases']]
    args.output.write_text(json.dumps({'passed': True, 'method': 'independent RK4(32) and enumerated cone vertices', 'cases': results}, indent=2)+'\n')
    print(f'Independent oracle PASS: {len(results)} cases')


if __name__ == '__main__':
    main()
