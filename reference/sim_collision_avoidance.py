#!/usr/bin/env python3
"""Référence normative en miroir du simulateur Rust ALG_COLLISION_AVOIDANCE."""

import random
import sys


def correction(pi, pj, vi, vj, distance_min, alpha):
    original_x = pi[0] - pj[0]
    original_y = pi[1] - pj[1]
    rx, ry = original_x, original_y
    r2 = rx * rx + ry * ry
    if r2 == 0.0:
        rx, ry = distance_min, 0.0
        r2 = rx * rx + ry * ry
    h = original_x * original_x + original_y * original_y - distance_min * distance_min
    relx, rely = vi[0] - vj[0], vi[1] - vj[1]
    lhs = 2.0 * (rx * relx + ry * rely) + alpha * h
    if lhs >= 0.0:
        return vi, vj, False
    lam = -lhs / (4.0 * r2)
    return ((vi[0] + rx * lam, vi[1] + ry * lam),
            (vj[0] - rx * lam, vj[1] - ry * lam), True)


def simuler(seed, scenario=0, agents=8, steps=80, distance_min=1.0):
    if agents < 2 or steps <= 0 or distance_min <= 0.0:
        raise ValueError("paramètres invalides")
    dt, alpha = 0.1, 2.0
    rng = random.Random(seed)
    espacement = 1.6 if scenario == 0 else (1.1 if scenario == 1 else 0.85)
    positions, consignes = [], []
    for i in range(agents):
        gauche = i % 2 == 0
        paire = float(i // 2)
        x_abs = 4.0 + 2.0 * rng.random()
        x = -x_abs if gauche else x_abs
        y = paire * espacement + (rng.random() - 0.5) * 0.08
        vitesse = 0.8 + 0.4 * rng.random()
        positions.append((x, y))
        consignes.append((vitesse if gauche else -vitesse, 0.0))

    collisions = quasi = interventions = 0
    reaction = None
    minimum = float("inf")
    surcout = 0.0
    for etape in range(steps):
        vitesses = list(consignes)
        for passe in range(4):
            for i in range(agents):
                for j in range(i + 1, agents):
                    vi, vj, active = correction(positions[i], positions[j], vitesses[i], vitesses[j], distance_min, alpha)
                    vitesses[i], vitesses[j] = vi, vj
                    if passe == 0 and active:
                        interventions += 1
                        if reaction is None:
                            reaction = etape
        for i in range(agents):
            dx = vitesses[i][0] - consignes[i][0]
            dy = vitesses[i][1] - consignes[i][1]
            surcout += (dx * dx + dy * dy) ** 0.5 * dt
            positions[i] = (positions[i][0] + vitesses[i][0] * dt,
                            positions[i][1] + vitesses[i][1] * dt)
        for i in range(agents):
            for j in range(i + 1, agents):
                dx = positions[i][0] - positions[j][0]
                dy = positions[i][1] - positions[j][1]
                distance = (dx * dx + dy * dy) ** 0.5
                if distance < minimum:
                    minimum = distance
                if distance < distance_min * 0.5:
                    collisions += 1
                elif distance < distance_min * 1.25:
                    quasi += 1
    return collisions, quasi, interventions, (-1 if reaction is None else reaction), minimum, surcout


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help", "--aide"):
        print("sim_collision_avoidance.py <seed> [scenario] [agents] [steps] [distance_min]", file=sys.stderr)
        return 0
    seed = int(sys.argv[1])
    scenario = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    agents = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    steps = int(sys.argv[4]) if len(sys.argv) > 4 else 80
    distance_min = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0
    r = simuler(seed, scenario, agents, steps, distance_min)
    print("seed,scenario,agents,steps,distance_min,collisions,quasi_collisions,interventions,temps_reaction,distance_min_observee,surcout_commande")
    print(f"{seed},{scenario},{agents},{steps},{distance_min:.6f},{r[0]},{r[1]},{r[2]},{r[3]},{r[4]:.6f},{r[5]:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
