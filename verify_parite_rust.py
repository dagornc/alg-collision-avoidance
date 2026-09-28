#!/usr/bin/env python3
"""Mesure la parité CSV Rust/Python de ALG_COLLISION_AVOIDANCE."""

import argparse
import subprocess
import sys
from pathlib import Path

RACINE = Path(__file__).resolve().parent
REFERENCE = RACINE / "reference" / "sim_collision_avoidance.py"
SCENARIOS = {
    "nominal": (0, 8, 80, 1.0),
    "dense": (1, 10, 80, 1.0),
    "stress": (2, 12, 100, 1.0),
}


def ligne(cmd):
    sortie = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout.strip().splitlines()
    return sortie[-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binaire", default=str(RACINE / "target" / "release" / "collision_avoidance_rs"))
    parser.add_argument("--graines", nargs=2, type=int, default=(1000, 1050))
    parser.add_argument("--scenarios", nargs="*", choices=SCENARIOS, default=list(SCENARIOS))
    args = parser.parse_args()
    binaire = Path(args.binaire)
    if not binaire.exists():
        print(f"ERREUR : binaire introuvable : {binaire}", file=sys.stderr)
        return 2
    total = ecarts = 0
    for nom in args.scenarios:
        scenario = SCENARIOS[nom]
        identiques = 0
        for seed in range(*args.graines):
            valeurs = [str(seed), *(str(v) for v in scenario)]
            python = ligne([sys.executable, str(REFERENCE), *valeurs])
            rust = ligne([str(binaire), *valeurs])
            total += 1
            if python == rust:
                identiques += 1
            else:
                ecarts += 1
                if ecarts <= 10:
                    print(f"DIVERGENCE [{nom}] seed={seed}\n  python: {python}\n  rust:   {rust}")
        print(f"scénario {nom:8s} : {identiques}/{args.graines[1] - args.graines[0]} identiques")
    print(f"TOTAL : {total - ecarts}/{total} identiques")
    print("PARITÉ BIT-À-BIT VÉRIFIÉE ✓" if ecarts == 0 else f"ÉCHEC : {ecarts} divergence(s)")
    return 0 if ecarts == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
