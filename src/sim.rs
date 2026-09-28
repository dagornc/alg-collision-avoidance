//! Simulateur déterministe d'évitement local par Control Barrier Function.

use crate::rng::PyRandom;
use std::ops::{Add, AddAssign, Mul, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y
    }
    pub fn norm2(self) -> f64 {
        self.dot(self)
    }
}

impl Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}
impl Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}
impl Mul<f64> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.x * rhs, self.y * rhs)
    }
}
impl AddAssign for Vec2 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}
impl SubAssign for Vec2 {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

/// Projette deux vitesses sur le demi-espace CBF
/// `2 r·(v_i-v_j) + alpha (||r||²-d_min²) >= 0`.
/// La correction symétrique minimise la norme quadratique jointe.
pub fn corriger_paire_cbf(
    p_i: Vec2,
    p_j: Vec2,
    v_i: Vec2,
    v_j: Vec2,
    distance_min: f64,
    alpha: f64,
) -> (Vec2, Vec2, bool) {
    let mut r = p_i - p_j;
    let mut r2 = r.norm2();
    if r2 == 0.0 {
        r = Vec2::new(distance_min, 0.0);
        r2 = r.norm2();
    }
    let h = (p_i - p_j).norm2() - distance_min * distance_min;
    let lhs = 2.0 * r.dot(v_i - v_j) + alpha * h;
    if lhs >= 0.0 {
        return (v_i, v_j, false);
    }
    let lambda = -lhs / (4.0 * r2);
    (v_i + r * lambda, v_j - r * lambda, true)
}

#[derive(Clone, Debug, PartialEq)]
pub struct Params {
    pub scenario: u8,
    pub agents: usize,
    pub steps: usize,
    pub dt: f64,
    pub distance_min: f64,
    pub alpha: f64,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            scenario: 0,
            agents: 8,
            steps: 80,
            dt: 0.1,
            distance_min: 1.0,
            alpha: 2.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Resultat {
    pub agents: usize,
    pub collisions: u64,
    pub quasi_collisions: u64,
    pub interventions: u64,
    pub premier_temps_reaction: Option<usize>,
    pub distance_min_observee: f64,
    pub surcout_commande: f64,
}

/// Simule des paires d'agents sur des trajectoires de croisement. Le scénario
/// règle l'espacement des couloirs (0 nominal, 1 dense, 2 stress).
pub fn simuler(seed: u64, params: &Params) -> Resultat {
    assert!(params.agents >= 2, "au moins deux agents requis");
    assert!(params.steps > 0, "horizon nul");
    assert!(params.dt > 0.0, "pas de temps non positif");
    assert!(params.distance_min > 0.0, "distance minimale non positive");
    assert!(params.alpha > 0.0, "alpha non positif");

    let mut rng = PyRandom::new(seed);
    let espacement = match params.scenario {
        0 => 1.6,
        1 => 1.1,
        _ => 0.85,
    };
    let mut positions = Vec::with_capacity(params.agents);
    let mut consignes = Vec::with_capacity(params.agents);
    for i in 0..params.agents {
        let gauche = i % 2 == 0;
        let paire = (i / 2) as f64;
        let x_abs = 4.0 + 2.0 * rng.random();
        let x = if gauche { -x_abs } else { x_abs };
        let y = paire * espacement + (rng.random() - 0.5) * 0.08;
        let vitesse = 0.8 + 0.4 * rng.random();
        positions.push(Vec2::new(x, y));
        consignes.push(Vec2::new(if gauche { vitesse } else { -vitesse }, 0.0));
    }

    let mut collisions = 0u64;
    let mut quasi_collisions = 0u64;
    let mut interventions = 0u64;
    let mut premier_temps_reaction = None;
    let mut distance_min_observee = f64::INFINITY;
    let mut surcout_commande = 0.0;

    for etape in 0..params.steps {
        let mut vitesses = consignes.clone();
        // Projections successives : quatre passes réduisent les violations
        // réintroduites par les contraintes voisines, sans solveur externe.
        for passe in 0..4 {
            for i in 0..params.agents {
                for j in (i + 1)..params.agents {
                    let (vi, vj, active) = corriger_paire_cbf(
                        positions[i],
                        positions[j],
                        vitesses[i],
                        vitesses[j],
                        params.distance_min,
                        params.alpha,
                    );
                    vitesses[i] = vi;
                    vitesses[j] = vj;
                    if passe == 0 && active {
                        interventions += 1;
                        if premier_temps_reaction.is_none() {
                            premier_temps_reaction = Some(etape);
                        }
                    }
                }
            }
        }
        for i in 0..params.agents {
            let correction = vitesses[i] - consignes[i];
            surcout_commande += correction.norm2().sqrt() * params.dt;
            positions[i] += vitesses[i] * params.dt;
        }
        for i in 0..params.agents {
            for j in (i + 1)..params.agents {
                let distance = (positions[i] - positions[j]).norm2().sqrt();
                if distance < distance_min_observee {
                    distance_min_observee = distance;
                }
                if distance < params.distance_min * 0.5 {
                    collisions += 1;
                } else if distance < params.distance_min * 1.25 {
                    quasi_collisions += 1;
                }
            }
        }
    }

    Resultat {
        agents: params.agents,
        collisions,
        quasi_collisions,
        interventions,
        premier_temps_reaction,
        distance_min_observee,
        surcout_commande,
    }
}
