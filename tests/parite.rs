use collision_avoidance_rs::sim::{Params, Vec2, corriger_paire_cbf, simuler};

#[test]
fn cbf_ne_modifie_pas_une_paire_deja_sure() {
    let p_i = Vec2::new(-2.0, 0.0);
    let p_j = Vec2::new(2.0, 0.0);
    let v_i = Vec2::new(-1.0, 0.0);
    let v_j = Vec2::new(1.0, 0.0);
    let (a, b, active) = corriger_paire_cbf(p_i, p_j, v_i, v_j, 1.0, 2.0);
    assert!(!active);
    assert_eq!(a, v_i);
    assert_eq!(b, v_j);
}

#[test]
fn cbf_projette_une_paire_en_approche() {
    let p_i = Vec2::new(-1.0, 0.0);
    let p_j = Vec2::new(1.0, 0.0);
    let (a, b, active) = corriger_paire_cbf(
        p_i,
        p_j,
        Vec2::new(2.0, 0.0),
        Vec2::new(-2.0, 0.0),
        1.0,
        2.0,
    );
    let r = p_i - p_j;
    let lhs = 2.0 * r.dot(a - b) + 2.0 * (r.norm2() - 1.0);
    assert!(active);
    assert!(lhs >= -1e-12, "contrainte CBF violée : {lhs}");
}

#[test]
fn simulation_est_deterministe_et_compte_les_agents() {
    let params = Params::default();
    let a = simuler(1001, &params);
    let b = simuler(1001, &params);
    assert_eq!(a, b);
    assert_eq!(a.agents, params.agents);
}

#[test]
fn cbf_intervient_sur_les_trajectoires_croisees() {
    for scenario in 0..=2 {
        let resultat = simuler(
            1001,
            &Params {
                scenario,
                ..Params::default()
            },
        );
        assert!(
            resultat.interventions > 0,
            "scénario {scenario} sans intervention"
        );
        assert!(resultat.premier_temps_reaction.is_some());
        assert!(resultat.surcout_commande > 0.0);
    }
}

#[test]
fn aucune_collision_sur_le_balayage_nominal() {
    for seed in 1000..1050 {
        let resultat = simuler(seed, &Params::default());
        assert_eq!(resultat.collisions, 0, "collision pour seed {seed}");
    }
}

#[test]
fn metriques_finies_sur_scenario_dense() {
    for seed in 1000..1020 {
        let resultat = simuler(
            seed,
            &Params {
                scenario: 2,
                agents: 12,
                ..Params::default()
            },
        );
        assert!(resultat.distance_min_observee.is_finite());
        assert!(resultat.surcout_commande.is_finite());
        assert!(resultat.distance_min_observee > 0.0);
    }
}
