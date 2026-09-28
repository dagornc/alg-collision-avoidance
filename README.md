# ALG_COLLISION_AVOIDANCE — portage Rust

Implémentation Rust déterministe d'une couche locale d'évitement de collision par Control Barrier Function (CBF), avec référence Python normative en miroir.

## Ce qui est implémenté

- Modèle cinématique 2D à intégrateur simple, local et indépendant du réseau.
- Trajectoires de croisement déterministes pour 3 densités (`scenario` 0, 1 ou 2).
- Barrière pair-à-pair `h = ||p_i-p_j||² - d_min²`.
- Projection symétrique de correction minimale sur la contrainte
  `2 r·(v_i-v_j) + alpha h >= 0`.
- Quatre passes de projections séquentielles par pas de temps, sans solveur ni dépendance externe.
- Métriques CSV : collisions, quasi-collisions, interventions, premier temps de réaction, distance minimale observée et surcoût de commande.

Interface identique des deux exécutables :

```text
collision_avoidance_rs <seed> [scenario] [agents] [steps] [distance_min]
sim_collision_avoidance.py <seed> [scenario] [agents] [steps] [distance_min]
```

## Ce qui est vérifié

Résultats mesurés dans l'environnement de publication :

- `cargo build --release` : succès ;
- `cargo test` : 10 tests passés, 0 échec (4 unitaires RNG + 6 intégration/invariants) ;
- parité CSV bit-à-bit : 150/150 comparaisons identiques, 0 écart ;
- matrice de parité : 50 graines (`1000..1050`) × 3 scénarios (`nominal`, `dense`, `stress`).

Rejeu :

```sh
cargo test
cargo build --release
python3 verify_parite_rust.py
```

## Ce qui ne l'est pas

- Il s'agit d'une implémentation de référence simplifiée, pas d'un contrôleur de vol certifié.
- Le modèle ne représente ni dynamique de quadrirotor, ni retard capteur/actionneur, ni incertitude d'estimation, ni obstacles statiques.
- Les projections séquentielles approchent le problème multi-contraintes ; elles ne remplacent pas un QP CBF global avec preuve de faisabilité.
- La métrique `collisions` utilise le seuil sévère `0.5 × distance_min`; les quasi-collisions couvrent jusqu'à `1.25 × distance_min`.
- L'absence de collision vérifiée par test concerne 50 graines du scénario nominal simulé seulement. Elle ne constitue pas une validation matérielle ou scientifique.
- Le catalogue système classe l'évidence `LOW` et maintient ouvert le GAP-2 (CBF hétérogène au-delà de 30 plateformes en matériel réel).

Statut : implémentation de référence simplifiée ; parité logicielle bit-à-bit vérifiée, validation opérationnelle non réalisée.

## Licence

MIT — voir [LICENSE](LICENSE).
