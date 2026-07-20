# TB-PoC-001 — Safety Decision Loop

## Status

**Implementováno a technicky otestováno.** PoC není bezpečnostně certifikován
ani uživatelsky validován.

## Produktová hypotéza

Titan Brain dokáže z jednoho striktně validovaného pozorování deterministicky
odvodit bezpečnostní rozhodnutí, zachovat jeho kauzální evidenci, atomicky
publikovat incident a následně jej rekonstruovat pomocí `tb replay`.

## Spustitelný tok

```text
SafetyObservation JSON
        |
        v
evaluate_safety()            čistá deterministická funkce
        |
        v
DecisionEvidence v0.2
        |
        +-- proceed ---------- bez incidentu
        |
        +-- stop ------------ FileIncidentStore (atomická publikace)
                                  |
                                  v
                          tb replay --store
```

## Vstupní kontrakt

`SafetyObservation` vyžaduje:

- nezáporný `timestamp_ns`,
- neprázdný `map_id`, `frame_id` a `sensor_id`,
- konečnou `Pose2D`,
- konečný a nezáporný `clearance_m`,
- konečnou `confidence` v intervalu `[0.0, 1.0]`.

Model je striktní, neměnný, odmítá neznámá pole, `NaN` a nekonečna.

## Normativní rozhodovací tabulka

| Podmínka | Akce | Pravidlo | Uložit incident |
| --- | --- | --- | --- |
| `clearance_m < 0.50` a `confidence >= 0.70` | `emergency_stop` | `EV-SAFE-01` | ano |
| `clearance_m < 0.50` a `confidence < 0.70` | `protective_stop` | `EV-SAFE-02` | ano |
| `clearance_m >= 0.50` | `proceed` | `EV-SAFE-00` | ne |

Hranice `clearance_m == 0.50` znamená `proceed`. Hranice
`confidence == 0.70` je dostatečná pro `EV-SAFE-01`.

## Determinismus

Identifikátor rozhodnutí je odvozen ze SHA-256 kanonického JSONu vstupního
pozorování a verze konfigurace pravidel. Stejný vstup a stejná konfigurace
vytvoří shodný `SafetyDecisionResult`, `DecisionEvidence` a identifikátor.

Vyhodnocení a persistence jsou oddělené. `evaluate_safety()` nemá vedlejší
efekty; `run_safety_decision_loop()` ukládá pouze incidenty.

## Atomicita a konflikty

`FileIncidentStore` nejprve zapíše a synchronizuje dočasný soubor ve stejném
filesystemu. Hotový inode publikuje atomickým hard linkem. Existující totožný
incident je idempotentní. Stejné ID s jiným obsahem vyvolá
`IncidentConflictError`; data nejsou tiše přepsána.

## CLI

Vyhodnocení a případné uložení:

```console
tb safety-evaluate observation.json --store ./incidents
```

Rekonstrukce vygenerovaného incidentu:

```console
tb replay <decision_id> --store ./incidents
```

Neplatný vstup končí kontrolovaným kódem `2` bez uložení incidentu.
Rozhodnutí o fyzickém fail-safe chování při nevalidním transportním vstupu
patří budoucímu hardwarovému adaptéru; PoC neposílá povely aktuátorům.

## Ověřené scénáře

- emergency stop,
- bezpečné `proceed` bez persistence,
- low-confidence protective stop,
- obě prahové hranice,
- odmítnutí `NaN`, nekonečen, záporného času a neúplných dat,
- bitově shodný obsah a SHA-256 hash ve 100 opakovaných vyhodnoceních,
- idempotentní uložení a odmítnutí konfliktu,
- end-to-end `SafetyObservation -> DecisionEvidence -> store -> tb replay`,
- kontrolovaná CLI chyba bez neobsloužené výjimky.

## Mimo rozsah

- ROS 2 nebo jiný hardware adapter,
- odesílání povelů aktuátorům,
- real-time garance,
- bezpečnostní certifikace,
- World Model, Memory System a Planning Engine,
- lidská validační studie.
