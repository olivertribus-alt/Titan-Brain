# Validace Titan Brain v0.2

## Stav ověření

Titan Brain v0.2 je definován jako **prostorově a časově ukotvená historie
rozhodování robota**.

K 20. 7. 2026 není pravidlo 60 sekund experimentálně ověřeno. V dostupném
projektu je implementována verze 0.2.0, příkaz `tb replay` a referenční dataset
`incident_042`. Kontrakty `Pose2D`, `SpatialContext`, `EvidenceValue` a
`DecisionEvidence` zahrnují migraci z v0.1.

Tento dokument proto obsahuje:

1. současný heuristický audit implementace,
2. protokol budoucího testu s člověkem,
3. formulář pro záznam skutečného chování.

Simulace ani heuristický audit se nesmí vykazovat jako uživatelský test.

## Heuristický audit současné implementace

### Co lze doložit

- Projekt deklaruje verzi `0.2.0`.
- Existuje omezená in-memory historie dokončených dispatchů (`TraceRegistry`).
- Jeden záznam obsahuje technická pole jako `message_id`, `trace_id`, typ payloadu,
  cíle, latenci a selhání doručení.
- Loader migruje legacy schema v0.1 do strukturovaného schema v0.2.
- Chybějící prostorový kontext je `None`; nulové hodnoty zůstávají `0.0`.
- Kontrakt odmítá `NaN`, nekonečna a prázdné textové identifikátory.
- Dataset `incident_042` prochází kontraktem v0.2.
- Integrační test fixuje výstup `tb replay incident_042` znak po znaku.
- `TB-PoC-001` generuje `DecisionEvidence` z validovaného bezpečnostního
  pozorování a atomicky publikuje pouze stop incidenty.
- Vygenerovaný incident lze načíst přes `tb replay --store` bez ztráty dat.

### Co zatím nelze ověřit

- lidskou srozumitelnost nebo pořadí informací ve výstupu `tb replay`,
- uživatelskou srozumitelnost zpětné kompatibility starších incidentů,
- lidskou interpretaci `expected`, `observed`, `threshold` a `unit`,
- zobrazení `frame_id` a polohy,
- skrytí interních ID a verzí za `--verbose`,
- diagnózu incidentu do 60 sekund.
- chování hardwarového adaptéru a aktuátorů,
- real-time a certifikační vlastnosti Safety Decision Loop.

### Vstupní podmínky pro test

Test lze spustit teprve tehdy, když:

- `tb replay incident_042` funguje v čistém terminálu,
- incident má jednu předem zapsanou správnou příčinu,
- výstup obsahuje dost dat k odvození příčiny i vztahu k prahu,
- testující nezná incident ani význam jednotlivých polí předem,
- pozorovatel zná správnou odpověď a nesmí testujícímu napovídat.

## Pravidlo 60 sekund

Test je úspěšný pouze tehdy, když testující do 60 sekund od zobrazení
výstupu:

1. správně pojmenuje příčinu incidentu,
2. vysvětlí ji pomocí pozorované hodnoty a prahové hodnoty,
3. nezeptá se na význam evidence ani jednotek.

Všechny tři podmínky jsou povinné. Neúspěch není selhání
testujícího; je to zjištění o rozhraní.

## Nábor vhodného testujícího

Konkrétní „Petr“ není nutný. Stačí jeden člověk, který:

- má základní zkušenost s robotikou, provozní diagnostikou nebo telemetrií,
- nepodílel se na návrhu výstupu,
- dosud neviděl `incident_042`,
- je ochoten nahlas popisovat své uvažování.

Autor rozhraní není vhodným testujícím, protože už zná správnou
interpretaci.

## Instrukce pro testujícího

Pozorovatel přečte pouze tento text:

> V terminálu spusť `tb replay incident_042`. Zjisti, co bylo příčinou
> incidentu a proč k tomu došlo. Při práci prosím nahlas říkej, na co se
> díváš a co si myslíš. Až budeš mít diagnózu, vyslov ji nahlas.

Poté pozorovatel spustí časomíru. Neposkytuje vysvětlení, nepotvrzuje
dílčí hypotézy a neklade naváděcí otázky.

Pokud se testující zeptá na význam pole, pozorovatel odpoví pouze:

> Pokračuj prosím podle informací, které máš k dispozici.

Otázku zapíše doslova.

## Záznamový formulář

- Datum a čas:
- Testující (anonymní označení):
- Relevantní zkušenost testujícího:
- Verze nebo commit aplikace:
- Hash nebo verze dat incidentu:
- Velikost a nastavení terminálu:

### Pozorování

- Čas do první správné hypotézy:
- Čas do konečné diagnózy:
- Konečná diagnóza (doslova):
- Vysvětlení vztahu k prahu (doslova):
- První otevřený nástroj nebo zdroj:
- Doplňující otázky (doslova a v pořadí):
- Ignorované části výstupu:
- Informace, které testujícímu chyběly:
- Jiné pozorované zaváhání:

### Verdikt

- [ ] Správná příčina
- [ ] Správné vysvětlení pomocí pozorované hodnoty a prahu
- [ ] Bez dotazu na význam evidence nebo jednotek
- [ ] Dokončeno do 60 sekund
- [ ] Test proběhl bez nápovědy

**Celkový výsledek:** PROŠEL / NEPROŠEL

**Jedna nejdůležitější zjištěná slabina:**

## Rozhodování po testu

- Pokud jsou splněny všechny podmínky, lze tvrdit pouze to, že daný
  testující diagnostikoval daný incident za daných podmínek do 60 sekund.
- Pokud podmínka splněna není, upraví se nejmenší část kontraktu nebo CLI,
  která přímo odpovídá pozorovanému problému.
- Po změně se použije jiný testující nebo jiný, ekvivalentně obtížný
  incident. Opakování stejného incidentu se stejným člověkem měří i
  zapamatování, ne pouze kvalitu rozhraní.

## Dočasný verdikt v0.2

Dokud není proveden zaznamenaný test se skutečným člověkem, správné
označení stavu je:

> **Titan Brain v0.2: implementováno, technicky otestováno a připraveno k
> lidské validaci; uživatelsky neověřeno.**
