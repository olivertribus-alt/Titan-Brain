# TB-CI-001 — GitHub Actions Quality Gate

## Status

**Workflow je lokálně implementován a jeho příkazy procházejí; aktivace na
GitHubu dosud neproběhla.**

Současný projektový adresář není Git repozitář a nemá GitHub remote.
Soubor `.github/workflows/ci.yml` je připraven k vložení do repozitáře, ale
bez GitHub remote a skutečného Actions běhu se nevydává za aktivní CI.

## Cíl

Při každém pull requestu a pushi do výchozí větve automaticky ověřit:

1. podporované verze Pythonu,
2. celou testovací sadu,
3. striktní typovou kontrolu,
4. lint celého zdrojového a testovacího stromu.

## Předpoklady

- Projekt je v kořeni Git repozitáře.
- Je znám název výchozí větve.
- GitHub Actions jsou pro repozitář povoleny.
- Nález `UP035` v `core/bus.py` je opraven bez globálního `ignore`.

## Workflow

Umístění: `.github/workflows/ci.yml`

Implementované triggery:

- `pull_request` pro libovolnou cílovou větev,
- `push` do libovolné větve,
- `workflow_dispatch` pro ruční ověření.

Minimální oprávnění:

```yaml
permissions:
  contents: read
```

Workflow použije `actions/checkout@v6`, `actions/setup-python@v6` a jeden job s
maticí Pythonu `3.11` a `3.12`, protože projekt deklaruje
`requires-python = ">=3.11"` a lokálně je ověřován na Pythonu 3.12.

Kroky jobu:

```console
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
python -m ruff check core tests
python -m mypy core tests
python -m pytest -q --cov=core --cov-report=term-missing
```

Job má mít nastavený timeout, pip cache a concurrency group, která zruší
zastaralý běh stejného pull requestu.

## Lokální baseline k 20. 7. 2026

- `python -m ruff check core tests`: prošlo bez ignorovaných pravidel,
- `python -m mypy core tests`: prošlo ve striktním režimu,
- `python -m pytest -q --cov=core --cov-report=term-missing`: 59 testů prošlo,
- celkové statement coverage: **87 %**,
- `core/safety.py`: **100 %** statement coverage.

Coverage je v této fázi diagnostická metrika, nikoliv quality gate s minimálním
procentem.

## Definition of Done

- [ ] Workflow se spustí na testovacím pull requestu.
- [ ] Obě verze Pythonu projdou všemi kontrolami.
- Úmyslně rozbitý test zablokuje workflow.
- Typová chyba zablokuje workflow.
- Lint chyba v libovolném souboru pod `core/` nebo `tests/` zablokuje workflow.
- Workflow nevyžaduje zapisovací oprávnění ani secrets.
- [ ] Ochrana výchozí větve vyžaduje úspěšný CI check před merge.

## Záměrně odloženo

- Coverage threshold; workflow coverage pouze změří a vypíše.
- Uzamčení přesných verzí vývojových závislostí.
- Publikování balíčku.
- Podepisování artefaktů.
- Simulátor, ROS 2 a hardware-in-the-loop joby.
- Automatické nasazení.
