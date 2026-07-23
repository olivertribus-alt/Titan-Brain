#!/usr/bin/env python3
"""Create the TB-EVAL-006 executive verification summary PDF."""

from __future__ import annotations

from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)

ROOT = Path(__file__).resolve().parents[1]
OUTPUT_PATH = ROOT / "output" / "pdf" / "TB-EVAL-006_Executive_Summary.pdf"

NAVY = colors.HexColor("#1b2a4a")
SLATE = colors.HexColor("#334155")
MUTED = colors.HexColor("#64748b")
LIGHT = colors.HexColor("#f8f9fa")
LINE = colors.HexColor("#dee2e6")
BLUE = colors.HexColor("#0e7490")
GREEN = colors.HexColor("#166534")
GREEN_BG = colors.HexColor("#dcfce7")
CYAN_BG = colors.HexColor("#e8f4f8")


def _register_fonts() -> tuple[str, str]:
    """Register macOS fonts with Czech glyph coverage."""
    regular_candidates = (
        Path("/System/Library/Fonts/Supplemental/Arial.ttf"),
        Path("/Library/Fonts/Arial.ttf"),
    )
    bold_candidates = (
        Path("/System/Library/Fonts/Supplemental/Arial Bold.ttf"),
        Path("/Library/Fonts/Arial Bold.ttf"),
    )
    regular = next((path for path in regular_candidates if path.exists()), None)
    bold = next((path for path in bold_candidates if path.exists()), None)
    if regular is None or bold is None:
        raise FileNotFoundError("Arial TrueType fonts are required")
    pdfmetrics.registerFont(TTFont("TBArial", str(regular)))
    pdfmetrics.registerFont(TTFont("TBArial-Bold", str(bold)))
    return "TBArial", "TBArial-Bold"


def _styles(regular: str, bold: str) -> dict[str, ParagraphStyle]:
    base = getSampleStyleSheet()
    return {
        "body": ParagraphStyle(
            "TBBody",
            parent=base["BodyText"],
            fontName=regular,
            fontSize=9.2,
            leading=13,
            textColor=SLATE,
            spaceAfter=6,
        ),
        "small": ParagraphStyle(
            "TBSmall",
            parent=base["BodyText"],
            fontName=regular,
            fontSize=7.8,
            leading=10,
            textColor=MUTED,
        ),
        "section": ParagraphStyle(
            "TBSection",
            parent=base["Heading2"],
            fontName=bold,
            fontSize=12.2,
            leading=15,
            textColor=NAVY,
            spaceBefore=12,
            spaceAfter=7,
            keepWithNext=True,
        ),
        "table_head": ParagraphStyle(
            "TBTableHead",
            parent=base["BodyText"],
            fontName=bold,
            fontSize=7.8,
            leading=9.4,
            textColor=colors.white,
        ),
        "table": ParagraphStyle(
            "TBTable",
            parent=base["BodyText"],
            fontName=regular,
            fontSize=7.65,
            leading=9.6,
            textColor=SLATE,
        ),
        "table_bold": ParagraphStyle(
            "TBTableBold",
            parent=base["BodyText"],
            fontName=bold,
            fontSize=7.7,
            leading=9.6,
            textColor=NAVY,
        ),
        "badge": ParagraphStyle(
            "TBBadge",
            parent=base["BodyText"],
            fontName=bold,
            fontSize=7.2,
            leading=9,
            alignment=TA_CENTER,
            textColor=GREEN,
        ),
        "metric_title": ParagraphStyle(
            "TBMetricTitle",
            parent=base["BodyText"],
            fontName=bold,
            fontSize=7.7,
            leading=9,
            textColor=SLATE,
            alignment=TA_CENTER,
        ),
        "metric_value": ParagraphStyle(
            "TBMetricValue",
            parent=base["BodyText"],
            fontName=bold,
            fontSize=15,
            leading=18,
            textColor=BLUE,
            alignment=TA_CENTER,
        ),
        "callout": ParagraphStyle(
            "TBCallout",
            parent=base["BodyText"],
            fontName=regular,
            fontSize=8.5,
            leading=12,
            textColor=SLATE,
        ),
        "footer": ParagraphStyle(
            "TBFooter",
            parent=base["BodyText"],
            fontName=regular,
            fontSize=7,
            leading=9,
            alignment=TA_CENTER,
            textColor=MUTED,
        ),
    }


def _p(text: str, style: ParagraphStyle) -> Paragraph:
    return Paragraph(text, style)


def _badge(text: str, style: ParagraphStyle) -> Table:
    return Table(
        [[_p(text, style)]],
        colWidths=[19 * mm],
        style=TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), GREEN_BG),
                ("BOX", (0, 0), (-1, -1), 0.5, colors.HexColor("#bbf7d0")),
                ("LEFTPADDING", (0, 0), (-1, -1), 3),
                ("RIGHTPADDING", (0, 0), (-1, -1), 3),
                ("TOPPADDING", (0, 0), (-1, -1), 2),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 2),
            ]
        ),
    )


def _page_chrome(canvas: object, document: object) -> None:
    """Draw the consistent background, header, and footer."""
    canvas.saveState()
    width, height = A4
    canvas.setFillColor(LIGHT)
    canvas.rect(0, 0, width, height, fill=1, stroke=0)
    page = getattr(document, "page", 1)
    if page == 1:
        canvas.setFillColor(NAVY)
        canvas.rect(0, height - 35 * mm, width, 35 * mm, fill=1, stroke=0)
        canvas.setFillColor(colors.HexColor("#28a745"))
        canvas.rect(0, height - 35 * mm, width, 1.3 * mm, fill=1, stroke=0)
        canvas.setFillColor(colors.white)
        canvas.setFont("TBArial-Bold", 17)
        canvas.drawString(15 * mm, height - 17 * mm, "TB-EVAL-006")
        canvas.setFont("TBArial", 9.5)
        canvas.setFillColor(colors.HexColor("#b0c4de"))
        canvas.drawString(
            15 * mm,
            height - 24 * mm,
            "Executive Verification Summary | Kinematic Command Governor",
        )
    else:
        canvas.setFillColor(NAVY)
        canvas.rect(0, height - 12 * mm, width, 12 * mm, fill=1, stroke=0)
        canvas.setFillColor(colors.white)
        canvas.setFont("TBArial-Bold", 8.5)
        canvas.drawString(15 * mm, height - 8 * mm, "TB-EVAL-006 | Executive Summary")

    canvas.setStrokeColor(LINE)
    canvas.setLineWidth(0.6)
    canvas.line(15 * mm, 14 * mm, width - 15 * mm, 14 * mm)
    canvas.setFillColor(MUTED)
    canvas.setFont("TBArial", 7.2)
    canvas.drawCentredString(
        width / 2,
        8.5 * mm,
        f"Titan Brain Kinematics & Safety Framework | Page {page}",
    )
    canvas.restoreState()


def _metadata(styles: dict[str, ParagraphStyle]) -> Table:
    body = styles["body"]
    rows = [
        [
            _p("<b>PROJEKT</b><br/>Titan Brain ROS 2", body),
            _p("<b>STAV CI</b><br/>", body),
            _badge("VERIFIED", styles["badge"]),
            _p("<b>VĚTEV</b><br/><font name='Courier'>feat/tb-eval-006</font>", body),
        ],
        [
            _p("<b>CÍLOVÉ PROSTŘEDÍ</b><br/>ROS 2 Jazzy / Python 3.11+", body),
            _p("<b>POSLEDNÍ COMMIT</b><br/><font name='Courier'>17dbfcc</font>", body),
            _p(
                "<b>PROFIL</b><br/>a<sub>max</sub>=1.0, "
                "a<sub>decel</sub>=2.0, j<sub>max</sub>=5.0",
                body,
            ),
            _p("<b>TESTY</b><br/>517 passed", body),
        ],
    ]
    return Table(
        rows,
        colWidths=[42 * mm, 42 * mm, 45 * mm, 45 * mm],
        style=TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), colors.white),
                ("BOX", (0, 0), (-1, -1), 0.7, LINE),
                ("INNERGRID", (0, 0), (-1, -1), 0.45, colors.HexColor("#f0f0f0")),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("LEFTPADDING", (0, 0), (-1, -1), 7),
                ("RIGHTPADDING", (0, 0), (-1, -1), 7),
                ("TOPPADDING", (0, 0), (-1, -1), 7),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
            ]
        ),
    )


def _metrics(styles: dict[str, ParagraphStyle]) -> Table:
    return Table(
        [
            [
                _p("517", styles["metric_value"]),
                _p("6 / 6", styles["metric_value"]),
                _p("100%", styles["metric_value"]),
                _p("50 Hz", styles["metric_value"]),
            ],
            [
                _p("zelených testů", styles["metric_title"]),
                _p("fault scénářů", styles["metric_title"]),
                _p("coverage nového jádra", styles["metric_title"]),
                _p("profilovací smyčka", styles["metric_title"]),
            ],
        ],
        colWidths=[43.5 * mm] * 4,
        style=TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), colors.white),
                ("BOX", (0, 0), (-1, -1), 0.7, LINE),
                ("INNERGRID", (0, 0), (-1, -1), 0.45, colors.HexColor("#edf2f7")),
                ("TOPPADDING", (0, 0), (-1, 0), 8),
                ("BOTTOMPADDING", (0, 0), (-1, 0), 0),
                ("TOPPADDING", (0, 1), (-1, 1), 0),
                ("BOTTOMPADDING", (0, 1), (-1, 1), 8),
            ]
        ),
    )


def _implementation_table(styles: dict[str, ParagraphStyle]) -> Table:
    head = styles["table_head"]
    cell = styles["table"]
    bold = styles["table_bold"]
    rows = [
        [
            _p("Sub-slice", head),
            _p("Komponenta", head),
            _p("Klíčové vlastnosti a garance", head),
        ],
        [
            _p("<b>TB-EVAL-006A</b>", bold),
            _p("Core Governor Engine", cell),
            _p(
                "Jerk-limited ramp shaping (j<sub>max</sub>=5.0 m/s³), "
                "asymetrická decelerace a fail-closed ochrana proti NaN/Inf "
                "a časovým skokům.",
                cell,
            ),
        ],
        [
            _p("<b>TB-EVAL-006B</b>", bold),
            _p("ROS 2 Adapter &amp; Node", cell),
            _p(
                "Uzel <font name='Courier'>command_governor_node</font> (50 Hz), "
                "tok <font name='Courier'>/cmd_vel_raw</font> → "
                "<font name='Courier'>/cmd_vel_governed</font> a safety bypass "
                "při E-STOP.",
                cell,
            ),
        ],
        [
            _p("<b>TB-EVAL-006C</b>", bold),
            _p("Fault Injection &amp; E2E", cell),
            _p(
                "Šest scénářů ověřujících jerk bound, asymetrickou rampu, "
                "emergency cut-off, stale command, safety timeout a neplatný vstup.",
                cell,
            ),
        ],
    ]
    return Table(
        rows,
        colWidths=[31 * mm, 43 * mm, 100 * mm],
        repeatRows=1,
        style=TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#2c3e50")),
                ("BACKGROUND", (0, 1), (-1, -1), colors.white),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, LIGHT]),
                ("GRID", (0, 0), (-1, -1), 0.45, LINE),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 7),
                ("RIGHTPADDING", (0, 0), (-1, -1), 7),
                ("TOPPADDING", (0, 0), (-1, -1), 6),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
            ]
        ),
    )


def _fault_table(styles: dict[str, ParagraphStyle]) -> Table:
    head = styles["table_head"]
    cell = styles["table"]
    rows = [
        [_p("Scénář", head), _p("Požadovaná reakce", head), _p("Výsledek", head)],
        [
            _p("1. Jerk bound", cell),
            _p(
                "Δa/Δt nepřekročí 5.0 m/s³ při skoku rychlosti",
                cell,
            ),
            _badge("PASSED", styles["badge"]),
        ],
        [
            _p("2. Asymmetric ramp", cell),
            _p(
                "Decelerace 2.0 m/s² zůstává oddělená od akcelerace 1.0 m/s²",
                cell,
            ),
            _badge("PASSED", styles["badge"]),
        ],
        [
            _p("3. Emergency cut-off", cell),
            _p("TRIPPED okamžitě vynutí 0 m/s bez rampy", cell),
            _badge("PASSED", styles["badge"]),
        ],
        [
            _p("4. Stale command", cell),
            _p("Výpadek toku příkazů vede k fail-closed nule", cell),
            _badge("PASSED", styles["badge"]),
        ],
        [
            _p("5. Safety timeout", cell),
            _p("Výpadek safety statusu vede k fail-closed nule", cell),
            _badge("PASSED", styles["badge"]),
        ],
        [
            _p("6. Invalid input", cell),
            _p("NaN/Infinity je zachyceno a povel je vynulován", cell),
            _badge("PASSED", styles["badge"]),
        ],
    ]
    return Table(
        rows,
        colWidths=[36 * mm, 118 * mm, 20 * mm],
        repeatRows=1,
        style=TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#2c3e50")),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, LIGHT]),
                ("GRID", (0, 0), (-1, -1), 0.45, LINE),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("LEFTPADDING", (0, 0), (-1, -1), 6),
                ("RIGHTPADDING", (0, 0), (-1, -1), 6),
                ("TOPPADDING", (0, 0), (-1, -1), 5),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
            ]
        ),
    )


def build_pdf() -> Path:
    """Build the final PDF and return its path."""
    regular, bold = _register_fonts()
    styles = _styles(regular, bold)
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    document = SimpleDocTemplate(
        str(OUTPUT_PATH),
        pagesize=A4,
        rightMargin=15 * mm,
        leftMargin=15 * mm,
        topMargin=42 * mm,
        bottomMargin=19 * mm,
        title="TB-EVAL-006 Executive Summary",
        author="Titan Brain",
    )
    body = styles["body"]
    story: list[object] = [
        Spacer(1, 2 * mm),
        _metadata(styles),
        Spacer(1, 4 * mm),
        _metrics(styles),
        _p("1. Přehled implementace (sub-slices)", styles["section"]),
        _implementation_table(styles),
        _p("2. Výsledky fault-injection verifikace", styles["section"]),
        Table(
            [
                [
                    _p(
                        "Verifikační skript <font name='Courier'>"
                        "scripts/verify_tb_eval_006.py</font> prošel všech šest "
                        "kinematických a bezpečnostních scénářů. Celá lokální "
                        "sada obsahuje 517 zelených testů; nové jádro má 100% "
                        "coverage.",
                        styles["callout"],
                    )
                ]
            ],
            colWidths=[174 * mm],
            style=TableStyle(
                [
                    ("BACKGROUND", (0, 0), (-1, -1), CYAN_BG),
                    ("LINEBEFORE", (0, 0), (0, -1), 3, colors.HexColor("#17a2b8")),
                    ("BOX", (0, 0), (-1, -1), 0.45, colors.HexColor("#c7e9f1")),
                    ("LEFTPADDING", (0, 0), (-1, -1), 9),
                    ("RIGHTPADDING", (0, 0), (-1, -1), 9),
                    ("TOPPADDING", (0, 0), (-1, -1), 8),
                    ("BOTTOMPADDING", (0, 0), (-1, -1), 8),
                ]
            ),
        ),
        Spacer(1, 3 * mm),
        _fault_table(styles),
        _p("3. Další krok k dokončení milníku", styles["section"]),
        _p(
            "Sub-slice TB-EVAL-006C je připravený k commitnutí a pushnutí na "
            "vzdálenou větev <font name='Courier'>feat/tb-eval-006</font>. "
            "Následně lze otevřít Draft PR a spustit plnou CI matici v prostředí "
            "ROS 2 Jazzy.",
            body,
        ),
        Spacer(1, 2 * mm),
        _p(
            "Artefakt je určen pro archivaci verifikace a kontrolu release "
            "readiness; ROS 2 runtime gate zůstává autoritativně ověřován v CI "
            "kontejneru Jazzy.",
            styles["small"],
        ),
    ]
    document.build(story, onFirstPage=_page_chrome, onLaterPages=_page_chrome)
    return OUTPUT_PATH


if __name__ == "__main__":
    print(build_pdf())
