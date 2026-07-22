"""Create the TB-SAFE-001 executive summary PDF."""

from __future__ import annotations

import shutil
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Flowable,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)

ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "output" / "pdf"
OUTPUT_PDF = OUTPUT_DIR / "TB-SAFE-001_Executive_Summary.pdf"
COMPAT_OUTPUT_PDF = ROOT / "output" / "TB-SAFE-001_Executive_Summary.pdf"

NAVY = colors.HexColor("#1B2A4A")
SLATE = colors.HexColor("#334155")
MUTED = colors.HexColor("#64748B")
LIGHT = colors.HexColor("#F8F9FA")
PALE_BLUE = colors.HexColor("#E8F4F8")
LINE = colors.HexColor("#DEE2E6")
GREEN = colors.HexColor("#1F8A4C")
RED = colors.HexColor("#C0392B")
WHITE = colors.white


class Banner(Flowable):
    """Draw a compact full-width title banner."""

    def __init__(self, width: float) -> None:
        super().__init__()
        self.width = width
        self.height = 29 * mm

    def draw(self) -> None:
        canvas = self.canv
        canvas.saveState()
        canvas.setFillColor(NAVY)
        canvas.rect(0, 0, self.width, self.height, fill=1, stroke=0)
        canvas.setFillColor(colors.HexColor("#D9534F"))
        canvas.rect(0, 0, self.width, 1.4 * mm, fill=1, stroke=0)
        canvas.setFillColor(WHITE)
        canvas.setFont("TBArial-Bold", 17)
        canvas.drawString(10 * mm, self.height - 12 * mm, "TB-SAFE-001")
        canvas.setFont("TBArial", 10)
        canvas.setFillColor(colors.HexColor("#B0C4DE"))
        canvas.drawString(
            10 * mm,
            self.height - 20 * mm,
            "External Safety Loop Supervisor",
        )
        canvas.restoreState()


def _register_fonts() -> None:
    regular = Path("/System/Library/Fonts/Supplemental/Arial.ttf")
    bold = Path("/System/Library/Fonts/Supplemental/Arial Bold.ttf")
    if not regular.is_file() or not bold.is_file():
        raise FileNotFoundError("Arial fonts with Czech glyph coverage are required")
    pdfmetrics.registerFont(TTFont("TBArial", str(regular)))
    pdfmetrics.registerFont(TTFont("TBArial-Bold", str(bold)))


def _styles() -> dict[str, ParagraphStyle]:
    base = getSampleStyleSheet()
    return {
        "body": ParagraphStyle(
            "TBBody",
            parent=base["BodyText"],
            fontName="TBArial",
            fontSize=9.3,
            leading=13,
            textColor=SLATE,
            spaceAfter=5,
        ),
        "small": ParagraphStyle(
            "TBSmall",
            parent=base["BodyText"],
            fontName="TBArial",
            fontSize=8.1,
            leading=10.5,
            textColor=MUTED,
        ),
        "heading": ParagraphStyle(
            "TBHeading",
            parent=base["Heading2"],
            fontName="TBArial-Bold",
            fontSize=12.5,
            leading=15,
            textColor=NAVY,
            spaceBefore=9,
            spaceAfter=7,
        ),
        "table": ParagraphStyle(
            "TBTable",
            parent=base["BodyText"],
            fontName="TBArial",
            fontSize=8.1,
            leading=10.2,
            textColor=SLATE,
        ),
        "table_bold": ParagraphStyle(
            "TBTableBold",
            parent=base["BodyText"],
            fontName="TBArial-Bold",
            fontSize=8.1,
            leading=10.2,
            textColor=SLATE,
        ),
        "table_header": ParagraphStyle(
            "TBTableHeader",
            parent=base["BodyText"],
            fontName="TBArial-Bold",
            fontSize=8.2,
            leading=10.2,
            textColor=WHITE,
        ),
        "meta_label": ParagraphStyle(
            "TBMetaLabel",
            parent=base["BodyText"],
            fontName="TBArial-Bold",
            fontSize=8.2,
            leading=10,
            textColor=SLATE,
        ),
        "meta_value": ParagraphStyle(
            "TBMetaValue",
            parent=base["BodyText"],
            fontName="TBArial",
            fontSize=8.2,
            leading=10,
            textColor=colors.HexColor("#111111"),
        ),
        "center": ParagraphStyle(
            "TBCenter",
            parent=base["BodyText"],
            fontName="TBArial-Bold",
            fontSize=8.2,
            leading=10,
            alignment=TA_CENTER,
            textColor=GREEN,
        ),
    }


def _p(text: str, style: ParagraphStyle) -> Paragraph:
    return Paragraph(text, style)


def _meta_table(styles: dict[str, ParagraphStyle]) -> Table:
    label = styles["meta_label"]
    value = styles["meta_value"]
    rows = [
        [
            _p("Projekt", label),
            _p("Titan Brain ROS 2", value),
            _p("Stav CI", label),
            _p("Lokální matice ověřena; CI pending", value),
        ],
        [
            _p("Větev", label),
            _p("feat/tb-safe-001", value),
            _p("Cílové prostředí", label),
            _p("ROS 2 Jazzy / Python 3.11+", value),
        ],
        [
            _p("Poslední commit", label),
            _p("ac8703e (TB-SAFE-001C)", value),
            _p("Profil", label),
            _p("Fail-closed + hardware fault latch", value),
        ],
    ]
    table = Table(rows, colWidths=[25 * mm, 48 * mm, 25 * mm, 76 * mm])
    table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (0, -1), colors.HexColor("#F1F3F5")),
                ("BACKGROUND", (2, 0), (2, -1), colors.HexColor("#F1F3F5")),
                ("GRID", (0, 0), (-1, -1), 0.45, LINE),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("LEFTPADDING", (0, 0), (-1, -1), 7),
                ("RIGHTPADDING", (0, 0), (-1, -1), 7),
                ("TOPPADDING", (0, 0), (-1, -1), 6),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
            ]
        )
    )
    return table


def _implementation_table(styles: dict[str, ParagraphStyle]) -> Table:
    table_style = styles["table"]
    bold = styles["table_bold"]
    header = styles["table_header"]
    rows = [
        [
            _p("Sub-slice", header),
            _p("Komponenta", header),
            _p("Klíčové vlastnosti a garance", header),
        ],
        [
            _p("TB-SAFE-001A", bold),
            _p("Core Supervisor", table_style),
            _p(
                "Per-channel 200 ms heartbeat timeout, monotónní hodiny a "
                "fail-closed reléový kontrakt.",
                table_style,
            ),
        ],
        [
            _p("TB-SAFE-001B", bold),
            _p("Relay Feedback and Latch", table_style),
            _p(
                "50 ms přechodové okno, detekce svařených kontaktů, sticky "
                "HARDWARE_FAULT_LATCH a autorizovaný reset.",
                table_style,
            ),
        ],
        [
            _p("TB-SAFE-001C", bold),
            _p("ROS 2 Adapter and Messages", table_style),
            _p(
                "100 Hz safety_loop_supervisor_node, tři ROS kontrakty a "
                "idempotentní Jazzy teardown.",
                table_style,
            ),
        ],
        [
            _p("TB-SAFE-001D", bold),
            _p("Fault Injection and E2E Prep", table_style),
            _p(
                "Deterministická emulace relé a šest fault scénářů s "
                "auditním výsledkem state/reason.",
                table_style,
            ),
        ],
    ]
    table = Table(rows, colWidths=[31 * mm, 47 * mm, 96 * mm], repeatRows=1)
    table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#2C3E50")),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [WHITE, LIGHT]),
                ("GRID", (0, 0), (-1, -1), 0.45, LINE),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 7),
                ("RIGHTPADDING", (0, 0), (-1, -1), 7),
                ("TOPPADDING", (0, 0), (-1, -1), 7),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
            ]
        )
    )
    return table


def _fault_table(styles: dict[str, ParagraphStyle]) -> Table:
    table_style = styles["table"]
    header = styles["table_header"]
    center = styles["center"]
    rows = [
        [
            _p("Poruchový scénář", header),
            _p("Požadovaná reakce", header),
            _p("Výsledek", header),
        ],
        [
            _p("1. Missing heartbeat", table_style),
            _p("Initialization timeout; relay open request", table_style),
            _p("PASSED", center),
        ],
        [
            _p("2. Stale heartbeat", table_style),
            _p("Heartbeat timeout; transition to TRIPPED", table_style),
            _p("PASSED", center),
        ],
        [
            _p("3. Welded relay contacts", table_style),
            _p("WELDED_CONTACTS; permanent hardware latch", table_style),
            _p("PASSED", center),
        ],
        [
            _p("4. Clock regression", table_style),
            _p("CLOCK_REGRESSION; no stale evidence accepted", table_style),
            _p("PASSED", center),
        ],
        [
            _p("5. Sequence replay", table_style),
            _p("HEARTBEAT_ERROR; fail-closed trip", table_style),
            _p("PASSED", center),
        ],
        [
            _p("6. Unauthorized reset", table_style),
            _p("RESET_REJECTED; latch remains active", table_style),
            _p("PASSED", center),
        ],
    ]
    table = Table(rows, colWidths=[49 * mm, 99 * mm, 26 * mm], repeatRows=1)
    table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#2C3E50")),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [WHITE, LIGHT]),
                ("GRID", (0, 0), (-1, -1), 0.45, LINE),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("LEFTPADDING", (0, 0), (-1, -1), 7),
                ("RIGHTPADDING", (0, 0), (-1, -1), 7),
                ("TOPPADDING", (0, 0), (-1, -1), 7),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
            ]
        )
    )
    return table


def _footer(canvas: object, document: object) -> None:
    """Draw page numbering and archival footer."""
    canvas.saveState()  # type: ignore[attr-defined]
    canvas.setStrokeColor(LINE)  # type: ignore[attr-defined]
    canvas.line(12 * mm, 11 * mm, A4[0] - 12 * mm, 11 * mm)  # type: ignore[attr-defined]
    canvas.setFont("TBArial", 7.5)  # type: ignore[attr-defined]
    canvas.setFillColor(MUTED)  # type: ignore[attr-defined]
    canvas.drawString(12 * mm, 6 * mm, "Titan Brain Safety Framework")  # type: ignore[attr-defined]
    canvas.drawRightString(  # type: ignore[attr-defined]
        A4[0] - 12 * mm,
        6 * mm,
        f"TB-SAFE-001 | strana {document.page}",  # type: ignore[attr-defined]
    )
    canvas.restoreState()  # type: ignore[attr-defined]


def build_pdf() -> Path:
    """Build and return the canonical executive-summary PDF path."""
    _register_fonts()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    styles = _styles()
    document = SimpleDocTemplate(
        str(OUTPUT_PDF),
        pagesize=A4,
        rightMargin=12 * mm,
        leftMargin=12 * mm,
        topMargin=15 * mm,
        bottomMargin=16 * mm,
        title="TB-SAFE-001 Executive Summary",
        author="Titan Brain Safety Framework",
    )
    usable_width = A4[0] - document.leftMargin - document.rightMargin
    story: list[Flowable] = [
        Banner(usable_width),
        Spacer(1, 6 * mm),
        _meta_table(styles),
        Spacer(1, 5 * mm),
        _p("1. Přehled implementace", styles["heading"]),
        _implementation_table(styles),
        PageBreak(),
        _p("2. Fault-injection verifikace", styles["heading"]),
        Table(
            [
                [
                    _p(
                        "Lokální verifikační skript prošel všech šest "
                        "simulovaných poruchových stavů.",
                        styles["body"],
                    )
                ]
            ],
            colWidths=[usable_width],
            style=TableStyle(
                [
                    ("BACKGROUND", (0, 0), (-1, -1), PALE_BLUE),
                    ("LINEBEFORE", (0, 0), (0, -1), 3, colors.HexColor("#17A2B8")),
                    ("BOX", (0, 0), (-1, -1), 0.5, colors.HexColor("#B8DDE5")),
                    ("LEFTPADDING", (0, 0), (-1, -1), 10),
                    ("RIGHTPADDING", (0, 0), (-1, -1), 10),
                    ("TOPPADDING", (0, 0), (-1, -1), 8),
                    ("BOTTOMPADDING", (0, 0), (-1, -1), 8),
                ]
            ),
        ),
        Spacer(1, 5 * mm),
        _fault_table(styles),
        Spacer(1, 5 * mm),
        _p("3. Stav a další krok", styles["heading"]),
        _p(
            "TB-SAFE-001A až 001D jsou lokálně ověřené. Commit TB-SAFE-001C "
            "ac8703e je na větvi feat/tb-safe-001; TB-SAFE-001D čeká na "
            "commit, push a plnou CI matici ROS 2 Jazzy.",
            styles["body"],
        ),
        _p(
            "Ověřené lokální příkazy: python scripts/verify_tb_safe_001.py, "
            "compileall a git diff --check.",
            styles["small"],
        ),
        Spacer(1, 8 * mm),
        _p(
            "Archivní výkaz pro milník TB-SAFE-001. Stav CI je záměrně "
            "uveden jako pending, dokud neproběhne nový push.",
            styles["small"],
        ),
    ]
    document.build(story, onFirstPage=_footer, onLaterPages=_footer)
    shutil.copyfile(OUTPUT_PDF, COMPAT_OUTPUT_PDF)
    return OUTPUT_PDF


if __name__ == "__main__":
    print(f"PDF successfully created at {build_pdf()}")
