#!/usr/bin/env python3
"""Export the generated HW4 Markdown to PDF using ReportLab and rsvg-convert."""

import argparse
import io
import re
import subprocess
from pathlib import Path
from xml.sax.saxutils import escape

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.platypus import Image, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

INLINE = re.compile(r"(\*\*[^*]+\*\*|\*[^*]+\*|\[[^\]]+\]\(https?://[^)]+\))")
BLUE = colors.HexColor("#17628c")


ESCAPED_STAR = "\x00"  # placeholder for a literal * written as \* in the Markdown


def markup(text):
    text = text.replace("\\*", ESCAPED_STAR)
    parts = []
    for index, part in enumerate(INLINE.split(text)):
        if index % 2 == 0:
            parts.append(escape(part))
        elif part.startswith("**"):
            parts.append(f"<b>{escape(part[2:-2])}</b>")
        elif part.startswith("*"):
            parts.append(f"<i>{escape(part[1:-1])}</i>")
        else:
            label, url = part[1:-1].split("](", 1)
            parts.append(f'<a href="{escape(url, {chr(34): "&quot;"})}" color="#17628c">{escape(label)}</a>')
    return "".join(parts).replace(ESCAPED_STAR, "*")


def styles():
    base = getSampleStyleSheet()
    for name, font, size, leading in (
        ("Title", "Times-Bold", 24, 27),
        ("Heading2", "Times-Bold", 15, 18),
        ("Heading3", "Times-Bold", 12, 15),
        ("BodyText", "Times-Roman", 11, 14),
    ):
        style = base[name]
        style.fontName = font
        style.fontSize = size
        style.leading = leading
        style.textColor = colors.HexColor("#182431")
        style.alignment = TA_LEFT
        style.spaceAfter = 6
    base.add(ParagraphStyle("Cell", parent=base["BodyText"], fontSize=9, leading=11, spaceAfter=0))
    return base


def table(lines, style, width):
    rows = [
        [Paragraph(markup(cell.strip()), style["Cell"]) for cell in line.strip("|").split("|")]
        for line in (lines[:1] + lines[2:])
    ]
    columns = len(rows[0])
    weights = [1.45] + [1] * (columns - 1)
    output = Table(rows, colWidths=[width * w / sum(weights) for w in weights], repeatRows=1)
    output.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e5eef4")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LINEBELOW", (0, 0), (-1, -1), 0.3, colors.HexColor("#c0ccd7")),
    ]))
    return output


def figure(directory, name, label):
    image_path = directory / name
    png = subprocess.run(
        ["rsvg-convert", "--width", "1100", str(image_path)], check=True, capture_output=True
    ).stdout
    image = Image(io.BytesIO(png), width=335, height=335)
    image.hAlign = "CENTER"
    image.keepWithNext = label.startswith("Arena")
    return image


def export(source, destination):
    style = styles()
    margin = 51
    document = SimpleDocTemplate(
        str(destination), pagesize=letter, leftMargin=margin, rightMargin=margin,
        topMargin=42, bottomMargin=40, title="CSE 276A Homework 4: Path Planning",
        author="Fong-Yu Lin",
    )
    blocks = re.split(r"\n\s*\n", source.read_text().strip())
    flow = []
    for block in blocks:
        lines = block.splitlines()
        if lines[0].startswith("|"):
            flow.extend((table(lines, style, document.width - 12), Spacer(1, 6)))
            continue
        if all(line.startswith("**") for line in lines):
            flow.append(Paragraph("<br/>".join(markup(line.rstrip()) for line in lines), style["BodyText"]))
            continue
        for line in lines if lines[0].startswith("#") else [" ".join(lines)]:
            heading = re.match(r"^(#{1,3}) (.+)$", line)
            img = re.fullmatch(r"!\[([^\]]+)\]\(([^)]+)\)", line)
            if heading:
                name = {1: "Title", 2: "Heading2", 3: "Heading3"}[len(heading[1])]
                flow.append(Paragraph(markup(heading[2]), style[name]))
            elif img:
                flow.append(figure(source.parent, img[2], img[1]))
            else:
                flow.append(Paragraph(markup(line), style["BodyText"]))

    def footer(canvas, doc):
        canvas.setFillColor(BLUE)
        canvas.setFont("Times-Roman", 9)
        canvas.drawCentredString(letter[0] / 2, 24, str(doc.page))

    document.build(flow, onFirstPage=footer, onLaterPages=footer)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("markdown", type=Path)
    parser.add_argument("pdf", type=Path)
    args = parser.parse_args()
    export(args.markdown, args.pdf)


if __name__ == "__main__":
    main()
