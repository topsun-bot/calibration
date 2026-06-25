#!/usr/bin/env python3
"""生成 OpenCV 兼容的 9×6 内角点棋盘格（A4 打印用 PNG/PDF）。"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / "config" / "patterns" / "chessboard_9x6_25mm.png"
DEFAULT_PDF = ROOT / "config" / "patterns" / "chessboard_9x6_25mm_a4.pdf"

A4_LANDSCAPE_MM = (297.0, 210.0)


def generate_chessboard(
    cols: int = 9,
    rows: int = 6,
    square_mm: float = 25.0,
    dpi: int = 300,
    paper_mm: tuple[float, float] = A4_LANDSCAPE_MM,
) -> np.ndarray:
    """cols/rows 为内角点数；物理方格边长 square_mm，默认放入 A4 横向整页。"""
    square_px = max(24, int(round(square_mm / 25.4 * dpi)))
    squares_x = cols + 1
    squares_y = rows + 1

    board_w = squares_x * square_px
    board_h = squares_y * square_px
    w = int(round(paper_mm[0] / 25.4 * dpi))
    h = int(round(paper_mm[1] / 25.4 * dpi))
    if board_w > w or board_h > h:
        raise ValueError(
            f"棋盘 {squares_x}x{squares_y} 格、{square_mm}mm 方格无法放入 "
            f"{paper_mm[0]}x{paper_mm[1]}mm 纸张"
        )

    x_offset = (w - board_w) // 2
    y_offset = (h - board_h) // 2
    img = np.full((h, w), 255, dtype=np.uint8)
    for row in range(squares_y):
        for col in range(squares_x):
            if (row + col) % 2 == 0:
                y0 = y_offset + row * square_px
                x0 = x_offset + col * square_px
                img[y0 : y0 + square_px, x0 : x0 + square_px] = 0
    return img


def save_with_dpi(img: np.ndarray, png_path: Path, pdf_path: Path | None, dpi: int) -> None:
    """cv2.imwrite 不写 DPI 元数据，这里用 Pillow 保存 PNG/PDF。"""
    pil = Image.fromarray(img)
    png_path.parent.mkdir(parents=True, exist_ok=True)
    pil.save(png_path, dpi=(dpi, dpi))
    if pdf_path is not None:
        pdf_path.parent.mkdir(parents=True, exist_ok=True)
        pil.convert("RGB").save(pdf_path, "PDF", resolution=float(dpi))


def main() -> None:
    parser = argparse.ArgumentParser(description="生成标定棋盘格 PNG")
    parser.add_argument("--cols", type=int, default=9, help="内角点列数")
    parser.add_argument("--rows", type=int, default=6, help="内角点行数")
    parser.add_argument("--square-mm", type=float, default=25.0, help="方格边长 (mm)")
    parser.add_argument("--dpi", type=int, default=300, help="打印分辨率")
    parser.add_argument("--pdf-output", type=Path, default=DEFAULT_PDF, help="同时输出 PDF（推荐打印）")
    parser.add_argument("--no-pdf", action="store_true", help="不生成 PDF")
    parser.add_argument("-o", "--output", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    img = generate_chessboard(args.cols, args.rows, args.square_mm, args.dpi)
    pdf_path = None if args.no_pdf else args.pdf_output
    save_with_dpi(img, args.output, pdf_path, args.dpi)
    print(f"已生成: {args.output.resolve()}")
    if pdf_path is not None:
        print(f"已生成: {pdf_path.resolve()}  (推荐打印此 PDF)")
    print(
        f"  内角点 {args.cols}x{args.rows}，方格 {args.square_mm} mm，"
        f"A4 横向 {img.shape[1]}x{img.shape[0]} px @ {args.dpi} DPI"
    )
    print("  打印 PDF 时请选择 100% / 实际大小，关闭“适应页面/缩放”")


if __name__ == "__main__":
    main()
