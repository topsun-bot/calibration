# 标定棋盘格（打印用）

| 文件 | 说明 |
|------|------|
| `chessboard_9x6_25mm_a4.pdf` | 推荐打印：A4 横向，9×6 **内角点**，方格边长 **25 mm** |
| `chessboard_9x6_25mm.png` | 同内容 PNG，带 300 DPI 元数据 |

## 打印

1. 优先打开 `chessboard_9x6_25mm_a4.pdf` 打印。
2. 打印设置选择 **A4 横向**、**实际大小/100%**，关闭 **适应页面/缩放**。
3. 打印后用尺子抽查：**每个方格边长应为 25 mm**。
3. 贴在硬纸板/亚克力上，夹到 D1 夹爪上用于眼在手外标定。

## 重新生成

```bash
python3 calibate/tools/generate_chessboard.py
# 自定义输出
python3 calibate/tools/generate_chessboard.py -o calibate/config/patterns/my_board.png --square-mm 25
```

如果打印软件提示“不缩放显示不全”，通常是图片 DPI/纸张边距问题。请改用本目录 PDF；它是 A4 横向整页模板，棋盘居中，黑白方格区域约 250 × 175 mm，周围留有 A4 纸边距。
