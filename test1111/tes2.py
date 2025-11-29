

# Vẽ lại sơ đồ tư duy trực tiếp (không lưu file ngoài)
dot = Digraph(comment="Mindmap Chương 4")
dot.attr(size="10", rankdir="LR", bgcolor="white")

# Node trung tâm
dot.node("Ch4", "Chương 4\nHoạch định & lập tiến độ dự án", shape="ellipse", style="filled", fillcolor="lightblue")

# Nhánh chính
dot.node("HDD", "1. Hoạch định dự án", shape="box", style="filled", fillcolor="lightyellow")
dot.edge("Ch4", "HDD")
dot.node("CC", "2. Công cụ tiến độ", shape="box", style="filled", fillcolor="lightyellow")
dot.edge("Ch4", "CC")
dot.node("DC", "3. Điều chỉnh tiến độ", shape="box", style="filled", fillcolor="lightyellow")
dot.edge("Ch4", "DC")
dot.node("DH", "4. Điều hòa nguồn lực", shape="box", style="filled", fillcolor="lightyellow")
dot.edge("Ch4", "DH")

# Xuất ra ảnh để hiển thị ngay
file_path = "/mnt/data/chuong4_mindmap_inline.png"
dot.render(file_path, format="png", cleanup=True)

file_path
