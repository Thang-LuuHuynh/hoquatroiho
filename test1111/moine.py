# Chương trình mô phỏng bộ lọc tín hiệu đơn giản
import random

# 1. Tạo dữ liệu gốc (ví dụ nhiệt độ ổn định ở 30 độ C)
goc = [30.0] * 20 

# 2. Thêm nhiễu vào tín hiệu (giả lập sai số cảm biến)
tin_hieu_nhieu = [val + random.uniform(-2, 2) for val in goc]

# 3. Hàm lọc nhiễu bằng phương pháp Trung bình trượt
def bo_loc_moving_average(data, window_size=3):
    ket_qua = []
    for i in range(len(data)):
        # Lấy trung bình của giá trị hiện tại và các giá trị trước đó
        start_idx = max(0, i - window_size + 1)
        window = data[start_idx : i + 1]
        trung_binh = sum(window) / len(window)
        ket_qua.append(round(trung_binh, 2))
    return ket_qua

# 4. Chạy bộ lọc
tin_hieu_sach = bo_loc_moving_average(tin_hieu_nhieu)

# 5. In kết quả ra màn hình
print(f"{'Gốc':<10} | {'Bị Nhiễu':<10} | {'Sau khi Lọc':<10}")
print("-" * 35)
for g, n, s in zip(goc, tin_hieu_nhieu, tin_hieu_sach):
    print(f"{g:<10} | {n:<10.2f} | {s:<10.2f}")
    print(f"{g:<10} | {n:<10.2f} | {s:<10.2f}")
