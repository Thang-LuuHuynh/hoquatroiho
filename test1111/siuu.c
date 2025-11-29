#include <stdio.h>   // Thư viện chuẩn cho các hàm nhập/xuất (printf, scanf)
#include <stdlib.h>  // Thư viện chuẩn cho cấp phát bộ nhớ động (malloc, free)
#include <string.h>  // Thư viện chuẩn cho các hàm xử lý chuỗi (strcpy, strlen)

// Định nghĩa một cấu trúc (struct) để lưu thông tin sinh viên
typedef struct {
    char ma_sinh_vien[10]; // Mã sinh viên (VD: "SV001")
    char ho_ten[50];       // Họ và tên
    int tuoi;              // Tuổi
    float diem_trung_binh; // Điểm trung bình
} SinhVien;

// --- Các hàm nguyên mẫu (Function prototypes) ---
void nhap_thong_tin_sinh_vien(SinhVien *sv);
void in_thong_tin_sinh_vien(const SinhVien *sv);
void in_danh_sach_sinh_vien(const SinhVien **danh_sach, int so_luong);

int main() {
    int so_luong_sv;
    SinhVien **danh_sach_sinh_vien = NULL; // Con trỏ cấp 2 để lưu mảng con trỏ SinhVien

    printf("--- Chuong trinh Quan ly Sinh vien don gian ---\n");

    // Yêu cầu người dùng nhập số lượng sinh viên
    printf("Nhap so luong sinh vien ban muon quan ly: ");
    scanf("%d", &so_luong_sv);

    // Xóa bộ đệm bàn phím (để tránh lỗi khi dùng fgets sau scanf)
    while (getchar() != '\n');

    // Cấp phát bộ nhớ động cho mảng các con trỏ SinhVien
    // Kích thước là so_luong_sv * kích thước của một con trỏ SinhVien*
    danh_sach_sinh_vien = (SinhVien **)malloc(so_luong_sv * sizeof(SinhVien *));

    // Kiểm tra xem việc cấp phát bộ nhớ có thành công không
    if (danh_sach_sinh_vien == NULL) {
        printf("Loi: Khong du bo nho de cap phat danh sach sinh vien!\n");
        return 1; // Trả về mã lỗi
    }

    // Nhập thông tin cho từng sinh viên
    for (int i = 0; i < so_luong_sv; i++) {
        printf("\n--- Nhap thong tin cho Sinh vien thu %d ---\n", i + 1);

        // Cấp phát bộ nhớ động cho từng đối tượng SinhVien riêng lẻ
        danh_sach_sinh_vien[i] = (SinhVien *)malloc(sizeof(SinhVien));

        // Kiểm tra cấp phát
        if (danh_sach_sinh_vien[i] == NULL) {
            printf("Loi: Khong du bo nho de cap phat sinh vien thu %d!\n", i + 1);
            // Giải phóng bộ nhớ đã cấp phát trước đó nếu có lỗi
            for (int j = 0; j < i; j++) {
                free(danh_sach_sinh_vien[j]);
            }
            free(danh_sach_sinh_vien);
            return 1;
        }

        // Gọi hàm để nhập thông tin cho sinh viên hiện tại
        nhap_thong_tin_sinh_vien(danh_sach_sinh_vien[i]);
    }

    // In ra danh sách sinh viên đã nhập
    printf("\n--- Danh sach Sinh vien --- \n");
    in_danh_sach_sinh_vien((const SinhVien **)danh_sach_sinh_vien, so_luong_sv);

    // Giải phóng bộ nhớ đã cấp phát động (rất quan trọng để tránh memory leaks)
    // Giải phóng từng đối tượng SinhVien con trước
    for (int i = 0; i < so_luong_sv; i++) {
        free(danh_sach_sinh_vien[i]);
        danh_sach_sinh_vien[i] = NULL; // Đặt con trỏ về NULL sau khi giải phóng
    }
    // Sau đó giải phóng mảng các con trỏ
    free(danh_sach_sinh_vien);
    danh_sach_sinh_vien = NULL; // Đặt con trỏ về NULL sau khi giải phóng

    printf("\nChuong trinh ket thuc.\n");

    return 0; // Trả về 0 báo hiệu chương trình kết thúc thành công
}

// --- Định nghĩa các hàm ---

/**
 * @brief Hàm này dùng để nhập thông tin cho một sinh viên.
 * @param sv Con trỏ tới đối tượng SinhVien cần nhập.
 */
void nhap_thong_tin_sinh_vien(SinhVien *sv) {
    printf("   Ma sinh vien: ");
    // fgets an toàn hơn scanf khi đọc chuỗi, đọc cả khoảng trắng
    // stdin là luồng nhập chuẩn (bàn phím)
    fgets(sv->ma_sinh_vien, sizeof(sv->ma_sinh_vien), stdin);
    // Xóa ký tự xuống dòng cuối cùng nếu có
    sv->ma_sinh_vien[strcspn(sv->ma_sinh_vien, "\n")] = 0;

    printf("   Ho ten: ");
    fgets(sv->ho_ten, sizeof(sv->ho_ten), stdin);
    sv->ho_ten[strcspn(sv->ho_ten, "\n")] = 0;

    printf("   Tuoi: ");
    scanf("%d", &sv->tuoi);
    while (getchar() != '\n'); // Xóa bộ đệm bàn phím

    printf("   Diem trung binh: ");
    scanf("%f", &sv->diem_trung_binh);
    while (getchar() != '\n'); // Xóa bộ đệm bàn phím
}

/**
 * @brief Hàm này dùng để in thông tin của một sinh viên.
 * @param sv Con trỏ tới đối tượng SinhVien cần in. (const để báo hiệu hàm không thay đổi dữ liệu)
 */
void in_thong_tin_sinh_vien(const SinhVien *sv) {
    printf("   Ma SV: %s\n", sv->ma_sinh_vien);
    printf("   Ho ten: %s\n", sv->ho_ten);
    printf("   Tuoi: %d\n", sv->tuoi);
    printf("   Diem TB: %.2f\n", sv->diem_trung_binh); // .2f để định dạng 2 chữ số sau dấu thập phân
}

/**
 * @brief Hàm này dùng để in danh sách các sinh viên.
 * @param danh_sach Mảng các con trỏ tới đối tượng SinhVien.
 * @param so_luong Số lượng sinh viên trong danh sách.
 */
void in_danh_sach_sinh_vien(const SinhVien **danh_sach, int so_luong) {
    if (so_luong == 0) {
        printf("Danh sach rong.\n");
        return;
    }
    for (int i = 0; i < so_luong; i++) {
        printf("\nSinh vien thu %d:\n", i + 1);
        if (danh_sach[i] != NULL) { // Đảm bảo con trỏ không phải NULL trước khi truy cập
            in_thong_tin_sinh_vien(danh_sach[i]);
        } else {
            printf("   Du lieu sinh vien bi thieu hoac loi.\n");
        }
    }
}