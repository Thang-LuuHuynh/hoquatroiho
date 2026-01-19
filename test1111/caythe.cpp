#include <iostream>
#include <string>
#include <thread> // Thư viện để tạm dừng chương trình
#include <chrono> // Thư viện để tính thời gian

using namespace std;

int main() {
    float mucNuoc = 20.5;      // Mức nước hiện tại (m)
    float mucToiDa = 90.0;     // Ngưỡng dừng bơm
    bool mayBom = false;       // Trạng thái máy bơm (On/Off)

    cout << "--- HE THONG DIEU KHIEN BON NUOC TU DONG ---" << endl;

    // Vòng lặp mô phỏng hệ thống chạy liên tục
    for (int i = 0; i < 10; i++) {
        cout << "Muc nuoc hien tai: " << mucNuoc << "m" << endl;

        // Logic điều khiển (giống như lập trình PLC)
        if (mucNuoc < 30.0) {
            mayBom = true;
            cout << "=> Canh bao: Muc nuoc thap! Bat may bom." << endl;
        } 
        else if (mucNuoc >= mucToiDa) {
            mayBom = false;
            cout << "=> Thong bao: Bon da day. Tat may bom." << endl;
        }

        // Mô phỏng sự thay đổi mức nước
        if (mayBom) {
            mucNuoc += 15.0; // Nếu bơm đang chạy, nước tăng lên
        } else {
            mucNuoc -= 5.0;  // Nếu bơm tắt, nước giảm do sử dụng
        }

        cout << "----------------------------------------" << endl;
        
        // Tạm dừng 1 giây để dễ quan sát (giống Scan Time của PLC)
        this_thread::sleep_for(chrono::seconds(1));
    }

    cout << "Ket thuc mo phong." << endl;
    return 0;
}