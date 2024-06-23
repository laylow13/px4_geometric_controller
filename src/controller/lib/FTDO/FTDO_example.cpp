#include "FTDO.h"
#include <iostream>
#include <fstream>
#include <array>
#include <Eigen/Dense>

int main() {
    array<double, 2> lambda{0.4, 0.1};
    double L =2;
    double dt = 0.1;
    FTDO<1, 2> observer(lambda, L, dt);
    Vector2d f{2, 1};
    double m = 1;
    Vector2d a{};
    Vector2d v{1, 2};
    Vector2d d{};

    // 设置初始状态估计
//    array<Vector2d, 4> z0 = {Vector2d(0.01, 0.01), Vector2d(0.01, 0.01), Vector2d::Zero(), Vector2d::Zero()};
//    observer.set_initial_estimation(z0);

    // 打开文件输出流
    std::ofstream file("output.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open output.csv for writing." << std::endl;
        return -1;
    }

    // 写入CSV文件头
    file << "Iteration,Actual v0,Actual v1,Estimated v0,Estimated v1,Actual d0,Actual d1,Estimated d0,Estimated d1\n";

    for (int i = 0; i < 200; i++) {
        d << 0.1, 0.2;
        a = (f + d) / m;
        v += a * dt;
        observer.update(f, v);
        array<Vector2d, 2> z{};
        observer.get_estimation(z);

//        // 输出到控制台
//        std::cout << "Iteration " << i << ":\n";
//        std::cout << "Actual velocity [v]: " << v.transpose() << "\n";
//        std::cout << "Estimated velocity [v_e]: " << z[0].transpose() << "\n";
//        std::cout << "Actual disturbance [d]: " << d.transpose() << "\n";
//        std::cout << "Estimated disturbance [d_e]: " << z[1].transpose() << "\n";
//        std::cout << "----------\n";

        // 写入CSV文件
        file << i << ","
             << v(0) << "," << v(1) << ","
             << z[0](0) << "," << z[0](1) << ","
             << d(0) << "," << d(1) << ","
             << z[1](0) << "," << z[1](1) << "\n";
    }

    // 关闭文件输出流
    file.close();

    return 0;
}
