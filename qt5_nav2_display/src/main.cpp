#include <QApplication>
#include <QMainWindow>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "qt5_nav2_display/nav2_view_widget.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);

    std::string map_yaml_path = ament_index_cpp::get_package_share_directory("qt5_nav2_display") + "/map/WHEELTEC.yaml";
    if (argc > 1) {
        map_yaml_path = argv[1];
    }

    auto node = std::make_shared<rclcpp::Node>("nav2_view_node");

    auto* widget = new Nav2ViewWidget(map_yaml_path, node);

    QMainWindow window;
    window.setCentralWidget(widget);
    window.setWindowTitle("Nav2 Display");
    window.show();

    std::thread spin_thread([&node]() {
        rclcpp::spin(node);
    });

    int result = app.exec();

    rclcpp::shutdown();
    spin_thread.join();

    return result;
}
