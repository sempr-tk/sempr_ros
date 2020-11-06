#include <ros/ros.h>
#include <sempr-gui/SemprGui.hpp>
#include "ROSConnectionClient.hpp"

#include <QApplication>

int main(int argc, char** args)
{
    ::ros::init(argc, args, "sempr_gui", ::ros::init_options::AnonymousName);

    QApplication app(argc, args);

    auto client = std::make_shared<sempr::ros::ROSConnectionClient>();
    sempr::gui::SemprGui gui(client);

    gui.show();

    ::ros::Rate rate(100);
    while (::ros::ok() && gui.isVisible())
    {
        ::ros::spinOnce();
        app.processEvents();
        rate.sleep();
    }
}
