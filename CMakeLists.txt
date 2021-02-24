#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.


cmake_minimum_required(VERSION 2.8.12)
project(geom_inertia_estimator)

find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs
    nav_msgs
    sensor_msgs
    message_generation
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_message_files(
    FILES
    ParameterEstimates.msg
    MotorRPM.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy geometry_msgs nav_msgs sensor_msgs message_runtime
)