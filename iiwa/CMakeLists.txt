cmake_minimum_required(VERSION 2.8.3)
project(iiwa)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation)

# Declare the message files to be built
add_message_files(FILES IIWAMsg.msg)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES	 ${PROJECT_NAME}
	CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/IIWARobot.cpp include/IIWARobot.h)

add_executable(${PROJECT_NAME}-bin src/main.cpp )

target_link_libraries(${PROJECT_NAME}-bin ${PROJECT_NAME} ${catkin_LIBRARIES})

FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
SET(QT_USE_QTOPENGL true)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${QT_INCLUDES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
