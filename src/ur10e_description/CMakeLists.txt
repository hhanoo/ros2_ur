# CMake 최소 버전 요구사항 설정
cmake_minimum_required(VERSION 3.8)

# 프로젝트 이름 설정
project(ur10e_description)

# GCC 또는 Clang 컴파일러를 사용하는 경우 컴파일 옵션 추가
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)

# 추가 의존성 수동 추가 시
# find_package(<dependency> REQUIRED)

# launch, urdf, meshes 설치 추가
install(DIRECTORY
  launch
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# ROS2 패키지 설정 완료
ament_package()
