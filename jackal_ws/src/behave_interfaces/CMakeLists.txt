cmake_minimum_required(VERSION 3.8)
project(behave_interfaces)

# 경고 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ✅ 필수 의존성
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ✅ 인터페이스 정의: 액션 파일 추가
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToGoal.action"
)

# ✅ 테스트 설정 (건드릴 필요 없음)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ✅ 런타임 의존성 export
ament_export_dependencies(rosidl_default_runtime)

ament_package()

