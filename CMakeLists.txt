
# ============================================================
#                VERSIÓN MÍNIMA DE CMAKE
# ============================================================

cmake_minimum_required(VERSION 3.8)


# ============================================================
#                 NOMBRE DEL PAQUETE
# ============================================================

project(scorbot_ik_trajectories)


# ============================================================
#              OPCIONES DE COMPILACIÓN
# ============================================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(-Wall -Wextra -Wpedantic)
  # Activa advertencias del compilador

endif()


# ============================================================
#                  DEPENDENCIAS DEL PAQUETE
# ============================================================

find_package(ament_cmake REQUIRED)
# Dependencia principal para paquetes tipo ament_cmake


# ============================================================
#                 PRUEBAS Y LINTERS
# ============================================================

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  # Evita revisión de copyright durante pruebas

  set(ament_cmake_cpplint_FOUND TRUE)
  # Evita revisión de cpplint si no estás usando git

  ament_lint_auto_find_test_dependencies()

endif()


# ============================================================
#             INSTALACIÓN DE CARPETAS DEL PAQUETE
# ============================================================

install(
  DIRECTORY urdf meshes launch rviz config
  # Carpetas que se copiarán al espacio install

  DESTINATION share/${PROJECT_NAME}
  # Ruta final: install/scorbot_description/share/scorbot_description
)

install(PROGRAMS
  scripts/scorbot_ik_publisher.py
  scripts/scorbot_trajectory_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  )
# ============================================================
#                  FINALIZACIÓN DEL PAQUETE
# ============================================================

ament_package()
# Declara el paquete como paquete ROS 2 tipo ament