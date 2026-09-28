cmake_minimum_required(VERSION 3.5)
project(mycobot_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz config meshes
  DESTINATION share/${PROJECT_NAME}
)

# arbeitsraum_marker.py zeichnet die GEMESSENE Reichweitengrenze in RViz.
# Die Messung selbst steht in config/arbeitsraum_ring.json und wird von
# tools/arbeitsraum_grenze.py erzeugt.
# arbeitsraum_wolke.py beschneidet die Kamera-Punktwolke auf die Grundplatte (RViz).
install(PROGRAMS scripts/arbeitsraum_marker.py scripts/arbeitsraum_wolke.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
