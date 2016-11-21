set(projectName tiffReader)
project (${projectName})

set(VTK_DIR "P:\\build\\vs64")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

set(src
    tiffReader
  )


add_executable(${projectName} ${src})
target_link_libraries(${projectName} ${VTK_LIBRARIES})
