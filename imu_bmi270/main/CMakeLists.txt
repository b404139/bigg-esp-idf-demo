set(SOURCES 
            "main.c"
            "component/touch_button.c"
            )

set(INCLUDE_DIRS "." "component")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    )

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})