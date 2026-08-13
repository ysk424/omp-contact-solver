cmake_minimum_required(VERSION 3.20)

project(omp_contact_solver VERSION 0.1.0 LANGUAGES C CXX)

option(OCS_BUILD_TESTS "Build the solver tests" ON)
option(OCS_BUILD_EXAMPLES "Build the examples" ON)
option(OCS_BUILD_BLENDER_EXTENSION "Enable Blender Extension staging" ON)
set(OCS_BLENDER_EXECUTABLE "" CACHE FILEPATH
    "Blender executable used to package and test the Extension")

find_package(OpenMP REQUIRED COMPONENTS CXX)

add_library(omp_contact_solver SHARED
    src/api.cpp
    src/solver.cpp
)
add_library(omp_contact_solver::omp_contact_solver ALIAS omp_contact_solver)

target_compile_features(omp_contact_solver PUBLIC cxx_std_17)
target_compile_definitions(omp_contact_solver PRIVATE OCS_BUILD_DLL)
target_include_directories(omp_contact_solver
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
set_target_properties(omp_contact_solver PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    WINDOWS_EXPORT_ALL_SYMBOLS NO
)
if(WIN32)
    set_target_properties(omp_contact_solver PROPERTIES PREFIX "")
endif()

if(MSVC)
    target_compile_options(omp_contact_solver PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(omp_contact_solver PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(MINGW)
    # Produce one distributable solver DLL. FindOpenMP normally selects the
    # libgomp import library; the same toolchain also provides the static
    # archive and pthread implementation used here.
    get_filename_component(OCS_GOMP_DIR "${OpenMP_gomp_LIBRARY}" DIRECTORY)
    get_filename_component(OCS_THREAD_DIR "${OpenMP_mingwthrd_LIBRARY}" DIRECTORY)
    set(OCS_GOMP_STATIC "${OCS_GOMP_DIR}/libgomp.a")
    set(OCS_PTHREAD_STATIC "${OCS_THREAD_DIR}/libwinpthread.a")
    set(OCS_DL_STATIC "${OCS_THREAD_DIR}/libdl.a")
    if(NOT EXISTS "${OCS_GOMP_STATIC}" OR NOT EXISTS "${OCS_PTHREAD_STATIC}"
       OR NOT EXISTS "${OCS_DL_STATIC}")
        message(FATAL_ERROR "MinGW static OpenMP runtime archives were not found")
    endif()
    target_compile_options(omp_contact_solver PRIVATE ${OpenMP_CXX_FLAGS})
    target_link_libraries(omp_contact_solver PRIVATE
        "${OCS_GOMP_STATIC}"
        "${OCS_DL_STATIC}"
        "${OpenMP_mingwthrd_LIBRARY}"
        "${OCS_PTHREAD_STATIC}"
    )
    target_link_options(omp_contact_solver PRIVATE -static-libgcc -static-libstdc++)
else()
    target_link_libraries(omp_contact_solver PRIVATE OpenMP::OpenMP_CXX)
endif()

if(OCS_BUILD_TESTS)
    enable_testing()
    add_executable(omp_contact_solver_tests tests/test_solver.cpp)
    target_compile_features(omp_contact_solver_tests PRIVATE cxx_std_17)
    target_link_libraries(omp_contact_solver_tests PRIVATE omp_contact_solver)
    add_test(NAME omp_contact_solver_tests COMMAND omp_contact_solver_tests)

    add_executable(omp_contact_solver_c_api_test tests/test_c_api.c)
    target_compile_features(omp_contact_solver_c_api_test PRIVATE c_std_11)
    target_link_libraries(omp_contact_solver_c_api_test PRIVATE omp_contact_solver)
    add_test(NAME omp_contact_solver_c_api_test COMMAND omp_contact_solver_c_api_test)

    add_executable(omp_contact_solver_visual_test tests/visualize_tests.cpp)
    target_compile_features(omp_contact_solver_visual_test PRIVATE cxx_std_17)
    target_link_libraries(omp_contact_solver_visual_test PRIVATE omp_contact_solver)
    set(OCS_VISUAL_TEST_DIR "${CMAKE_BINARY_DIR}/visual-tests")
    add_test(NAME omp_contact_solver_visual_test
             COMMAND omp_contact_solver_visual_test "${OCS_VISUAL_TEST_DIR}")
    set_tests_properties(omp_contact_solver_visual_test PROPERTIES LABELS visual)
    add_custom_target(visual-test-report
        COMMAND omp_contact_solver_visual_test "${OCS_VISUAL_TEST_DIR}"
        DEPENDS omp_contact_solver_visual_test
        BYPRODUCTS
            "${OCS_VISUAL_TEST_DIR}/index.html"
            "${OCS_VISUAL_TEST_DIR}/final_state.obj"
            "${OCS_VISUAL_TEST_DIR}/summary.json"
            "${OCS_VISUAL_TEST_DIR}/animation.json"
        COMMENT "Generating self-contained visual test report"
        VERBATIM
    )
endif()

if(OCS_BUILD_EXAMPLES)
    add_executable(drop_cloth examples/drop_cloth.cpp)
    target_compile_features(drop_cloth PRIVATE cxx_std_17)
    target_link_libraries(drop_cloth PRIVATE omp_contact_solver)
endif()

if(OCS_BUILD_BLENDER_EXTENSION AND WIN32)
    set(OCS_EXTENSION_SOURCE
        "${CMAKE_CURRENT_SOURCE_DIR}/blender_extension/omp_contact_solver")
    set(OCS_EXTENSION_STAGE
        "${CMAKE_BINARY_DIR}/blender-extension/omp_contact_solver")
    set(OCS_EXTENSION_PACKAGE_DIR "${CMAKE_BINARY_DIR}/packages")
    set(OCS_EXTENSION_PACKAGE
        "${OCS_EXTENSION_PACKAGE_DIR}/omp_contact_solver-0.1.0-windows-x64.zip")
    file(GLOB OCS_EXTENSION_FILES CONFIGURE_DEPENDS
        "${OCS_EXTENSION_SOURCE}/*.py"
        "${OCS_EXTENSION_SOURCE}/*.toml"
        "${OCS_EXTENSION_SOURCE}/README.md"
        "${OCS_EXTENSION_SOURCE}/LICENSE")

    add_custom_command(
        OUTPUT "${OCS_EXTENSION_STAGE}/.stage-complete"
        COMMAND "${CMAKE_COMMAND}"
            "-DOCS_EXTENSION_SOURCE=${OCS_EXTENSION_SOURCE}"
            "-DOCS_EXTENSION_STAGE=${OCS_EXTENSION_STAGE}"
            "-DOCS_SOLVER_LIBRARY=$<TARGET_FILE:omp_contact_solver>"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/stage_blender_extension.cmake"
        DEPENDS omp_contact_solver ${OCS_EXTENSION_FILES}
        BYPRODUCTS
            "${OCS_EXTENSION_STAGE}/blender_manifest.toml"
            "${OCS_EXTENSION_STAGE}/bin/omp_contact_solver.dll"
        COMMENT "Staging the Blender Extension with its solver DLL"
        VERBATIM
    )
    add_custom_target(stage-blender-extension
        DEPENDS "${OCS_EXTENSION_STAGE}/.stage-complete")

    if(OCS_BLENDER_EXECUTABLE)
        if(NOT EXISTS "${OCS_BLENDER_EXECUTABLE}")
            message(FATAL_ERROR
                "OCS_BLENDER_EXECUTABLE does not exist: ${OCS_BLENDER_EXECUTABLE}")
        endif()
        add_custom_target(blender-extension
            COMMAND "${CMAKE_COMMAND}" -E make_directory
                    "${OCS_EXTENSION_PACKAGE_DIR}"
            COMMAND "${OCS_BLENDER_EXECUTABLE}" --command extension build
                    --source-dir "${OCS_EXTENSION_STAGE}"
                    --output-filepath "${OCS_EXTENSION_PACKAGE}"
            DEPENDS stage-blender-extension
            BYPRODUCTS "${OCS_EXTENSION_PACKAGE}"
            COMMENT "Building the installable Blender Extension package"
            VERBATIM
        )
        if(OCS_BUILD_TESTS)
            add_custom_target(blender-extension-test
                COMMAND "${OCS_BLENDER_EXECUTABLE}"
                        --background --factory-startup
                        --python
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/blender_extension_smoke.py"
                        -- --extension-stage "${OCS_EXTENSION_STAGE}"
                DEPENDS stage-blender-extension
                COMMENT "Running the Blender Extension smoke test"
                VERBATIM
            )
        endif()
    else()
        message(STATUS
            "Set OCS_BLENDER_EXECUTABLE to enable blender-extension packaging and tests")
    endif()
elseif(OCS_BUILD_BLENDER_EXTENSION)
    message(STATUS "The bundled Blender Extension currently targets Windows x64")
endif()

include(GNUInstallDirs)
install(TARGETS omp_contact_solver
    EXPORT omp_contact_solver_targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES include/omp_contact_solver.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT omp_contact_solver_targets
    FILE omp_contact_solverTargets.cmake
    NAMESPACE omp_contact_solver::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/omp_contact_solver
)
