# The host code-generation tools install a crash backtrace reporter
# (XenonUtils/crash_report.cpp). When building them for the cross-compile
# pass, keep debug symbols so a crash prints a readable stack in the build
# log (e.g. GitHub Actions) instead of a bare "Segmentation fault".
if (UNLEASHED_RECOMP_HOST_TOOLS_ONLY)
    if (MSVC)
        add_compile_options(/Zi)
    else()
        add_compile_options(-g -fno-omit-frame-pointer)
    endif()
endif()

add_subdirectory(${UNLEASHED_RECOMP_TOOLS_ROOT}/bc_diff)
add_subdirectory(${UNLEASHED_RECOMP_TOOLS_ROOT}/file_to_c)
add_subdirectory(${UNLEASHED_RECOMP_TOOLS_ROOT}/fshasher)
add_subdirectory(${UNLEASHED_RECOMP_TOOLS_ROOT}/x_decompress)
add_subdirectory(${UNLEASHED_RECOMP_TOOLS_ROOT}/XenonRecomp)
add_subdirectory(${UNLEASHED_RECOMP_TOOLS_ROOT}/XenosRecomp)
