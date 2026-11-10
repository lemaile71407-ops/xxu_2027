# 为当前目录定义的 C++ 目标统一启用编译诊断。
include_guard(GLOBAL)

option(LOCALIZATION_WARNINGS_AS_ERRORS "将项目代码的编译警告视为错误" ON)

# 覆盖库、节点、工具和测试，不修改依赖子目录中的目标。
function(localization_enable_warnings)
    get_property(targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
    foreach(target IN LISTS targets)
        get_target_property(type ${target} TYPE)
        if(NOT type MATCHES "^(EXECUTABLE|STATIC_LIBRARY|SHARED_LIBRARY|MODULE_LIBRARY|OBJECT_LIBRARY)$")
            continue()
        endif()
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
            if(LOCALIZATION_WARNINGS_AS_ERRORS)
                target_compile_options(${target} PRIVATE -Werror)
                # GCC 13 对 Eigen/std::variant 报告的该类警告保留输出，避免 Jazzy 默认构建中断。
                if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND
                   CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 13 AND
                   CMAKE_CXX_COMPILER_VERSION VERSION_LESS 14)
                    target_compile_options(${target} PRIVATE -Wno-error=maybe-uninitialized)
                endif()
            endif()
        elseif(MSVC)
            target_compile_options(${target} PRIVATE /W4)
            if(LOCALIZATION_WARNINGS_AS_ERRORS)
                target_compile_options(${target} PRIVATE /WX)
            endif()
        endif()
    endforeach()
endfunction()
