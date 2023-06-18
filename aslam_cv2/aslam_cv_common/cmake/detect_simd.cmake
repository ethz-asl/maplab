# Detect the preprocessor directives which are set by the compiler.
execute_process(COMMAND ${CMAKE_CXX_COMPILER} -march=native -dM -E -x c /dev/null
                OUTPUT_VARIABLE PREPROCESSOR_DIRECTIVES)

set(IS_SSE_ENABLED FALSE)
set(IS_NEON_ENABLED FALSE)
if (PREPROCESSOR_DIRECTIVES MATCHES "__SSSE3__")
  add_definitions(-mssse3)
  set(IS_SSE_ENABLED TRUE)
# For both armv7 and armv8, __ARM_NEON is used as preprocessor directive.
elseif (PREPROCESSOR_DIRECTIVES MATCHES "__ARM_ARCH 7")
  add_definitions(-mfpu=neon) # Needs to be set for armv7.
  set(IS_NEON_ENABLED TRUE)
elseif (PREPROCESSOR_DIRECTIVES MATCHES "__ARM_ARCH 8")
  set(IS_NEON_ENABLED TRUE)
else()
  message(WARNING "No SIMD instruction set detected.")
endif()

unset(PREPROCESSOR_DIRECTIVES CACHE)
