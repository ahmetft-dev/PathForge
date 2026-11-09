# Runs the CLI and checks the exact exit code and (optionally) a regular expression on stdout+stderr.
#   cmake -DCLI=<exe> -DEXPECT=<code> "-DARGS=a|b|c" [-DREGEX=<regex>] -P cli_expect.cmake
string(REPLACE "|" ";" ARGS "${ARGS}")
execute_process(COMMAND ${CLI} ${ARGS} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(NOT "${rc}" STREQUAL "${EXPECT}")
  message(FATAL_ERROR "expected exit code ${EXPECT}, got ${rc}\nargs: ${ARGS}\nstdout:\n${out}\nstderr:\n${err}")
endif()
if(DEFINED REGEX AND NOT "${out}${err}" MATCHES "${REGEX}")
  message(FATAL_ERROR "output does not match '${REGEX}'\nstdout:\n${out}\nstderr:\n${err}")
endif()
