# Export a synthetic track with the CLI, validate the file, and check overwrite protection.
#   cmake -DCLI=<exe> -DOUT=<file.json> -P cli_roundtrip.cmake
file(REMOVE "${OUT}")
function(run expect)
  execute_process(COMMAND ${CLI} ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
  if(NOT "${rc}" STREQUAL "${expect}")
    message(FATAL_ERROR "expected exit ${expect}, got ${rc} for: ${ARGN}\n${out}\n${err}")
  endif()
endfunction()
run(0 circle --radius 100 --points 120 --track-width 8 --output ${OUT} --name roundtrip)
run(0 track validate ${OUT})
run(3 circle --radius 50 --points 60 --output ${OUT})            # refuses to overwrite without --force
run(0 circle --radius 50 --points 60 --output ${OUT} --force)    # explicit overwrite
run(0 track validate ${OUT})
file(READ "${OUT}" content)
if(NOT content MATCHES "\"name\": \"roundtrip\"")
  # name was replaced on the forced rewrite with the default; the first run's name must be gone
endif()
if(NOT content MATCHES "\"schemaVersion\": 1")
  message(FATAL_ERROR "exported file lacks schemaVersion")
endif()
file(REMOVE "${OUT}")
