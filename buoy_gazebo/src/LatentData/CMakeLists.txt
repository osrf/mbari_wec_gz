file(GLOB dirs LIST_DIRECTORIES true *)
foreach(dir ${dirs})
  if(IS_DIRECTORY ${dir})
    add_subdirectory(${dir})
  endif()
endforeach()
