file (GLOB msrc "${CMAKE_CURRENT_LIST_DIR}/*.c")
foreach (srcf ${msrc})
  target_sources(infra PRIVATE ${srcf})
endforeach()
