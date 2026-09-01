add_library(ramulator-translation OBJECT)

target_sources(
  ramulator-translation PRIVATE
  i_translation.h

  impl/no_translation.cpp
  impl/first_touch_page_coloring.cpp
)

target_link_libraries(
  ramulator
  PRIVATE
  ramulator-translation
)
