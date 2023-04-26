# The list of compiler flags used in deepbreak can be prefixed with catkin config
# Addition flags are to be separated by \;
# For example, to turn on architecture specific optimizations:
#   catkin config --cmake-args -DDEEPBREAK_CXX_FLAGS=-march=native\;-mtune=native
list(APPEND DEEPBREAK_CXX_FLAGS
  "-march=native"
  "-mtune=native"
  "-fPIC"
  "-pthread"
  "-Wfatal-errors"
  "-Wl,--no-as-needed"
  )

# Force Boost dynamic linking
list(APPEND DEEPBREAK_CXX_FLAGS
  "-DBOOST_ALL_DYN_LINK"
  )

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
