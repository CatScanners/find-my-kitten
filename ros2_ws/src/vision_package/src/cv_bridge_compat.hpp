#pragma once

// This header is needed due to cv_bridge shipped with ROS2 Humble and Jazzy
// having different header extensions. Humble has .h, Jazzy has .hpp
// If we move to Jazzy, this should be removed and swappedd to just
// including <cv_bridge/cv_bridge.hpp> directly

#if defined(__has_include)
  #if __has_include(<cv_bridge/cv_bridge.hpp>)
    #include <cv_bridge/cv_bridge.hpp>
  #elif __has_include(<cv_bridge/cv_bridge.h>)
    #include <cv_bridge/cv_bridge.h>
  #else
    #error "cv_bridge header not found. Install cv_bridge or fix include paths."
  #endif
#else
  #include <cv_bridge/cv_bridge.hpp>
#endif
