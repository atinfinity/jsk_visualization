// -*- mode: c++ -*-
// rviz_rendering/render_window.hpp uses the plain Qt `slots` keyword and
// does not compile with QT_NO_KEYWORDS; provide the macro around the
// include.
#ifndef JSK_RVIZ_PLUGINS_RENDER_WINDOW_COMPAT_H_
#define JSK_RVIZ_PLUGINS_RENDER_WINDOW_COMPAT_H_

#if defined(QT_NO_KEYWORDS) && !defined(slots)
#define slots Q_SLOTS
#define JSK_RVIZ_PLUGINS_UNDEF_SLOTS
#endif

#include <rviz_rendering/render_window.hpp>

#ifdef JSK_RVIZ_PLUGINS_UNDEF_SLOTS
#undef slots
#undef JSK_RVIZ_PLUGINS_UNDEF_SLOTS
#endif

#endif
